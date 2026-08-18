"""
Drone Pad API Service — Docking Controller

Talks to the ESP32 dock-lock controller (see
https://github.com/onlinecodewithme/drone_dock_locking_system) over BLE.
Sends DOCK / UNDOCK commands on the command characteristic and tracks
progress via notifications on the status characteristic until a terminal
status (*_COMPLETE or ERROR) arrives or the command times out.

This module is intentionally standalone (async, no GPIO/thread coupling)
so it can be exercised directly against real hardware. Wiring it into
PadStateMachine is a separate follow-up.

start()/stop() run a persistent auto-reconnect loop as a background
asyncio task, so callers (e.g. the FastAPI app) can hold one long-lived
DockingController for the life of the process and always have a
connection available for commands and status.
"""

from __future__ import annotations

import asyncio
import logging
from typing import Optional

from config import DockingConfig, DockStatus, Settings

logger = logging.getLogger(__name__)

try:
    from bleak import BleakClient
    _HAS_BLEAK = True
except ImportError:
    BleakClient = None  # type: ignore[assignment,misc]
    _HAS_BLEAK = False


# Status strings that end a DOCK/UNDOCK command (success or failure). The
# firmware transitions directly into these stable resting states — there's
# no separate transient "*_COMPLETE" status that then reverts to idle.
_TERMINAL_STATUSES = {
    DockStatus.DOCKED.value,
    DockStatus.UNDOCKED.value,
    DockStatus.ERROR.value,
}

# Auto-reconnect backoff (the ESP32 re-advertises on disconnect, so a plain
# retry loop is enough — no need for anything fancier).
_MIN_RECONNECT_DELAY_SECONDS = 2.0
_MAX_RECONNECT_DELAY_SECONDS = 30.0
_CONNECTION_POLL_SECONDS = 5.0


class DockingError(Exception):
    """Base class for docking controller errors."""
    pass


class DockingConnectionError(DockingError):
    """Raised when not connected to the dock controller, or connect fails."""
    pass


class DockingTimeoutError(DockingError):
    """Raised when no terminal status arrives within the command timeout."""
    pass


class DockingController:
    """
    BLE client for the ESP32 dock-lock controller.

    Usage:
        async with DockingController(settings) as dock:
            status = await dock.undock()
    """

    def __init__(self, config: DockingConfig) -> None:
        if not _HAS_BLEAK:
            raise ImportError(
                "bleak is required for DockingController. Install it with: pip install bleak"
            )
        self._config = config
        self._client: Optional[BleakClient] = None
        self._status = DockStatus.UNKNOWN
        self._terminal_status: Optional[DockStatus] = None
        self._command_done = asyncio.Event()
        self._command_lock = asyncio.Lock()
        self._reconnect_task: Optional[asyncio.Task] = None

    @classmethod
    def from_settings(cls, settings: Settings) -> "DockingController":
        return cls(settings.docking_config)

    @property
    def is_connected(self) -> bool:
        return self._client is not None and self._client.is_connected

    @property
    def is_busy(self) -> bool:
        """Whether a dock/undock command is currently in flight."""
        return self._command_lock.locked()

    @property
    def last_status(self) -> DockStatus:
        return self._status

    @property
    def device_mac(self) -> str:
        return self._config.device_mac

    async def __aenter__(self) -> "DockingController":
        await self.connect()
        return self

    async def __aexit__(self, *exc_info) -> None:
        await self.disconnect()

    # ----- Connection -----

    async def connect(self) -> None:
        """Make one connection attempt and subscribe to status notifications."""
        if self.is_connected:
            return

        logger.info(
            "Connecting to dock controller %s (timeout=%.1fs)",
            self._config.device_mac, self._config.connect_timeout_seconds,
        )
        client = BleakClient(
            self._config.device_mac,
            timeout=self._config.connect_timeout_seconds,
            disconnected_callback=self._handle_disconnect,
        )
        try:
            # BleakClient's own `timeout=` is passed to the underlying
            # BlueZ/D-Bus backend, which doesn't reliably honor it — a
            # wedged D-Bus call can leave client.connect() hanging
            # indefinitely with no exception ever raised. Since
            # _connection_loop() is a single sequential task, a hang here
            # doesn't just make one attempt slow — it freezes every future
            # reconnect attempt too, silently, with no log output, for as
            # long as the hang lasts. Enforce the timeout from our side so
            # a stuck attempt always fails and lets the backoff loop retry.
            await asyncio.wait_for(
                client.connect(), timeout=self._config.connect_timeout_seconds + 5.0
            )
            await client.start_notify(self._config.status_uuid, self._on_status_notify)
        except Exception as exc:
            # Best-effort cleanup of a half-opened client before raising —
            # otherwise a timed-out attempt can leave a dangling connection
            # object behind for the next attempt to trip over.
            try:
                await client.disconnect()
            except Exception:
                pass
            raise DockingConnectionError(
                f"Failed to connect to dock controller at {self._config.device_mac}: {exc}"
            ) from exc

        self._client = client
        logger.info("Connected to dock controller")

        # Notifications only fire on a state *change*, and the firmware
        # deliberately suppresses the very first one after boot — so seed
        # our cached status with an active read instead of waiting for a
        # notify that may never come (e.g. right after boot or RESET).
        try:
            self._status = await self.get_status()
        except DockingConnectionError:
            pass

    async def disconnect(self) -> None:
        """Disconnect from the dock controller, if connected."""
        if self._client is None:
            return
        try:
            if self._client.is_connected:
                try:
                    await self._client.stop_notify(self._config.status_uuid)
                except Exception:
                    pass
                await self._client.disconnect()
                logger.info("Disconnected from dock controller")
        finally:
            self._client = None

    def _handle_disconnect(self, _client: "BleakClient") -> None:
        """bleak calls this from the event loop on an unexpected BLE drop."""
        logger.warning("Lost BLE connection to dock controller — will auto-reconnect")

    async def start(self) -> None:
        """Start the background auto-reconnect loop (call once at app startup)."""
        if self._reconnect_task is not None:
            return
        self._reconnect_task = asyncio.create_task(
            self._connection_loop(), name="dock-ble-reconnect"
        )

    async def stop(self) -> None:
        """Stop the auto-reconnect loop and disconnect (call at app shutdown)."""
        task, self._reconnect_task = self._reconnect_task, None
        if task is not None:
            task.cancel()
            try:
                await task
            except asyncio.CancelledError:
                pass
        await self.disconnect()

    async def _connection_loop(self) -> None:
        """Keep the BLE link up for the life of the app, reconnecting on drop."""
        delay = _MIN_RECONNECT_DELAY_SECONDS
        while True:
            if not self.is_connected:
                try:
                    await self.connect()
                    delay = _MIN_RECONNECT_DELAY_SECONDS
                except DockingConnectionError as exc:
                    logger.warning(
                        "Dock controller connect failed: %s — retrying in %.0fs", exc, delay,
                    )
                    await asyncio.sleep(delay)
                    delay = min(delay * 2, _MAX_RECONNECT_DELAY_SECONDS)
                    continue
            await asyncio.sleep(_CONNECTION_POLL_SECONDS)

    # ----- Commands -----

    async def dock(self, timeout: Optional[float] = None) -> DockStatus:
        """Send DOCK and wait for DOCKED (or ERROR)."""
        return await self._execute_command("DOCK", timeout=timeout)

    async def undock(self, timeout: Optional[float] = None) -> DockStatus:
        """Send UNDOCK and wait for UNDOCKED (or ERROR)."""
        return await self._execute_command("UNDOCK", timeout=timeout)

    async def reset(self) -> None:
        """Send RESET — stops all motors and re-derives DOCKED/UNDOCKED/UNKNOWN from the limit switches."""
        await self._write_command("RESET")

    async def get_status(self) -> DockStatus:
        """Read the current status characteristic value directly."""
        if not self.is_connected:
            raise DockingConnectionError("Not connected to dock controller")
        try:
            raw = await self._client.read_gatt_char(self._config.status_uuid)
        except Exception as exc:
            raise DockingConnectionError(f"Failed to read dock status: {exc}") from exc
        return self._parse_status(raw)

    # ----- Internals -----

    async def _write_command(self, command: str) -> None:
        if not self.is_connected:
            raise DockingConnectionError("Not connected to dock controller")
        try:
            await self._client.write_gatt_char(self._config.command_uuid, command.encode("utf-8"))
        except Exception as exc:
            raise DockingConnectionError(f"Failed to send {command}: {exc}") from exc
        logger.info("Sent %s command", command)

    async def _execute_command(self, command: str, timeout: Optional[float]) -> DockStatus:
        timeout = timeout if timeout is not None else self._config.command_timeout_seconds

        async with self._command_lock:
            self._command_done.clear()
            self._terminal_status = None
            await self._write_command(command)

            try:
                await asyncio.wait_for(self._command_done.wait(), timeout=timeout)
            except asyncio.TimeoutError as exc:
                raise DockingTimeoutError(
                    f"No terminal status received within {timeout:.1f}s after {command}"
                ) from exc

            # Read the status captured at the moment it went terminal, not
            # self._status — the firmware's passive rest-state re-verification
            # can follow DOCKED/UNDOCKED with a near-immediate second
            # notification (e.g. a momentary UNKNOWN from a switch-read
            # blip), which would otherwise overwrite it before this
            # coroutine gets scheduled again.
            result = self._terminal_status
            if result == DockStatus.ERROR:
                raise DockingError(f"{command} failed — dock controller reported ERROR")

            return result

    def _on_status_notify(self, _sender, data: bytearray) -> None:
        status = self._parse_status(data)
        self._status = status
        logger.info("Dock status: %s", status.value)
        if status.value in _TERMINAL_STATUSES:
            self._terminal_status = status
            self._command_done.set()

    @staticmethod
    def _parse_status(data: bytes) -> DockStatus:
        raw = data.decode("utf-8", errors="replace").strip()
        try:
            return DockStatus(raw)
        except ValueError:
            logger.warning("Unknown status string from dock controller: %r", raw)
            return DockStatus.UNKNOWN


# ---------------------------------------------------------------------------
# CLI — quick manual test against real hardware
# ---------------------------------------------------------------------------

async def _cli_main(args) -> int:
    from config import load_settings
    from dataclasses import replace

    settings = load_settings()
    docking_config = settings.docking_config
    if args.mac:
        docking_config = replace(docking_config, device_mac=args.mac)

    controller = DockingController(docking_config)
    try:
        await controller.connect()

        if args.command == "status":
            status = await controller.get_status()
            print(f"Status: {status.value}")
        elif args.command == "dock":
            status = await controller.dock()
            print(f"Dock result: {status.value}")
        elif args.command == "undock":
            status = await controller.undock()
            print(f"Undock result: {status.value}")
        elif args.command == "reset":
            await controller.reset()
            print("RESET sent")

        return 0
    except DockingError as exc:
        print(f"Error: {exc}")
        return 1
    finally:
        await controller.disconnect()


if __name__ == "__main__":
    import argparse
    import sys

    parser = argparse.ArgumentParser(description="Manually test the BLE dock-lock controller")
    parser.add_argument("command", choices=["dock", "undock", "status", "reset"])
    parser.add_argument("--mac", default=None, help="Override the dock controller's BLE MAC address")
    parser.add_argument("-v", "--verbose", action="store_true", help="Enable debug logging")
    cli_args = parser.parse_args()

    logging.basicConfig(
        level=logging.DEBUG if cli_args.verbose else logging.INFO,
        format="%(asctime)s | %(levelname)-8s | %(name)-20s | %(message)s",
        datefmt="%Y-%m-%d %H:%M:%S",
    )

    sys.exit(asyncio.run(_cli_main(cli_args)))
