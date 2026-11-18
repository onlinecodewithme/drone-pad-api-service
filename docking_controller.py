"""
Drone Pad API Service — Docking Controller (USB serial)

Talks to the ESP32 dock-lock controller (firmware:
https://github.com/onlinecodewithme/drone_dock_locking_system) over its USB
serial link (CP2102 USB-UART, 115200 8N1). BLE is no longer used.

Line protocol, one '\\n'-terminated line each:

    Pi -> ESP32   DOCK | UNDOCK | PROPCYCLE | RESET | STATUS
    ESP32 -> Pi   STATE:<state>                  on every state change, and once at boot
                  [STATUS] State=<state> ...     reply to STATUS
                  anything else                  human-readable firmware log

A DOCK/UNDOCK finishes when the firmware reaches a stable resting state
(DOCKED / UNDOCKED) or ERROR. It only counts as success if it ends in the
state the command was meant to reach.

PROPCYCLE runs only the propeller closer (open, then close) and never moves
the docking motors M1/M2, so it can end in DOCKED, UNDOCKED or UNKNOWN. It
counts as success only if a STATUS read afterwards shows the propeller
closer's close limit switch triggered.

start()/stop() run a persistent auto-reconnect loop as a background asyncio
task, so the FastAPI app can hold one long-lived DockingController and always
have a link available. A dedicated reader thread owns the blocking serial
reads and hands each line to the event loop.

Opening the port must not reset the ESP32. Its dev board ties EN/GPIO0 to
RTS/DTR through an auto-reset circuit that only fires when the two lines
differ. The kernel raises both together on open and drops both together on
close, and pyserial's defaults (dtr=True, rts=True) leave them that way. Do
not toggle DTR/RTS individually on this port.
"""

from __future__ import annotations

import asyncio
import logging
import os
import threading
from typing import Awaitable, Callable, Dict, FrozenSet, List, Optional

from config import DockingConfig, DockStatus, Settings

logger = logging.getLogger(__name__)

try:
    import serial
    from serial.tools import list_ports
    _HAS_PYSERIAL = True
except ImportError:
    serial = None  # type: ignore[assignment]
    list_ports = None  # type: ignore[assignment]
    _HAS_PYSERIAL = False


# Stable resting states (plus ERROR) that end a DOCK/UNDOCK command.
_TERMINAL_STATUSES = {DockStatus.DOCKED, DockStatus.UNDOCKED, DockStatus.ERROR}

# States the firmware is in while it is actually running a sequence.
_BUSY_STATUSES = {
    DockStatus.UNDOCKING_M1, DockStatus.UNDOCKING_M2,
    DockStatus.DOCKING_M2, DockStatus.DOCKING_M1,
    DockStatus.DOCKING_PROP_OPEN, DockStatus.DOCKING_PROP_CLOSE,
    DockStatus.PROP_CYCLE_OPEN, DockStatus.PROP_CYCLE_CLOSE,
    DockStatus.TESTING,
}

# PROPCYCLE leaves M1/M2 wherever they are, so any resting state ends it.
_PROP_CYCLE_TERMINALS = frozenset(
    {DockStatus.DOCKED, DockStatus.UNDOCKED, DockStatus.UNKNOWN, DockStatus.ERROR}
)

# USB vendor ID of the CP210x bridge on the ESP32 dev board. Used to find the
# board if the configured port path doesn't exist.
_CP210X_VID = 0x10C4

# How long to wait for a reply to STATUS.
_STATUS_REPLY_TIMEOUT_S = 2.5

# While a DOCK/UNDOCK runs, poll STATUS this often as a backstop in case a
# STATE: line is missed. It also lets a command that never started fail fast
# instead of waiting the full command timeout.
_COMMAND_POLL_INTERVAL_S = 5.0

# Auto-reconnect backoff.
_MIN_RECONNECT_DELAY_SECONDS = 2.0
_MAX_RECONNECT_DELAY_SECONDS = 30.0
_CONNECTION_POLL_SECONDS = 5.0

_BOOT_BANNER = "DOCKING SYSTEM STARTING"


class DockingError(Exception):
    """Base class for docking controller errors."""
    pass


class DockingConnectionError(DockingError):
    """Raised when not connected to the dock controller, or connect fails."""
    pass


class DockingTimeoutError(DockingError):
    """Raised when no terminal status (or STATUS reply) arrives in time."""
    pass


class DockingController:
    """
    USB serial client for the ESP32 dock-lock controller.

    Usage:
        async with DockingController(settings.docking_config) as dock:
            status = await dock.undock()
    """

    def __init__(self, config: DockingConfig) -> None:
        if not _HAS_PYSERIAL:
            raise ImportError(
                "pyserial is required for DockingController. Install it with: pip install pyserial"
            )
        self._config = config
        self._serial = None
        self._port_in_use: Optional[str] = None
        self._loop: Optional[asyncio.AbstractEventLoop] = None
        self._reader_thread: Optional[threading.Thread] = None
        self._reader_stop = threading.Event()
        self._write_lock = threading.Lock()

        self._status = DockStatus.UNKNOWN
        self._status_waiters: List[asyncio.Future] = []
        # Limit switch readings from the last STATUS reply, e.g. {"Prop_close": "HIT"}.
        self._last_switches: Dict[str, str] = {}

        # Per-command bookkeeping, only meaningful while _command_lock is held.
        self._command_lock = asyncio.Lock()
        self._command_done = asyncio.Event()
        self._command_name: Optional[str] = None
        self._terminal_status: Optional[DockStatus] = None
        self._command_error: Optional[str] = None
        self._seen_busy = False
        self._command_terminals: FrozenSet[DockStatus] = frozenset(_TERMINAL_STATUSES)
        self._prop_cycle_completed = False

        self._reconnect_task: Optional[asyncio.Task] = None

    @classmethod
    def from_settings(cls, settings: Settings) -> "DockingController":
        return cls(settings.docking_config)

    # ----- Properties -----

    @property
    def is_connected(self) -> bool:
        return (
            self._serial is not None
            and self._serial.is_open
            and self._reader_thread is not None
            and self._reader_thread.is_alive()
        )

    @property
    def is_busy(self) -> bool:
        """Whether a dock/undock command is currently in flight."""
        return self._command_lock.locked()

    @property
    def last_status(self) -> DockStatus:
        return self._status

    @property
    def last_switches(self) -> Dict[str, str]:
        """Limit switch readings from the most recent STATUS reply."""
        return dict(self._last_switches)

    @property
    def port(self) -> str:
        """The serial port in use, or the configured one when not connected."""
        return self._port_in_use or self._config.port

    async def __aenter__(self) -> "DockingController":
        await self.connect()
        return self

    async def __aexit__(self, *exc_info) -> None:
        await self.disconnect()

    # ----- Connection -----

    def _resolve_port(self) -> Optional[str]:
        """Configured port if it exists, else the single CP210x device present."""
        if os.path.exists(self._config.port):
            return self._config.port
        candidates = [p.device for p in list_ports.comports() if p.vid == _CP210X_VID]
        if len(candidates) == 1:
            logger.warning(
                "Configured dock port %s not found — using the only CP210x device, %s",
                self._config.port, candidates[0],
            )
            return candidates[0]
        if len(candidates) > 1:
            logger.error(
                "Configured dock port %s not found and %d CP210x devices are present (%s) — "
                "set DRONE_PAD_DOCK_PORT to pick one",
                self._config.port, len(candidates), ", ".join(candidates),
            )
        return None

    def _open_port(self, port: str):
        ser = serial.Serial()
        ser.port = port
        ser.baudrate = self._config.baudrate
        ser.timeout = 0.5
        ser.write_timeout = 1.0
        # Advisory lock so a second instance of this service can't share the port.
        ser.exclusive = True
        # dtr/rts are left at pyserial's defaults (both asserted) on purpose — see
        # the module docstring. Changing them one at a time would reset the ESP32.
        ser.open()
        ser.reset_input_buffer()
        return ser

    async def connect(self) -> None:
        """Open the serial port and confirm the docking firmware answers STATUS."""
        if self.is_connected:
            return

        self._loop = asyncio.get_running_loop()
        port = self._resolve_port()
        if port is None:
            raise DockingConnectionError(f"Dock controller serial port {self._config.port} not found")

        logger.info("Connecting to dock controller on %s @ %d baud", port, self._config.baudrate)
        try:
            ser = await self._loop.run_in_executor(None, self._open_port, port)
        except Exception as exc:
            raise DockingConnectionError(f"Failed to open {port}: {exc}") from exc

        self._serial = ser
        self._port_in_use = port
        self._reader_stop.clear()
        self._reader_thread = threading.Thread(
            target=self._reader_loop, args=(ser,), name="dock-serial-reader", daemon=True,
        )
        self._reader_thread.start()

        # Confirm it's actually the docking firmware, and seed the cached status.
        # Retried across connect_timeout_seconds: if the board happens to be
        # booting it ignores serial input for ~3 s.
        deadline = self._loop.time() + self._config.connect_timeout_seconds
        last_exc: Optional[Exception] = None
        while self._loop.time() < deadline:
            try:
                self._status = await self.get_status()
                logger.info("Connected to dock controller on %s (status %s)", port, self._status.value)
                return
            except DockingError as exc:
                last_exc = exc
        await self.disconnect()
        raise DockingConnectionError(
            f"No STATUS reply from {port} within {self._config.connect_timeout_seconds:.0f}s — "
            f"is the docking firmware running? ({last_exc})"
        )

    async def disconnect(self) -> None:
        """Stop the reader and close the port, if open."""
        ser, self._serial = self._serial, None
        self._reader_stop.set()
        if ser is not None:
            try:
                await asyncio.get_running_loop().run_in_executor(None, ser.close)
            except Exception:
                pass
            logger.info("Disconnected from dock controller")
        thread, self._reader_thread = self._reader_thread, None
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout=2.0)
        self._fail_pending("disconnected from dock controller")

    async def start(self) -> None:
        """Start the background auto-reconnect loop (call once at app startup)."""
        if self._reconnect_task is not None:
            return
        self._reconnect_task = asyncio.create_task(
            self._connection_loop(), name="dock-serial-reconnect"
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
        """Keep the serial link up for the life of the app, reconnecting on loss."""
        delay = _MIN_RECONNECT_DELAY_SECONDS
        while True:
            if not self.is_connected:
                try:
                    await self.connect()
                    delay = _MIN_RECONNECT_DELAY_SECONDS
                except DockingError as exc:
                    logger.warning("Dock controller connect failed: %s — retrying in %.0fs", exc, delay)
                    await asyncio.sleep(delay)
                    delay = min(delay * 2, _MAX_RECONNECT_DELAY_SECONDS)
                    continue
            await asyncio.sleep(_CONNECTION_POLL_SECONDS)

    # ----- Reader thread -----

    def _reader_loop(self, ser) -> None:
        """Blocking serial reads; every complete line goes to the event loop."""
        while not self._reader_stop.is_set():
            try:
                raw = ser.readline()
            except Exception as exc:  # unplugged, port closed under us, etc.
                if not self._reader_stop.is_set():
                    self._post(self._on_link_lost, str(exc))
                return
            if not raw:
                continue
            text = raw.decode("utf-8", errors="replace").strip()
            if text:
                self._post(self._handle_line, text)

    def _post(self, fn, *args) -> None:
        loop = self._loop
        if loop is None or loop.is_closed():
            return
        try:
            loop.call_soon_threadsafe(fn, *args)
        except RuntimeError:
            pass  # loop shutting down

    def _on_link_lost(self, reason: str) -> None:
        logger.warning("Lost USB serial link to dock controller (%s) — will auto-reconnect", reason)
        ser, self._serial = self._serial, None
        if ser is not None:
            try:
                ser.close()
            except Exception:
                pass
        self._fail_pending(f"USB serial link to dock controller lost: {reason}")

    def _fail_pending(self, reason: str) -> None:
        for fut in self._status_waiters:
            if not fut.done():
                fut.set_exception(DockingConnectionError(reason))
        self._status_waiters.clear()
        if self._command_name is not None and not self._command_done.is_set():
            self._command_error = reason
            self._command_done.set()

    # ----- Line handling (event loop thread) -----

    def _handle_line(self, text: str) -> None:
        if text.startswith("STATE:"):
            self._on_state(self._parse_status(text[len("STATE:"):]), source="change")
            return

        if text.startswith("[STATUS] State="):
            status = self._parse_status(text[len("[STATUS] State="):].split()[0] if
                                        len(text) > len("[STATUS] State=") else "")
            logger.debug("Dock STATUS reply: %s", text)
            self._last_switches = dict(
                tok.split("=", 1) for tok in text.split()[2:] if "=" in tok
            )
            self._on_state(status, source="status")
            waiters, self._status_waiters = self._status_waiters, []
            for fut in waiters:
                if not fut.done():
                    fut.set_result(status)
            return

        in_command = self._command_name is not None and not self._command_done.is_set()

        if _BOOT_BANNER in text:
            logger.warning("Dock controller rebooted")
            if in_command:
                self._command_error = f"dock controller rebooted during {self._command_name}"
                self._command_done.set()
            return

        if text.startswith("PROP_CYCLE_COMPLETE") and in_command and self._command_name == "PROPCYCLE":
            self._prop_cycle_completed = True
            logger.info("Dock firmware: %s", text)
            return

        if in_command and not self._seen_busy and (
            text.startswith("System busy") or text.startswith("Unknown command")
        ):
            self._command_error = f"dock controller rejected {self._command_name}: {text}"
            self._command_done.set()
            logger.warning("%s", self._command_error)
            return

        logger.info("Dock firmware: %s", text)

    def _on_state(self, status: DockStatus, source: str) -> None:
        if status != self._status or source == "change":
            logger.info("Dock status: %s", status.value)
        self._status = status

        if self._command_name is None or self._command_done.is_set():
            return
        if status in _BUSY_STATUSES:
            self._seen_busy = True
        elif status == DockStatus.ERROR:
            self._terminal_status = status
            self._command_done.set()
        elif status in self._command_terminals and (self._seen_busy or self._prop_cycle_completed):
            # Only after the sequence has visibly started — a resting state
            # reported before that is the old position, not the result.
            self._terminal_status = status
            self._command_done.set()

    # ----- Commands -----

    async def dock(self, timeout: Optional[float] = None) -> DockStatus:
        """Send DOCK and wait for DOCKED. Raises DockingError on ERROR or any other outcome."""
        return await self._execute_command("DOCK", DockStatus.DOCKED, timeout=timeout)

    async def undock(self, timeout: Optional[float] = None) -> DockStatus:
        """Send UNDOCK and wait for UNDOCKED. Raises DockingError on ERROR or any other outcome."""
        return await self._execute_command("UNDOCK", DockStatus.UNDOCKED, timeout=timeout)

    async def propeller_cycle(self, timeout: Optional[float] = None) -> DockStatus:
        """
        Send PROPCYCLE: propeller closer open -> close, docking motors untouched.

        Returns the resting state the firmware reports afterwards (DOCKED,
        UNDOCKED or UNKNOWN — M1/M2 weren't moved). Raises DockingError on
        ERROR, or if the propeller closer's close limit isn't confirmed
        triggered by a STATUS read at the end.
        """
        async def verify_closed() -> None:
            await self.get_status()
            if self._last_switches.get("Prop_close") != "HIT":
                raise DockingError(
                    "propeller cycle finished but the propeller closer's close limit "
                    f"is not triggered (switches: {self._last_switches or 'unknown'})"
                )

        return await self._execute_command(
            "PROPCYCLE", None, timeout=timeout,
            terminals=_PROP_CYCLE_TERMINALS, verify=verify_closed,
        )

    async def reset(self) -> None:
        """Send RESET — stops all motors and re-derives DOCKED/UNDOCKED/UNKNOWN from the limit switches."""
        await self._write_command("RESET")

    async def get_status(self) -> DockStatus:
        """Ask the firmware for its current state (STATUS) and return the reply."""
        if not self.is_connected:
            raise DockingConnectionError("Not connected to dock controller")
        loop = asyncio.get_running_loop()
        fut = loop.create_future()
        self._status_waiters.append(fut)
        try:
            await self._write_command("STATUS")
            return await asyncio.wait_for(asyncio.shield(fut), timeout=_STATUS_REPLY_TIMEOUT_S)
        except asyncio.TimeoutError as exc:
            raise DockingTimeoutError("No reply to STATUS from dock controller") from exc
        finally:
            if fut in self._status_waiters:
                self._status_waiters.remove(fut)

    # ----- Internals -----

    def _write_blocking(self, payload: bytes) -> None:
        ser = self._serial
        if ser is None:
            raise DockingConnectionError("Not connected to dock controller")
        with self._write_lock:
            ser.write(payload)
            ser.flush()

    async def _write_command(self, command: str) -> None:
        if not self.is_connected:
            raise DockingConnectionError("Not connected to dock controller")
        try:
            await asyncio.get_running_loop().run_in_executor(
                None, self._write_blocking, (command + "\n").encode("ascii")
            )
        except DockingError:
            raise
        except Exception as exc:
            raise DockingConnectionError(f"Failed to send {command}: {exc}") from exc
        if command != "STATUS":
            logger.info("Sent %s command", command)

    async def _execute_command(
        self,
        command: str,
        expected: Optional[DockStatus],
        timeout: Optional[float],
        terminals: FrozenSet[DockStatus] = frozenset(_TERMINAL_STATUSES),
        verify: Optional[Callable[[], Awaitable[None]]] = None,
    ) -> DockStatus:
        """
        Send `command` and wait for it to finish.

        `expected` is the resting state that counts as success (DOCK/UNDOCK),
        or None when any state in `terminals` other than ERROR is acceptable
        (PROPCYCLE), in which case `verify` decides success.
        """
        timeout = timeout if timeout is not None else self._config.command_timeout_seconds
        loop = asyncio.get_running_loop()

        async with self._command_lock:
            self._command_done.clear()
            self._terminal_status = None
            self._command_error = None
            self._seen_busy = False
            self._prop_cycle_completed = False
            self._command_terminals = frozenset(terminals)
            self._command_name = command
            try:
                await self._write_command(command)
                deadline = loop.time() + timeout

                while not self._command_done.is_set():
                    remaining = deadline - loop.time()
                    if remaining <= 0:
                        raise DockingTimeoutError(
                            f"No terminal status received within {timeout:.1f}s after {command}"
                        )
                    try:
                        await asyncio.wait_for(
                            self._command_done.wait(), timeout=min(remaining, _COMMAND_POLL_INTERVAL_S)
                        )
                        break
                    except asyncio.TimeoutError:
                        pass

                    # Backstop poll.
                    try:
                        polled = await self.get_status()
                    except DockingTimeoutError:
                        continue
                    except DockingConnectionError:
                        continue  # _fail_pending() will have completed the command
                    if self._command_done.is_set():
                        break
                    started = self._seen_busy or self._prop_cycle_completed
                    if polled in _BUSY_STATUSES:
                        self._seen_busy = True
                    elif polled == DockStatus.ERROR or (started and polled in terminals):
                        self._terminal_status = polled
                        break
                    elif expected is not None and polled == expected:
                        # Already finished, and its STATE: lines were missed.
                        self._terminal_status = polled
                        break
                    elif not started:
                        raise DockingError(
                            f"{command} was not started by the dock controller "
                            f"(still {polled.value} after {_COMMAND_POLL_INTERVAL_S:.0f}s)"
                        )

                if self._command_error:
                    raise DockingError(self._command_error)

                result = self._terminal_status
                if result == DockStatus.ERROR:
                    raise DockingError(f"{command} failed — dock controller reported ERROR")
                if result is None:
                    raise DockingError(f"{command} ended without a final state")
                if expected is not None and result != expected:
                    raise DockingError(
                        f"{command} ended in {result.value}, expected {expected.value}"
                    )
                if verify is not None:
                    await verify()
                return result
            finally:
                self._command_name = None

    @staticmethod
    def _parse_status(raw: str) -> DockStatus:
        raw = raw.strip()
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
    if args.port:
        docking_config = replace(docking_config, port=args.port)

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
        elif args.command == "propcycle":
            status = await controller.propeller_cycle()
            print(f"Propeller cycle result: {status.value} (switches: {controller.last_switches})")
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

    parser = argparse.ArgumentParser(description="Manually test the USB serial dock-lock controller")
    parser.add_argument("command", choices=["dock", "undock", "propcycle", "status", "reset"])
    parser.add_argument("--port", default=None, help="Override the dock controller's serial port")
    parser.add_argument("-v", "--verbose", action="store_true", help="Enable debug logging")
    cli_args = parser.parse_args()

    logging.basicConfig(
        level=logging.DEBUG if cli_args.verbose else logging.INFO,
        format="%(asctime)s | %(levelname)-8s | %(name)-20s | %(message)s",
        datefmt="%Y-%m-%d %H:%M:%S",
    )

    sys.exit(asyncio.run(_cli_main(cli_args)))
