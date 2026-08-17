"""
Drone Pad API Service — Debug/Test Controller

Isolated single-axis motor testing, independent of PadStateMachine's
CLOSED/OPEN sequencing. Jogs one motor toward one limit switch — e.g.
"lift up" or "slide close" — so each axis can be bench-tested on its
own before relying on the full open/close sequence.
"""

from __future__ import annotations

import logging
import threading
import time
from dataclasses import dataclass
from datetime import datetime, timezone
from typing import Callable, Dict, Optional, Tuple

from config import Direction, MotorId, Settings
from gpio_manager import GPIOManager
from motor_controller import MotorCancelledError, MotorController, MotorTimeoutError

logger = logging.getLogger(__name__)


# (motor_id, direction) -> GPIOManager method name for the limit switch
# that should stop this jog. Mirrors the per-stage checks in PadStateMachine.
_LIMIT_CHECK_METHOD: Dict[Tuple[MotorId, Direction], str] = {
    (MotorId.OPENING, Direction.FORWARD): "is_open_limit_triggered",   # slide open
    (MotorId.OPENING, Direction.REVERSE): "is_close_limit_triggered",  # slide close
    (MotorId.LIFT, Direction.FORWARD): "is_lift_upper_triggered",      # lift up
    (MotorId.LIFT, Direction.REVERSE): "is_lift_lower_triggered",      # lift down
}


@dataclass
class DebugJogStatus:
    """Snapshot of the current/last single-axis jog."""
    running: bool
    motor_id: Optional[str]
    direction: Optional[str]
    started_at: Optional[str]
    elapsed_seconds: Optional[float]
    last_result: Optional[str]   # "success" | "timeout" | "cancelled" | "error" | None
    last_message: Optional[str]


class DebugMotorController:
    """
    Runs a single motor toward a single limit switch, respecting the
    same timeout/cancel safety mechanics as MotorController.run_until_limit(),
    but without any of PadStateMachine's CLOSED/OPEN/OPENING_* sequencing.
    """

    def __init__(self, gpio: GPIOManager, motor: MotorController, settings: Settings) -> None:
        self._gpio = gpio
        self._motor = motor
        self._settings = settings

        self._lock = threading.Lock()
        self._thread: Optional[threading.Thread] = None
        self._cancel_event = threading.Event()
        self._pause_event = threading.Event()
        self._pause_event.set()  # never paused for debug jogs — just running or not

        self._running = False
        self._motor_id: Optional[MotorId] = None
        self._direction: Optional[Direction] = None
        self._started_at_monotonic: Optional[float] = None
        self._started_at_iso: Optional[str] = None
        self._last_result: Optional[str] = None
        self._last_message: Optional[str] = None

    def get_status(self) -> DebugJogStatus:
        with self._lock:
            elapsed = (
                round(time.monotonic() - self._started_at_monotonic, 1)
                if (self._running and self._started_at_monotonic is not None)
                else None
            )
            return DebugJogStatus(
                running=self._running,
                motor_id=self._motor_id.value if self._motor_id else None,
                direction=self._direction.name if self._direction else None,
                started_at=self._started_at_iso,
                elapsed_seconds=elapsed,
                last_result=self._last_result,
                last_message=self._last_message,
            )

    def jog(self, motor_id: MotorId, direction: Direction) -> dict:
        """Start jogging `motor_id` toward `direction`'s limit switch."""
        limit_method_name = _LIMIT_CHECK_METHOD.get((motor_id, direction))
        if limit_method_name is None:
            return {
                "success": False,
                "message": f"No limit switch mapping for {motor_id.value}/{direction.name}",
            }

        with self._lock:
            if self._running:
                return {
                    "success": False,
                    "message": (
                        f"A jog is already running "
                        f"({self._motor_id.value}/{self._direction.name})"
                    ),
                }
            self._running = True
            self._motor_id = motor_id
            self._direction = direction
            self._started_at_monotonic = time.monotonic()
            self._started_at_iso = datetime.now(timezone.utc).isoformat()
            self._last_result = None
            self._last_message = None
            self._cancel_event.clear()
            self._pause_event.set()

        limit_check = getattr(self._gpio, limit_method_name)

        self._thread = threading.Thread(
            target=self._run,
            args=(motor_id, direction, limit_check),
            name=f"debug-jog-{motor_id.value}",
            daemon=True,
        )
        self._thread.start()
        logger.info("Debug jog started: %s %s", motor_id.value, direction.name)
        return {"success": True, "message": f"Jogging {motor_id.value} {direction.name}"}

    def stop(self) -> dict:
        """Cancel the current jog, if any, and wait for it to unwind."""
        with self._lock:
            if not self._running:
                return {"success": True, "message": "No jog in progress"}
            self._cancel_event.set()
            self._pause_event.set()

        thread = self._thread
        if thread is not None and thread.is_alive():
            thread.join(timeout=5.0)

        return {"success": True, "message": "Jog stopped"}

    def _run(
        self,
        motor_id: MotorId,
        direction: Direction,
        limit_check: Callable[[], bool],
    ) -> None:
        try:
            self._motor.run_until_limit(
                motor_id=motor_id,
                direction=direction,
                limit_check=limit_check,
                cancel_event=self._cancel_event,
                pause_event=self._pause_event,
            )
            self._finish("success", f"{motor_id.value} reached its {direction.name} limit switch")
        except MotorCancelledError:
            self._finish("cancelled", "Jog stopped by request")
        except MotorTimeoutError as exc:
            self._finish("timeout", str(exc))
        except Exception as exc:
            logger.exception("Unexpected error during debug jog")
            self._finish("error", str(exc))

    def _finish(self, result: str, message: str) -> None:
        with self._lock:
            self._running = False
            self._last_result = result
            self._last_message = message
        logger.info("Debug jog finished: %s — %s", result, message)
