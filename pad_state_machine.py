"""
Drone Pad API Service — Pad State Machine

Thread-safe state machine that orchestrates the open/close sequences,
handles interrupts (close during open), and supports pause/resume.
"""

from __future__ import annotations

import logging
import threading
import time
from dataclasses import dataclass, field
from datetime import datetime, timezone
from typing import Optional, List

from config import PadState, MotorId, Direction, Settings
from gpio_manager import GPIOManager
from motor_controller import (
    MotorController,
    MotorTimeoutError,
    MotorCancelledError,
)

logger = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# Status snapshot
# ---------------------------------------------------------------------------

@dataclass
class PadStatus:
    """Immutable snapshot of the pad's current status."""
    state: PadState
    is_paused: bool
    active_motor: Optional[str]
    error_message: Optional[str]
    last_command: Optional[str]
    last_command_time: Optional[str]
    uptime_seconds: float


# ---------------------------------------------------------------------------
# Event log entry
# ---------------------------------------------------------------------------

@dataclass
class EventEntry:
    """Record of a state machine event."""
    timestamp: str
    event: str
    from_state: str
    to_state: str


# ---------------------------------------------------------------------------
# Pad State Machine
# ---------------------------------------------------------------------------

class PadStateMachine:
    """
    Orchestrates the drone pad's open/close lifecycle.

    Thread Safety:
      - All state transitions are protected by a reentrant lock.
      - Motor sequences run in a dedicated background thread.
      - Only one sequence thread can be active at a time.

    Interrupt Handling:
      - A close command during an open sequence sets the cancel_event,
        which causes the active motor loop to abort, then the close
        sequence starts from the current stage.

    Pause/Resume:
      - The toggle command clears/sets the pause_event, which causes
        the active motor loop to wait until resumed.
    """

    # Maximum event log entries to retain
    _MAX_EVENT_LOG = 100

    def __init__(
        self,
        gpio: GPIOManager,
        motor: MotorController,
        settings: Settings,
    ) -> None:
        self._gpio = gpio
        self._motor = motor
        self._settings = settings

        # State
        self._state = PadState.CLOSED
        self._active_motor: Optional[MotorId] = None
        self._error_message: Optional[str] = None
        self._last_command: Optional[str] = None
        self._last_command_time: Optional[datetime] = None
        self._start_time = time.monotonic()

        # Threading
        self._lock = threading.RLock()
        self._sequence_thread: Optional[threading.Thread] = None
        self._cancel_event = threading.Event()
        self._pause_event = threading.Event()
        self._pause_event.set()  # Start in "running" (not paused) state
        self._is_paused = False

        # Event log
        self._event_log: List[EventEntry] = []

    # ----- Public API -----

    def get_status(self) -> PadStatus:
        """Return a snapshot of the current pad status."""
        with self._lock:
            return PadStatus(
                state=self._state,
                is_paused=self._is_paused,
                active_motor=self._active_motor.value if self._active_motor else None,
                error_message=self._error_message,
                last_command=self._last_command,
                last_command_time=(
                    self._last_command_time.isoformat()
                    if self._last_command_time else None
                ),
                uptime_seconds=round(time.monotonic() - self._start_time, 1),
            )

    def get_event_log(self) -> List[EventEntry]:
        """Return a copy of the recent event log."""
        with self._lock:
            return list(self._event_log)

    def open(self) -> dict:
        """
        Start the open sequence.

        Returns a dict with 'success' and 'message' keys.
        """
        with self._lock:
            self._last_command = "open"
            self._last_command_time = datetime.now(timezone.utc)

            if self._state == PadState.OPEN:
                return {"success": True, "message": "Pad is already open"}

            if self._state == PadState.OPENING_SLIDE or self._state == PadState.OPENING_LIFT:
                return {"success": True, "message": "Open sequence already in progress"}

            if self._state in (PadState.CLOSING_LIFT, PadState.CLOSING_SLIDE):
                return {
                    "success": False,
                    "message": (
                        "Cannot open while closing is in progress. "
                        "Wait for close to complete or issue emergency-stop first."
                    ),
                }

            if self._state == PadState.ERROR:
                return {
                    "success": False,
                    "message": f"Pad is in error state: {self._error_message}. "
                               "Issue emergency-stop to reset.",
                }

            # State is CLOSED — start open sequence
            self._error_message = None
            self._cancel_event.clear()
            self._pause_event.set()
            self._is_paused = False

            self._start_sequence(self._open_sequence, "open")
            return {"success": True, "message": "Open sequence started"}

    def close(self) -> dict:
        """
        Start the close sequence, or interrupt an in-progress open.

        Returns a dict with 'success' and 'message' keys.
        """
        with self._lock:
            self._last_command = "close"
            self._last_command_time = datetime.now(timezone.utc)

            if self._state == PadState.CLOSED:
                return {"success": True, "message": "Pad is already closed"}

            if self._state in (PadState.CLOSING_LIFT, PadState.CLOSING_SLIDE):
                return {"success": True, "message": "Close sequence already in progress"}

            if self._state == PadState.ERROR:
                return {
                    "success": False,
                    "message": f"Pad is in error state: {self._error_message}. "
                               "Issue emergency-stop to reset.",
                }

            # If currently opening — interrupt it
            if self._state in (PadState.OPENING_SLIDE, PadState.OPENING_LIFT):
                logger.info(
                    "Close command received during %s — interrupting",
                    self._state.value,
                )
                # Resume if paused so the cancel can propagate
                self._pause_event.set()
                self._is_paused = False
                self._cancel_event.set()

                # Wait for the open sequence thread to finish
                thread = self._sequence_thread
                if thread and thread.is_alive():
                    self._lock.release()
                    try:
                        thread.join(timeout=5.0)
                    finally:
                        self._lock.acquire()

                # Now start close sequence from wherever we are
                self._cancel_event.clear()
                self._pause_event.set()
                self._is_paused = False

                self._start_sequence(self._close_from_current, "close-interrupt")
                return {
                    "success": True,
                    "message": "Open interrupted — close sequence started",
                }

            # State is OPEN — start normal close sequence
            self._error_message = None
            self._cancel_event.clear()
            self._pause_event.set()
            self._is_paused = False

            self._start_sequence(self._close_sequence, "close")
            return {"success": True, "message": "Close sequence started"}

    def toggle(self) -> dict:
        """
        Toggle pause/resume on the currently running operation.

        Returns a dict with 'success' and 'message' keys.
        """
        with self._lock:
            self._last_command = "toggle"
            self._last_command_time = datetime.now(timezone.utc)

            # Only meaningful if a sequence is running
            if self._state in (PadState.CLOSED, PadState.OPEN):
                return {
                    "success": False,
                    "message": f"No operation in progress (state={self._state.value})",
                }

            if self._state == PadState.ERROR:
                return {
                    "success": False,
                    "message": "Pad is in error state",
                }

            if self._is_paused:
                # Resume
                self._is_paused = False
                self._pause_event.set()
                self._log_event("RESUMED", self._state.value, self._state.value)
                logger.info("Operation RESUMED in state %s", self._state.value)
                return {"success": True, "message": "Operation resumed"}
            else:
                # Pause
                self._is_paused = True
                self._pause_event.clear()
                self._log_event("PAUSED", self._state.value, self._state.value)
                logger.info("Operation PAUSED in state %s", self._state.value)
                return {"success": True, "message": "Operation paused"}

    def emergency_stop(self) -> dict:
        """
        Immediately stop all motors and reset to a safe state.
        """
        with self._lock:
            self._last_command = "emergency_stop"
            self._last_command_time = datetime.now(timezone.utc)

            prev_state = self._state

            # Signal cancellation and resume (so thread can exit)
            self._cancel_event.set()
            self._pause_event.set()
            self._is_paused = False

            # Immediately kill motors at hardware level
            self._motor.emergency_stop()

            # Wait for sequence thread
            thread = self._sequence_thread
            if thread and thread.is_alive():
                self._lock.release()
                try:
                    thread.join(timeout=5.0)
                finally:
                    self._lock.acquire()

            self._state = PadState.ERROR
            self._active_motor = None
            self._error_message = "Emergency stop activated"

            self._log_event("EMERGENCY_STOP", prev_state.value, PadState.ERROR.value)
            logger.warning("EMERGENCY STOP executed from state %s", prev_state.value)

            return {"success": True, "message": "Emergency stop executed — all motors disabled"}

    def reset_from_error(self) -> dict:
        """
        Reset the pad from ERROR state to CLOSED.
        Only allowed when no sequence is running.
        """
        with self._lock:
            if self._state != PadState.ERROR:
                return {
                    "success": False,
                    "message": f"Not in error state (current: {self._state.value})",
                }

            thread = self._sequence_thread
            if thread and thread.is_alive():
                return {
                    "success": False,
                    "message": "A sequence thread is still running",
                }

            self._state = PadState.CLOSED
            self._error_message = None
            self._active_motor = None
            self._cancel_event.clear()
            self._pause_event.set()
            self._is_paused = False

            self._log_event("RESET", PadState.ERROR.value, PadState.CLOSED.value)
            logger.info("Pad reset from ERROR to CLOSED")
            return {"success": True, "message": "Pad reset to CLOSED state"}

    # ----- Private: Sequence orchestration -----

    def _start_sequence(self, target: callable, name: str) -> None:
        """Launch a sequence function in a background thread."""
        self._sequence_thread = threading.Thread(
            target=target,
            name=f"pad-{name}",
            daemon=True,
        )
        self._sequence_thread.start()
        logger.info("Sequence thread '%s' started", name)

    # ----- Private: Open Sequence -----

    def _open_sequence(self) -> None:
        """Full open: slide open → lift up."""
        try:
            # Stage 1: Slide open (NEMA 23 forward)
            self._transition(PadState.OPENING_SLIDE, MotorId.OPENING)
            self._motor.run_until_limit(
                motor_id=MotorId.OPENING,
                direction=Direction.FORWARD,
                limit_check=self._gpio.is_open_limit_triggered,
                cancel_event=self._cancel_event,
                pause_event=self._pause_event,
            )

            # Stage 2: Lift up (NEMA 17 forward)
            self._transition(PadState.OPENING_LIFT, MotorId.LIFT)
            self._motor.run_until_limit(
                motor_id=MotorId.LIFT,
                direction=Direction.FORWARD,
                limit_check=self._gpio.is_lift_upper_triggered,
                cancel_event=self._cancel_event,
                pause_event=self._pause_event,
            )

            # Done
            self._transition(PadState.OPEN, None)
            logger.info("Open sequence completed successfully")

        except MotorCancelledError:
            logger.info("Open sequence cancelled (interrupt)")
            # State will be set by the close sequence that follows
        except MotorTimeoutError as exc:
            self._handle_error(str(exc))
        except Exception as exc:
            self._handle_error(f"Unexpected error during open: {exc}")
            logger.exception("Unexpected error in open sequence")

    # ----- Private: Close Sequence -----

    def _close_sequence(self) -> None:
        """Full close: lift down → slide close."""
        try:
            # Stage 1: Lift down (NEMA 17 reverse)
            self._transition(PadState.CLOSING_LIFT, MotorId.LIFT)
            self._motor.run_until_limit(
                motor_id=MotorId.LIFT,
                direction=Direction.REVERSE,
                limit_check=self._gpio.is_lift_lower_triggered,
                cancel_event=self._cancel_event,
                pause_event=self._pause_event,
            )

            # Stage 2: Slide close (NEMA 23 reverse)
            self._transition(PadState.CLOSING_SLIDE, MotorId.OPENING)
            self._motor.run_until_limit(
                motor_id=MotorId.OPENING,
                direction=Direction.REVERSE,
                limit_check=self._gpio.is_close_limit_triggered,
                cancel_event=self._cancel_event,
                pause_event=self._pause_event,
            )

            # Done
            self._transition(PadState.CLOSED, None)
            logger.info("Close sequence completed successfully")

        except MotorCancelledError:
            logger.warning("Close sequence cancelled unexpectedly")
            self._handle_error("Close sequence was cancelled")
        except MotorTimeoutError as exc:
            self._handle_error(str(exc))
        except Exception as exc:
            self._handle_error(f"Unexpected error during close: {exc}")
            logger.exception("Unexpected error in close sequence")

    # ----- Private: Close from current position (interrupt) -----

    def _close_from_current(self) -> None:
        """
        Close sequence starting from the current stage.

        Called when a close command interrupts an open sequence:
          - If we were in OPENING_SLIDE → go directly to CLOSING_SLIDE
          - If we were in OPENING_LIFT → go to CLOSING_LIFT first, then CLOSING_SLIDE
        """
        try:
            current = self._state

            # Determine starting point
            if current in (PadState.OPENING_LIFT, PadState.OPEN, PadState.CLOSING_LIFT):
                # Need to lower the lift first
                self._transition(PadState.CLOSING_LIFT, MotorId.LIFT)
                self._motor.run_until_limit(
                    motor_id=MotorId.LIFT,
                    direction=Direction.REVERSE,
                    limit_check=self._gpio.is_lift_lower_triggered,
                    cancel_event=self._cancel_event,
                    pause_event=self._pause_event,
                )

            # Then close the slide
            self._transition(PadState.CLOSING_SLIDE, MotorId.OPENING)
            self._motor.run_until_limit(
                motor_id=MotorId.OPENING,
                direction=Direction.REVERSE,
                limit_check=self._gpio.is_close_limit_triggered,
                cancel_event=self._cancel_event,
                pause_event=self._pause_event,
            )

            self._transition(PadState.CLOSED, None)
            logger.info("Close-from-current sequence completed successfully")

        except MotorCancelledError:
            logger.warning("Close-from-current cancelled")
            self._handle_error("Close interrupted")
        except MotorTimeoutError as exc:
            self._handle_error(str(exc))
        except Exception as exc:
            self._handle_error(f"Unexpected error during close-from-current: {exc}")
            logger.exception("Unexpected error in close-from-current")

    # ----- Private: Helpers -----

    def _transition(self, new_state: PadState, motor: Optional[MotorId]) -> None:
        """Thread-safe state transition with logging."""
        with self._lock:
            old_state = self._state
            self._state = new_state
            self._active_motor = motor
            self._log_event("TRANSITION", old_state.value, new_state.value)
        logger.info("State: %s → %s (motor=%s)", old_state.value, new_state.value,
                     motor.value if motor else "none")

    def _handle_error(self, message: str) -> None:
        """Transition to ERROR state."""
        with self._lock:
            old_state = self._state
            self._state = PadState.ERROR
            self._active_motor = None
            self._error_message = message
            self._log_event("ERROR", old_state.value, PadState.ERROR.value)
        # Ensure all motors are disabled
        self._motor.emergency_stop()
        logger.error("Pad entered ERROR state: %s", message)

    def _log_event(self, event: str, from_state: str, to_state: str) -> None:
        """Append to the internal event log (must be called under lock)."""
        entry = EventEntry(
            timestamp=datetime.now(timezone.utc).isoformat(),
            event=event,
            from_state=from_state,
            to_state=to_state,
        )
        self._event_log.append(entry)
        if len(self._event_log) > self._MAX_EVENT_LOG:
            self._event_log = self._event_log[-self._MAX_EVENT_LOG:]
