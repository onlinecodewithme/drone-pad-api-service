"""
Drone Pad API Service — Pad State Machine

Thread-safe state machine that orchestrates the open/close sequences,
handles interrupts (close during open), and supports pause/resume.
"""

from __future__ import annotations

import asyncio
import concurrent.futures
import logging
import threading
import time
from dataclasses import dataclass, field
from datetime import datetime, timezone
from typing import Optional, List, Tuple

from config import PadState, MotorId, Direction, Settings
from gpio_manager import GPIOManager
from motor_controller import (
    MotorController,
    MotorTimeoutError,
    MotorCancelledError,
)
from docking_controller import DockingController, DockingError

logger = logging.getLogger(__name__)

# States where the pad is communicating with the docking system over BLE.
# Not interruptible (open/close/toggle reject) and not pausable — aborting
# mid-flight while the dock-lock mechanism is moving is unsafe.
_DOCKING_BUSY_STATES = (PadState.UNDOCKING, PadState.DOCKING)

# Extra time allowed on top of the docking system's own command_timeout_seconds
# before giving up on the cross-thread bridge call itself.
_DOCKING_BRIDGE_TIMEOUT_BUFFER_S = 15.0

# How long to wait for the docking BLE link to (re)connect on its own before
# giving up on a DOCK/UNDOCK. DockingController runs an independent
# reconnect loop with its own backoff (2-30s) in the background — a command
# issued the instant it's needed can easily land mid-backoff, seconds away
# from a retry that would have succeeded. Failing instantly there wastes the
# ~seconds-to-tens-of-seconds of real motor travel that already happened to
# get here. Bounded so a genuinely dead link still fails in reasonable time.
_DOCKING_CONNECT_WAIT_S = 20.0


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
        docking: Optional[DockingController] = None,
        loop: Optional[asyncio.AbstractEventLoop] = None,
    ) -> None:
        self._gpio = gpio
        self._motor = motor
        self._settings = settings
        # BLE docking integration. `docking`'s coroutines run on `loop` (the
        # FastAPI event loop); this class runs sequences in a plain
        # threading.Thread, so calls into docking are bridged with
        # asyncio.run_coroutine_threadsafe() — see _run_docking_command().
        self._docking = docking
        self._loop = loop
        # The in-flight docking bridge future, if any — see _run_docking_command()
        # and emergency_stop().
        self._docking_future: Optional[concurrent.futures.Future] = None

        # State — determined from the limit switches, never assumed. A
        # freshly-restarted process has no memory of what happened before
        # it started; hardcoding CLOSED here would repeat exactly the bug
        # this class's reset_from_error() had to be fixed for (trusting
        # remembered state over physical reality). Retried patiently for a
        # few seconds since a fresh power-on may need a moment to settle —
        # if the switches still don't clearly agree on OPEN or CLOSED after
        # that, start in ERROR. This is only cosmetic for GET /api/status
        # while idle, though: open()/close() no longer require a resolved
        # state to work — they self-heal from ERROR the moment they're
        # called (see open()/close()/_close_sequence()).
        inferred = self._infer_physical_state_patient()
        if inferred is not None:
            self._state = inferred
            self._error_message = None
        else:
            self._state = PadState.ERROR
            self._error_message = (
                "Startup: could not determine pad position from limit switches. "
                "This will resolve automatically the next time open() or close() "
                "is called — or check POST /api/debug/limits / jog to a known "
                "limit and POST /api/reset to resolve it without moving anything."
            )
        self._active_motor: Optional[MotorId] = None
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

        Never trusts remembered state to answer "are we already open" or
        to refuse from ERROR — CLOSED, OPEN, and ERROR all (re)launch
        _open_sequence(), which live-verifies the pad's actual position at
        each stage via its limit switches and safely no-ops any stage
        that's already done (e.g. already-open skips the slide motor
        entirely and resolves in under a second). This is what lets the
        pad recover from a stale/incorrect ERROR — including one left by a
        power cycle whose switches hadn't settled yet — with nothing more
        than the caller issuing the command they actually wanted.

        Returns a dict with 'success' and 'message' keys.
        """
        with self._lock:
            self._last_command = "open"
            self._last_command_time = datetime.now(timezone.utc)

            if self._state in (PadState.OPENING_SLIDE, PadState.OPENING_LIFT, PadState.UNDOCKING):
                return {"success": True, "message": "Open sequence already in progress"}

            if self._state in (PadState.DOCKING, PadState.CLOSING_LIFT, PadState.CLOSING_SLIDE):
                return {
                    "success": False,
                    "message": (
                        "Cannot open while closing is in progress. "
                        "Wait for close to complete or issue emergency-stop first."
                    ),
                }

            # CLOSED, OPEN, or ERROR — (re)launch. Docking connectivity is
            # checked lazily, only if UNDOCK actually needs to be sent
            # (inside _run_docking_command), not up front — an already-open
            # pad shouldn't be blocked by an unrelated BLE hiccup.
            self._error_message = None
            self._cancel_event.clear()
            self._pause_event.set()
            self._is_paused = False

            if not self._start_sequence(self._open_sequence, "open"):
                return {
                    "success": False,
                    "message": "A previous sequence hasn't fully stopped yet — try again in a moment",
                }
            return {"success": True, "message": "Open sequence started"}

    def close(self) -> dict:
        """
        Start the close sequence, or interrupt an in-progress open.

        Never trusts remembered state to answer "are we already closed" or
        to refuse from ERROR — CLOSED, OPEN, and ERROR all (re)launch
        _close_sequence(), which live-checks lift_upper to decide whether a
        DOCK is even needed (skipping it if the pad was never lifted, e.g.
        already closed or an open interrupted before the lift finished),
        then live-verifies each remaining stage via its own limit switch,
        safely no-opping anything already done. Same recovery story as
        open() — no manual reset required, just call the command you want.

        Returns a dict with 'success' and 'message' keys.
        """
        with self._lock:
            self._last_command = "close"
            self._last_command_time = datetime.now(timezone.utc)

            if self._state in (PadState.DOCKING, PadState.CLOSING_LIFT, PadState.CLOSING_SLIDE):
                return {"success": True, "message": "Close sequence already in progress"}

            if self._state == PadState.UNDOCKING:
                return {
                    "success": False,
                    "message": (
                        "Cannot interrupt — waiting on the docking system's UNDOCK "
                        "to complete. Wait for it to finish or issue emergency-stop."
                    ),
                }

            # If currently opening — interrupt it. UNDOCKING is excluded above.
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

                self._cancel_event.clear()
                self._pause_event.set()
                self._is_paused = False
                self._error_message = None

                if not self._start_sequence(self._close_sequence, "close-interrupt"):
                    return {
                        "success": False,
                        "message": "A previous sequence hasn't fully stopped yet — try again in a moment",
                    }
                return {
                    "success": True,
                    "message": "Open interrupted — close sequence started",
                }

            # CLOSED, OPEN, or ERROR — (re)launch. Docking connectivity is
            # checked lazily inside _close_sequence, only if it determines
            # a DOCK is actually needed.
            self._error_message = None
            self._cancel_event.clear()
            self._pause_event.set()
            self._is_paused = False

            if not self._start_sequence(self._close_sequence, "close"):
                return {
                    "success": False,
                    "message": "A previous sequence hasn't fully stopped yet — try again in a moment",
                }
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

            if self._state in _DOCKING_BUSY_STATES:
                return {
                    "success": False,
                    "message": (
                        f"Cannot pause — waiting on the docking system "
                        f"({self._state.value}), not a motor move"
                    ),
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

            # cancel_event only unblocks a motor stepping loop — a sequence
            # thread currently blocked in _run_docking_command() waiting on
            # a DOCK/UNDOCK response never checks it at all, and would
            # otherwise keep running silently for its own full ~60s timeout
            # while self._sequence_thread gets overwritten by whatever the
            # caller starts next. Cancel that future directly so the thread
            # actually exits before the join below.
            docking_future = self._docking_future
            if docking_future is not None:
                docking_future.cancel()

            # Wait for sequence thread
            thread = self._sequence_thread
            if thread and thread.is_alive():
                self._lock.release()
                try:
                    thread.join(timeout=5.0)
                finally:
                    self._lock.acquire()

            # Best-effort: also stop the docking system's motors. RESET is a
            # single BLE write (no wait for a terminal status), so this is
            # fast — but never let a BLE hiccup block the pad's own stop.
            self._lock.release()
            try:
                self._reset_docking_best_effort()
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
        Reset the pad from ERROR back to a safe idle state.

        The target state is decided by reading all four limit switches
        directly, never by remembering what state we were in before the
        error — that's exactly the kind of stale bookkeeping an abort/error
        can leave behind, and trusting it can desync software state from
        physical reality. If the switches don't clearly agree on OPEN or
        CLOSED, this refuses to guess — the pad's real position must be
        physically verified and, if needed, jogged to a known limit via
        the debug motor endpoints before it can be reset.

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

            target_state = self._infer_physical_state()
            if target_state is None:
                readings = self._describe_limit_switches()
                return {
                    "success": False,
                    "message": (
                        f"Cannot safely determine pad position from limit switches ({readings}). "
                        "Verify the pad's physical position, then use the debug motor "
                        "endpoints (POST /api/debug/motor/{motor_id}/jog) to jog it to "
                        "a known limit before resetting."
                    ),
                }

            self._state = target_state
            self._error_message = None
            self._active_motor = None
            self._cancel_event.clear()
            self._pause_event.set()
            self._is_paused = False

            self._log_event("RESET", PadState.ERROR.value, target_state.value)
            logger.info("Pad reset from ERROR to %s (verified via limit switches)", target_state.value)
            return {"success": True, "message": f"Pad reset to {target_state.value} state"}

    # ----- Private: Sequence orchestration -----

    def _start_sequence(self, target: callable, name: str) -> bool:
        """
        Launch a sequence function in a background thread.

        Refuses (returns False) if a previous sequence thread is somehow
        still alive — this should be structurally impossible given the
        state guards in open()/close(), but emergency_stop() cancelling a
        stuck docking wait is what actually makes that guarantee hold in
        practice (see emergency_stop()'s docking_future.cancel()); this is
        the last line of defense against ever running two sequences
        concurrently against the same motors/docking link.
        """
        old = self._sequence_thread
        if old is not None and old.is_alive():
            logger.error(
                "Refusing to start '%s' — previous sequence thread '%s' is still running",
                name, old.name,
            )
            return False

        self._sequence_thread = threading.Thread(
            target=target,
            name=f"pad-{name}",
            daemon=True,
        )
        self._sequence_thread.start()
        logger.info("Sequence thread '%s' started", name)
        return True

    # ----- Private: Open Sequence -----

    def _open_sequence(self) -> None:
        """Full open: slide open → lift up → UNDOCK over BLE."""
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

            # Hard safety gate — never lift unless the slide-open is
            # independently confirmed via its own limit switch across
            # multiple samples, not just because run_until_limit() above
            # returned without raising (that alone can be a false-positive
            # "already there" skip if the switch misreports).
            if not self._gpio.confirmed(self._gpio.is_open_limit_triggered):
                self._handle_error(
                    "Refusing to lift — open_limit switch is not confirmed "
                    "triggered (consistently, across multiple reads) after "
                    "the slide-open stage"
                )
                return

            # Stage 2: Lift up (NEMA 17 forward)
            self._transition(PadState.OPENING_LIFT, MotorId.LIFT)
            self._motor.run_until_limit(
                motor_id=MotorId.LIFT,
                direction=Direction.FORWARD,
                limit_check=self._gpio.is_lift_upper_triggered,
                cancel_event=self._cancel_event,
                pause_event=self._pause_event,
            )

            # Stage 3: UNDOCK over BLE. Hard safety gate — re-check the limit
            # switch across multiple samples rather than trusting a single
            # read or that stage 2 returning implies we're really lifted;
            # never send UNDOCK otherwise.
            if not self._gpio.confirmed(self._gpio.is_lift_upper_triggered):
                self._handle_error(
                    "Refusing to UNDOCK — lift_upper limit switch is not confirmed "
                    "triggered (consistently, across multiple reads) after the lift stage"
                )
                return

            self._transition(PadState.UNDOCKING, None)
            ok, message = self._run_docking_command(
                lambda: self._docking.undock(), "UNDOCK"
            )
            if not ok:
                self._handle_error(f"UNDOCK failed — pad is open but drone was not released: {message}")
                return

            # Done
            self._transition(PadState.OPEN, None)
            logger.info("Open sequence completed successfully (%s)", message)

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
        """
        Close: DOCK over BLE (only if currently lifted) → lift down → slide close.

        Used for every close() call — the normal path from OPEN, recovery
        from ERROR/an unknown starting position, and interrupting an
        in-progress open. Live-checks lift_upper to decide whether DOCK is
        even needed rather than trusting which of those cases we're in:
        if the pad isn't lifted right now (already closed, or an open that
        was interrupted before the lift stage finished), the drone was
        never released, so there's nothing to dock — skip straight to
        lowering/closing, both of which safely no-op if already done.
        """
        try:
            if self._gpio.confirmed(self._gpio.is_lift_upper_triggered):
                # Stage 1: DOCK over BLE. Must complete before any motor
                # moves — never lift down or slide close while the drone
                # isn't confirmed docked/locked.
                self._transition(PadState.DOCKING, None)
                ok, message = self._run_docking_command(
                    lambda: self._docking.dock(), "DOCK"
                )
                if not ok:
                    self._handle_error(f"DOCK failed — pad remains open, not closing: {message}")
                    return
            else:
                logger.info("Not lifted — skipping DOCK, closing directly")

            # Stage 2: Lift down (NEMA 17 reverse)
            self._transition(PadState.CLOSING_LIFT, MotorId.LIFT)
            self._motor.run_until_limit(
                motor_id=MotorId.LIFT,
                direction=Direction.REVERSE,
                limit_check=self._gpio.is_lift_lower_triggered,
                cancel_event=self._cancel_event,
                pause_event=self._pause_event,
            )

            # Stage 3: Slide close (NEMA 23 reverse)
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

    # ----- Private: Helpers -----
    #
    # Multi-sample switch confirmation (self._gpio.confirmed(...), used
    # throughout this file) now lives on GPIOManager — motor_controller.py's
    # "already at the limit, skip the move" pre-check needs the exact same
    # patient confirmation this class's hard gates do, so it's shared in one
    # place rather than duplicated per caller.

    def _infer_physical_state(self) -> Optional[PadState]:
        """
        Determine the pad's real physical position from its limit switches.

        Returns None if the switches don't clearly, consistently agree on
        OPEN or CLOSED (e.g. mid-travel, or a switch fault) — callers must
        treat that as "unknown" and refuse to guess, never fall back to a
        default.
        """
        is_open = self._gpio.confirmed(self._gpio.is_open_limit_triggered)
        is_lifted = self._gpio.confirmed(self._gpio.is_lift_upper_triggered)
        is_slide_closed = self._gpio.confirmed(self._gpio.is_close_limit_triggered)
        is_lowered = self._gpio.confirmed(self._gpio.is_lift_lower_triggered)

        if is_open and is_lifted:
            return PadState.OPEN
        if is_slide_closed and is_lowered:
            return PadState.CLOSED
        return None

    def _infer_physical_state_patient(self, timeout_s: float = 5.0) -> Optional[PadState]:
        """
        Like _infer_physical_state(), but keeps retrying for up to
        `timeout_s` before giving up. Used only at startup, where a fresh
        power-on can need a moment longer than one confirmation window to
        settle — a genuinely ambiguous/mid-travel position still correctly
        returns None once the deadline passes.
        """
        deadline = time.monotonic() + timeout_s
        while True:
            state = self._infer_physical_state()
            if state is not None:
                return state
            if time.monotonic() >= deadline:
                return None
            time.sleep(0.5)

    def _describe_limit_switches(self) -> str:
        """Human-readable snapshot of all four limit switches, for error messages."""
        return (
            f"open_limit={self._gpio.is_open_limit_triggered()} "
            f"lift_upper={self._gpio.is_lift_upper_triggered()} "
            f"close_limit={self._gpio.is_close_limit_triggered()} "
            f"lift_lower={self._gpio.is_lift_lower_triggered()}"
        )

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

    # ----- Private: Docking (BLE) integration -----

    def _docking_precheck(self) -> Tuple[bool, str]:
        """Check whether a docking command could even be attempted right now."""
        if self._docking is None:
            return False, "docking controller is not available (bleak not installed)"
        if self._loop is None:
            return False, "docking event loop is not available"
        if not self._docking.is_connected:
            # Give the background reconnect loop a bounded window to land a
            # retry that's already in flight or imminent, rather than
            # failing the instant we happen to check — see
            # _DOCKING_CONNECT_WAIT_S.
            deadline = time.monotonic() + _DOCKING_CONNECT_WAIT_S
            while time.monotonic() < deadline and not self._docking.is_connected:
                time.sleep(0.5)
            if not self._docking.is_connected:
                return False, "docking system BLE is not connected"
        return True, "ok"

    def _run_docking_command(self, coro_fn, label: str) -> Tuple[bool, str]:
        """
        Run a DockingController coroutine (e.g. self._docking.undock) from
        this background sequence thread and block until it completes.

        `coro_fn` is a zero-arg callable that returns a fresh coroutine —
        called here so the coroutine is only ever created if we're actually
        going to schedule it (a coroutine that's created but never awaited
        raises a warning).

        The coroutine itself runs on the real asyncio event loop via
        run_coroutine_threadsafe(), so the BLE side stays properly async;
        only this sync thread blocks on the result.
        """
        ok, reason = self._docking_precheck()
        if not ok:
            return False, reason

        timeout = self._settings.docking_config.command_timeout_seconds + _DOCKING_BRIDGE_TIMEOUT_BUFFER_S
        future = asyncio.run_coroutine_threadsafe(coro_fn(), self._loop)
        # Tracked so emergency_stop() can cancel this specific in-flight call
        # rather than just setting cancel_event, which this blocking wait
        # never checks — without this, an emergency-stop mid-DOCK/UNDOCK
        # left the coroutine running for its own full timeout regardless,
        # and the next command would queue up behind DockingController's
        # command lock rather than actually being free to run.
        with self._lock:
            self._docking_future = future
        try:
            status = future.result(timeout=timeout)
            return True, f"{label} completed: {status.value}"
        except DockingError as exc:
            return False, str(exc)
        except concurrent.futures.CancelledError:
            return False, f"{label} cancelled (emergency stop)"
        except concurrent.futures.TimeoutError:
            future.cancel()
            return False, f"{label} timed out waiting for the BLE bridge call itself"
        finally:
            with self._lock:
                if self._docking_future is future:
                    self._docking_future = None

    def _reset_docking_best_effort(self) -> None:
        """Send RESET to the docking system during emergency_stop(). Never raises."""
        ok, reason = self._docking_precheck()
        if not ok:
            logger.warning("Emergency stop: skipping docking RESET — %s", reason)
            return
        try:
            future = asyncio.run_coroutine_threadsafe(self._docking.reset(), self._loop)
            future.result(timeout=10.0)
            logger.info("Emergency stop: docking system RESET sent")
        except Exception as exc:
            logger.error("Emergency stop: docking RESET failed: %s", exc)
