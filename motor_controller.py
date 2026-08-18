"""
Drone Pad API Service — Motor Controller

Provides mid-level motor operations: stepping loops that respect
cancellation, pause/resume, limit switches, and timeout watchdogs.
"""

from __future__ import annotations

import logging
import time
import threading
from typing import Callable, Optional

from config import MotorId, Direction, Settings
from gpio_manager import GPIOManager

logger = logging.getLogger(__name__)


class MotorTimeoutError(Exception):
    """Raised when a motor operation exceeds its configured timeout."""
    pass


class MotorCancelledError(Exception):
    """Raised when a motor operation is cancelled by an interrupt."""
    pass


class MotorController:
    """
    Controls stepper motor operations with safety features.

    Each run_until_limit call:
      - Steps the motor continuously until a limit switch triggers
      - Respects a cancel_event (for open→close interrupts)
      - Respects a pause_event (for toggle pause/resume)
      - Enforces a timeout watchdog
      - Properly enables/disables the motor driver
    """

    def __init__(self, gpio: GPIOManager, settings: Settings) -> None:
        self._gpio = gpio
        self._settings = settings

    def run_until_limit(
        self,
        motor_id: MotorId,
        direction: Direction,
        limit_check: Callable[[], bool],
        cancel_event: threading.Event,
        pause_event: threading.Event,
    ) -> bool:
        """
        Step the motor until the limit switch triggers.

        Args:
            motor_id:     Which motor to drive.
            direction:    Direction to rotate.
            limit_check:  Callable returning True when the target limit
                          switch is triggered.
            cancel_event: If set, abort immediately (interrupt scenario).
            pause_event:  If cleared, pause stepping. If set, resume.
                          Starts in "set" (running) state.

        Returns:
            True if the limit switch was reached successfully.

        Raises:
            MotorTimeoutError: If the operation exceeds the configured
                               timeout without reaching the limit.
            MotorCancelledError: If the cancel_event was set.
        """
        config = self._settings.get_motor_config(motor_id)
        step_count = 0
        start_time = time.monotonic()

        logger.info(
            "Motor %s starting: direction=%s, timeout=%.1fs",
            motor_id.value, direction.name, config.timeout_seconds,
        )

        # Check if already at the target limit before starting. Uses a
        # patient, multi-sample confirmation (not a single instantaneous
        # read) — this hardware has a known wiring-reliability issue where
        # a switch can briefly, spuriously misreport as triggered, which
        # would otherwise skip the motor move entirely on a single glitch
        # and leave the pad not actually where the caller thinks it is.
        if self._gpio.confirmed(limit_check):
            logger.info(
                "Motor %s: limit switch already triggered — skipping",
                motor_id.value,
            )
            return True

        # Configure and enable the motor
        self._gpio.set_direction(motor_id, direction)
        self._gpio.enable_motor(motor_id)

        try:
            while True:
                # --- Check cancellation ---
                if cancel_event.is_set():
                    logger.info(
                        "Motor %s: operation cancelled after %d steps",
                        motor_id.value, step_count,
                    )
                    raise MotorCancelledError(
                        f"Motor {motor_id.value} cancelled after {step_count} steps"
                    )

                # --- Check pause ---
                if not pause_event.is_set():
                    logger.info("Motor %s: PAUSED at step %d", motor_id.value, step_count)
                    # Wait until resumed or cancelled
                    while not pause_event.is_set():
                        if cancel_event.is_set():
                            raise MotorCancelledError(
                                f"Motor {motor_id.value} cancelled while paused"
                            )
                        time.sleep(0.01)  # 10ms poll interval while paused
                    logger.info("Motor %s: RESUMED at step %d", motor_id.value, step_count)

                # --- Check timeout (exclude paused time) ---
                elapsed = time.monotonic() - start_time
                if elapsed > config.timeout_seconds:
                    logger.error(
                        "Motor %s: TIMEOUT after %.1fs and %d steps",
                        motor_id.value, elapsed, step_count,
                    )
                    raise MotorTimeoutError(
                        f"Motor {motor_id.value} timed out after "
                        f"{elapsed:.1f}s and {step_count} steps"
                    )

                # --- Step ---
                self._gpio.step_pulse(motor_id)
                step_count += 1

                # --- Check limit switch ---
                if limit_check():
                    elapsed = time.monotonic() - start_time
                    logger.info(
                        "Motor %s: limit switch reached after %d steps (%.2fs)",
                        motor_id.value, step_count, elapsed,
                    )
                    return True

        finally:
            # Always disable the motor driver when done
            self._gpio.disable_motor(motor_id)
            logger.debug("Motor %s: driver disabled", motor_id.value)

    def emergency_stop(self) -> None:
        """Immediately disable all motor drivers."""
        logger.warning("EMERGENCY STOP — disabling all motors")
        self._gpio.disable_all_motors()
