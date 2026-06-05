"""
Drone Pad API Service — GPIO Manager (Pi 5 / lgpio)

Handles all direct GPIO interactions using lgpio, which is the native
GPIO library for Raspberry Pi 5 (RP1 chip). Falls back to a simulation
stub when lgpio is unavailable (e.g. during development).
"""

from __future__ import annotations

import logging
import time
import threading
from typing import Optional, Dict

from config import (
    Settings, StepperPins, LimitSwitchPins, SwitchConfig,
    MotorId, Direction,
)

logger = logging.getLogger(__name__)

# ---------------------------------------------------------------------------
# Try to import lgpio; fall back to simulation stub
# ---------------------------------------------------------------------------

try:
    import lgpio  # type: ignore[import-untyped]
    _HAS_LGPIO = True
except ImportError:
    _HAS_LGPIO = False
    lgpio = None  # type: ignore[assignment]

# Pi 5 user GPIO chip number
_GPIO_CHIP = 0


# ---------------------------------------------------------------------------
# Simulation GPIO Stub
# ---------------------------------------------------------------------------

class _SimulatedGPIO:
    """
    Minimal GPIO stub that logs pin operations for development/testing.
    Limit switches can be triggered programmatically via `trigger_switch()`.
    """

    def __init__(self) -> None:
        self._pin_states: Dict[int, int] = {}
        self._switch_states: Dict[int, bool] = {}  # True = triggered
        self._lock = threading.Lock()

    def claim_output(self, pin: int, initial: int = 0) -> None:
        with self._lock:
            self._pin_states[pin] = initial
        logger.debug("[SIM] GPIO %d claimed as OUTPUT (initial=%d)", pin, initial)

    def claim_input(self, pin: int, pull_up: bool = True) -> None:
        with self._lock:
            self._pin_states[pin] = 1 if pull_up else 0
        logger.debug("[SIM] GPIO %d claimed as INPUT (pull_up=%s)", pin, pull_up)

    def write(self, pin: int, level: int) -> None:
        with self._lock:
            self._pin_states[pin] = level

    def read(self, pin: int) -> int:
        with self._lock:
            triggered = self._switch_states.get(pin, False)
        # Active LOW: return 0 when triggered, 1 when not
        return 0 if triggered else 1

    def free(self, pin: int) -> None:
        with self._lock:
            self._pin_states.pop(pin, None)
        logger.debug("[SIM] GPIO %d freed", pin)

    def close(self) -> None:
        logger.info("[SIM] GPIO cleanup")
        with self._lock:
            self._pin_states.clear()
            self._switch_states.clear()

    def trigger_switch(self, pin: int, triggered: bool = True) -> None:
        """Programmatically trigger or release a simulated limit switch."""
        with self._lock:
            self._switch_states[pin] = triggered
        state_str = "TRIGGERED" if triggered else "RELEASED"
        logger.info("[SIM] Limit switch on GPIO %d → %s", pin, state_str)


# ---------------------------------------------------------------------------
# GPIO Manager
# ---------------------------------------------------------------------------

class GPIOManager:
    """
    Manages all GPIO interactions for the drone pad using lgpio.

    Responsibilities:
      - Initialize motor driver pins (STEP, DIR, ENABLE) as outputs
      - Initialize limit switch pins as inputs with pull-ups
      - Provide debounced limit switch reading
      - Step pulse generation
      - Clean shutdown
    """

    def __init__(self, settings: Settings) -> None:
        self._settings = settings
        self._switch_config = settings.switch_config

        # Last confirmed switch states and timestamps for debouncing
        self._switch_last_state: Dict[int, bool] = {}
        self._switch_last_change: Dict[int, float] = {}
        self._switch_lock = threading.Lock()

        # Track claimed pins for cleanup
        self._claimed_pins: list[int] = []

        # Determine if we should use real GPIO or simulation
        if settings.simulation_mode or not _HAS_LGPIO:
            if not _HAS_LGPIO and not settings.simulation_mode:
                logger.warning(
                    "lgpio not available — entering simulation mode automatically"
                )
            self._sim = _SimulatedGPIO()
            self._handle: Optional[int] = None
            self._simulation = True
        else:
            self._sim = None
            self._handle = None
            self._simulation = False

        self._initialized = False

    @property
    def is_simulation(self) -> bool:
        return self._simulation

    # ----- Initialization / Cleanup -----

    def initialize(self) -> None:
        """Set up all GPIO pins. Must be called once at startup."""
        if self._initialized:
            logger.warning("GPIO already initialized — skipping")
            return

        logger.info("Initializing GPIO (simulation=%s)", self._simulation)

        if not self._simulation:
            # Open the GPIO chip
            self._handle = lgpio.gpiochip_open(_GPIO_CHIP)
            logger.info("Opened gpiochip%d (handle=%d)", _GPIO_CHIP, self._handle)

        # Set up motor driver pins
        for motor_id in MotorId:
            pins = self._settings.get_motor_pins(motor_id)
            config = self._settings.get_motor_config(motor_id)

            # Disable level for initial state (active LOW → HIGH to disable)
            disable_level = 1 if config.enable_active_low else 0

            if self._simulation:
                self._sim.claim_output(pins.step, 0)
                self._sim.claim_output(pins.direction, 0)
                self._sim.claim_output(pins.enable, disable_level)
            else:
                lgpio.gpio_claim_output(self._handle, pins.step, 0)
                self._claimed_pins.append(pins.step)
                lgpio.gpio_claim_output(self._handle, pins.direction, 0)
                self._claimed_pins.append(pins.direction)
                lgpio.gpio_claim_output(self._handle, pins.enable, disable_level)
                self._claimed_pins.append(pins.enable)

            logger.info(
                "Motor %s initialized: STEP=%d DIR=%d EN=%d",
                motor_id.value, pins.step, pins.direction, pins.enable,
            )

        # Set up limit switch pins
        lsp = self._settings.limit_switch_pins
        for name, pin in [
            ("open_limit", lsp.open_limit),
            ("close_limit", lsp.close_limit),
            ("lift_upper", lsp.lift_upper),
            ("lift_lower", lsp.lift_lower),
        ]:
            if self._simulation:
                self._sim.claim_input(pin, pull_up=True)
            else:
                # lgpio flags: SET_PULL_UP = 1<<16
                lgpio.gpio_claim_input(self._handle, pin, lgpio.SET_PULL_UP)
                self._claimed_pins.append(pin)

            self._switch_last_state[pin] = False
            self._switch_last_change[pin] = 0.0
            logger.info("Limit switch '%s' initialized on GPIO %d", name, pin)

        self._initialized = True
        logger.info("GPIO initialization complete")

    def cleanup(self) -> None:
        """Release all GPIO resources. Call on shutdown."""
        if not self._initialized:
            return

        logger.info("Cleaning up GPIO resources")

        # Disable all motors before cleanup
        for motor_id in MotorId:
            try:
                self.disable_motor(motor_id)
            except Exception:
                pass

        if self._simulation:
            self._sim.close()
        else:
            # Free all claimed pins
            for pin in self._claimed_pins:
                try:
                    lgpio.gpio_free(self._handle, pin)
                except Exception:
                    pass
            self._claimed_pins.clear()

            # Close the chip handle
            if self._handle is not None:
                try:
                    lgpio.gpiochip_close(self._handle)
                except Exception:
                    pass
                self._handle = None

        self._initialized = False
        logger.info("GPIO cleanup complete")

    # ----- Motor Control -----

    def enable_motor(self, motor_id: MotorId) -> None:
        """Enable the stepper driver for the given motor."""
        pins = self._settings.get_motor_pins(motor_id)
        config = self._settings.get_motor_config(motor_id)
        enable_level = 0 if config.enable_active_low else 1
        self._write(pins.enable, enable_level)
        logger.debug("Motor %s ENABLED", motor_id.value)

    def disable_motor(self, motor_id: MotorId) -> None:
        """Disable the stepper driver for the given motor."""
        pins = self._settings.get_motor_pins(motor_id)
        config = self._settings.get_motor_config(motor_id)
        disable_level = 1 if config.enable_active_low else 0
        self._write(pins.enable, disable_level)
        logger.debug("Motor %s DISABLED", motor_id.value)

    def set_direction(self, motor_id: MotorId, direction: Direction) -> None:
        """Set the rotation direction for the given motor."""
        pins = self._settings.get_motor_pins(motor_id)
        self._write(pins.direction, int(direction))
        logger.debug("Motor %s direction set to %s", motor_id.value, direction.name)

    def step_pulse(self, motor_id: MotorId) -> None:
        """
        Generate a single step pulse for the given motor.

        The pulse width is half the configured step delay to maintain
        a proper duty cycle.
        """
        pins = self._settings.get_motor_pins(motor_id)
        config = self._settings.get_motor_config(motor_id)

        half_delay = config.step_delay_us / 2_000_000  # Convert μs to seconds, halved

        self._write(pins.step, 1)
        time.sleep(half_delay)
        self._write(pins.step, 0)
        time.sleep(half_delay)

    # ----- Limit Switch Reading -----

    def read_limit_switch(self, pin: int) -> bool:
        """
        Read a limit switch with software debouncing.

        Returns True if the switch is confirmed triggered (debounced).
        Uses a simple time-based debounce: the raw state must remain
        stable for `debounce_ms` before being accepted.
        """
        raw_level = self._read(pin)

        # Determine if the switch is physically triggered
        if self._switch_config.active_low:
            raw_triggered = (raw_level == 0)
        else:
            raw_triggered = (raw_level == 1)

        now = time.monotonic()

        with self._switch_lock:
            last_state = self._switch_last_state.get(pin, False)
            last_change = self._switch_last_change.get(pin, 0.0)

            if raw_triggered != last_state:
                # State changed — reset debounce timer
                self._switch_last_change[pin] = now
                # Return the old state until debounce settles
                return last_state

            # State is the same as last reading
            debounce_s = self._switch_config.debounce_ms / 1000.0
            if (now - last_change) >= debounce_s:
                # Debounce period passed — accept new state
                self._switch_last_state[pin] = raw_triggered
                return raw_triggered

            # Still within debounce window — return last confirmed state
            return last_state

    def is_open_limit_triggered(self) -> bool:
        """Check if the open limit switch is triggered."""
        return self.read_limit_switch(self._settings.limit_switch_pins.open_limit)

    def is_close_limit_triggered(self) -> bool:
        """Check if the close limit switch is triggered."""
        return self.read_limit_switch(self._settings.limit_switch_pins.close_limit)

    def is_lift_upper_triggered(self) -> bool:
        """Check if the lift upper limit switch is triggered."""
        return self.read_limit_switch(self._settings.limit_switch_pins.lift_upper)

    def is_lift_lower_triggered(self) -> bool:
        """Check if the lift lower limit switch is triggered."""
        return self.read_limit_switch(self._settings.limit_switch_pins.lift_lower)

    def disable_all_motors(self) -> None:
        """Emergency: disable all motor drivers immediately."""
        for motor_id in MotorId:
            try:
                self.disable_motor(motor_id)
            except Exception as exc:
                logger.error(
                    "Failed to disable motor %s during emergency stop: %s",
                    motor_id.value, exc,
                )

    # ----- Low-level I/O -----

    def _write(self, pin: int, level: int) -> None:
        """Write a level to a GPIO pin."""
        if self._simulation:
            self._sim.write(pin, level)
        else:
            lgpio.gpio_write(self._handle, pin, level)

    def _read(self, pin: int) -> int:
        """Read the level of a GPIO pin."""
        if self._simulation:
            return self._sim.read(pin)
        else:
            return lgpio.gpio_read(self._handle, pin)

    # ----- Simulation helpers -----

    def sim_trigger_switch(self, pin: int, triggered: bool = True) -> None:
        """Trigger a simulated limit switch (simulation mode only)."""
        if self._sim is not None:
            self._sim.trigger_switch(pin, triggered)
            # Also update debounce state immediately for simulation
            with self._switch_lock:
                self._switch_last_state[pin] = triggered
                self._switch_last_change[pin] = time.monotonic()
        else:
            logger.warning("sim_trigger_switch called but not in simulation mode")
