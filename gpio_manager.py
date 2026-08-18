"""
Drone Pad API Service — GPIO Manager (Pi 5 / lgpio)

Handles all direct GPIO interactions using lgpio, which is the native
GPIO library for Raspberry Pi 5 (RP1 chip). Falls back to a simulation
stub when lgpio is unavailable (e.g. during development).

Key fixes over original:
  - Replaced time.sleep() in step_pulse() with busy-wait (_sleep_us)
    for accurate sub-millisecond timing — eliminates OS scheduler jitter
    that caused rough/drifting motor sound.
  - Added run_steps() using lgpio.tx_pulse() for hardware-timed pulse
    trains — offloads step generation to RP1, zero Python jitter.
  - Added run_motor_to_limit() for batched hardware pulses with
    interleaved limit switch checks.
  - Added acceleration ramp support (ramp_steps parameter) to prevent
    stall on startup at high speeds.
  - Simulation stub updated to match new method signatures.
"""

from __future__ import annotations

import logging
import time
import threading
from typing import Optional, Dict, Callable

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

# Minimum step delay enforced as a safety floor (µs).
# Below ~50 µs the RP1 pulse engine becomes unreliable.
_MIN_STEP_DELAY_US = 500

# Batch size for run_motor_to_limit(): number of hardware pulses per
# burst before checking limit switches. At 400 µs/step, 50 steps = 40 ms
# per batch — responsive enough for limit switch debounce at 50 ms.
_DEFAULT_BATCH_SIZE = 50


# ---------------------------------------------------------------------------
# Simulation GPIO Stub
# ---------------------------------------------------------------------------

class _SimulatedGPIO:
    """
    Minimal GPIO stub that logs pin operations for development/testing.
    Limit switches can be triggered programmatically via trigger_switch().
    """

    def __init__(self) -> None:
        self._pin_states: Dict[int, int] = {}
        self._switch_states: Dict[int, bool] = {}
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
        return 0 if triggered else 1  # Active LOW

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

    def sim_run_steps(self, pin: int, steps: int, half_delay_us: int) -> None:
        """Simulate a hardware pulse train (just logs, no actual timing)."""
        duration_ms = (steps * half_delay_us * 2) / 1000
        logger.debug(
            "[SIM] tx_pulse on GPIO %d: %d steps @ %d µs half-delay (~%.1f ms)",
            pin, steps, half_delay_us, duration_ms,
        )
        # In simulation, sleep the equivalent wall time so callers
        # that await completion behave realistically.
        time.sleep(duration_ms / 1000)


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
      - Hardware-accurate step pulse generation via lgpio.tx_pulse()
      - Acceleration ramping to prevent stall at startup
      - Clean shutdown

    Step generation strategy
    ------------------------
    Two modes are provided:

    1. step_pulse() — single step with busy-wait timing.
       Use when you need to check limit switches every step
       and the step delay is large enough (> 1 ms) that CPU
       burn is acceptable.

    2. run_steps() / run_motor_to_limit() — hardware pulse train
       via lgpio.tx_pulse(). Offloads pulse generation entirely to
       the RP1; Python only polls for completion. Use for all normal
       motion — smoothest output, no jitter.
    """

    def __init__(self, settings: Settings) -> None:
        self._settings = settings
        self._switch_config = settings.switch_config

        self._switch_last_state: Dict[int, bool] = {}
        self._switch_last_change: Dict[int, Optional[float]] = {}
        self._switch_lock = threading.Lock()

        self._claimed_pins: list[int] = []

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
            self._handle = lgpio.gpiochip_open(_GPIO_CHIP)
            logger.info("Opened gpiochip%d (handle=%d)", _GPIO_CHIP, self._handle)

        for motor_id in MotorId:
            pins = self._settings.get_motor_pins(motor_id)
            config = self._settings.get_motor_config(motor_id)
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

        lsp = self._settings.limit_switch_pins
        switch_pins = [
            ("open_limit",  lsp.open_limit),
            ("close_limit", lsp.close_limit),
            ("lift_upper",  lsp.lift_upper),
            ("lift_lower",  lsp.lift_lower),
        ]
        for name, pin in switch_pins:
            if self._simulation:
                self._sim.claim_input(pin, pull_up=True)
            else:
                lgpio.gpio_claim_input(self._handle, pin, lgpio.SET_PULL_UP)
                self._claimed_pins.append(pin)

        # Seed each switch's confirmed state from a real read, not an
        # assumed default — otherwise every switch reports "not triggered"
        # until it happens to be polled twice with a debounce window
        # between the calls, silently misreporting the pad's true physical
        # position on every restart until something exercises it.
        time.sleep(0.02)  # let freshly-claimed pull-ups settle (~20ms)
        for name, pin in switch_pins:
            raw_level = self._read(pin)
            raw_triggered = (raw_level == 0) if self._switch_config.active_low \
                            else (raw_level == 1)
            self._switch_last_state[pin] = raw_triggered
            self._switch_last_change[pin] = None
            logger.info(
                "Limit switch '%s' initialized on GPIO %d (initial=%s)",
                name, pin, "TRIGGERED" if raw_triggered else "released",
            )

        self._initialized = True
        logger.info("GPIO initialization complete")

    def cleanup(self) -> None:
        """Release all GPIO resources. Call on shutdown."""
        if not self._initialized:
            return

        logger.info("Cleaning up GPIO resources")

        for motor_id in MotorId:
            try:
                self.disable_motor(motor_id)
            except Exception:
                pass

        if self._simulation:
            self._sim.close()
        else:
            for pin in self._claimed_pins:
                try:
                    lgpio.gpio_free(self._handle, pin)
                except Exception:
                    pass
            self._claimed_pins.clear()

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
        level = int(direction)
        if pins.direction_inverted:
            level ^= 1
        self._write(pins.direction, level)
        # TB6600 requires ≥5 µs setup time after DIR change before first STEP pulse
        self._sleep_us(10)
        logger.debug("Motor %s direction set to %s", motor_id.value, direction.name)

    # ----- Step Pulse Generation -----

    def step_pulse(self, motor_id: MotorId) -> None:
        """
        Generate a single step pulse using busy-wait timing.

        Accurate to ~1 µs on Pi 5. Use this when you need to check
        limit switches on every individual step. For bulk motion use
        run_steps() or run_motor_to_limit() instead.

        NOTE: Burns CPU for the duration of the delay. Acceptable for
        short bursts; avoid for long continuous runs.
        """
        pins = self._settings.get_motor_pins(motor_id)
        config = self._settings.get_motor_config(motor_id)
        half_us = max(config.step_delay_us // 2, _MIN_STEP_DELAY_US // 2)

        self._write(pins.step, 1)
        self._sleep_us(half_us)
        self._write(pins.step, 0)
        self._sleep_us(half_us)

    def run_steps(
        self,
        motor_id: MotorId,
        steps: int,
        step_delay_us: Optional[int] = None,
    ) -> None:
        """
        Generate `steps` pulses using lgpio.tx_pulse() (hardware-timed).

        The RP1 generates the pulse train in hardware — no Python jitter,
        perfectly even step spacing, smooth motor motion.

        Args:
            motor_id:       Which motor to step.
            steps:          Number of step pulses to generate.
            step_delay_us:  Override step delay in µs. Defaults to the
                            motor's configured step_delay_us.
        """
        if steps <= 0:
            return

        pins = self._settings.get_motor_pins(motor_id)
        config = self._settings.get_motor_config(motor_id)
        delay = max(
            step_delay_us if step_delay_us is not None else config.step_delay_us,
            _MIN_STEP_DELAY_US,
        )
        half_us = delay // 2

        logger.debug(
            "Motor %s: run_steps(%d) @ %d µs/step",
            motor_id.value, steps, delay,
        )

        if self._simulation:
            self._sim.sim_run_steps(pins.step, steps, half_us)
            return

        # tx_pulse(handle, gpio, pulse_on_us, pulse_off_us, pulse_offset, pulse_count)
        lgpio.tx_pulse(self._handle, pins.step, half_us, half_us, 0, steps)

        # Poll until the hardware pulse train finishes
        while lgpio.tx_busy(self._handle, pins.step, lgpio.TX_PWM):
            time.sleep(0.001)  # 1 ms poll interval — coarse is fine here

    def run_steps_ramped(
        self,
        motor_id: MotorId,
        steps: int,
        ramp_steps: int = 200,
    ) -> None:
        """
        Run `steps` pulses with a linear acceleration/deceleration ramp.

        Ramps from the motor's step_delay_us down to _MIN_STEP_DELAY_US
        over `ramp_steps` steps, holds at top speed, then ramps back up.
        This prevents stall at startup and reduces resonance noise.

        Args:
            motor_id:    Which motor to step.
            steps:       Total number of step pulses.
            ramp_steps:  Steps over which to ramp up (and down). If
                         steps < 2 * ramp_steps, ramp is shortened to fit.
        """
        if steps <= 0:
            return

        config = self._settings.get_motor_config(motor_id)
        start_delay = config.step_delay_us
        end_delay = max(start_delay // 4, _MIN_STEP_DELAY_US)

        # Shorten ramp if total travel is too short for a full ramp
        ramp = min(ramp_steps, steps // 2)

        logger.debug(
            "Motor %s: run_steps_ramped(%d steps, ramp=%d, %d→%d µs)",
            motor_id.value, steps, ramp, start_delay, end_delay,
        )

        for i in range(steps):
            if ramp > 0 and i < ramp:
                # Accelerating: interpolate delay linearly downward
                t = i / ramp
                delay = int(start_delay + (end_delay - start_delay) * t)
            elif ramp > 0 and i >= (steps - ramp):
                # Decelerating: interpolate delay linearly upward
                t = (steps - i) / ramp
                delay = int(start_delay + (end_delay - start_delay) * t)
            else:
                delay = end_delay

            delay = max(delay, _MIN_STEP_DELAY_US)

            # Use hardware pulse for each step in the ramp for accuracy
            self.run_steps(motor_id, 1, step_delay_us=delay)

    def run_motor_to_limit(
        self,
        motor_id: MotorId,
        limit_check_fn: Callable[[], bool],
        ramp_steps: int = 200,
        batch_size: int = _DEFAULT_BATCH_SIZE,
    ) -> bool:
        """
        Run the motor until a limit switch triggers, using batched
        hardware pulses for smooth motion.

        Generates `batch_size` hardware pulses per burst, then checks
        the limit switch between bursts. This gives smooth motion while
        remaining responsive to limit switch events.

        Args:
            motor_id:       Which motor to run.
            limit_check_fn: Callable that returns True when the limit
                            switch is triggered (motion should stop).
            ramp_steps:     Steps for acceleration ramp at start.
                            Set to 0 to disable ramping.
            batch_size:     Steps per hardware burst. Smaller = more
                            responsive limit switch checks but more
                            Python overhead. Default 50 @ 400 µs =
                            40 ms per batch.

        Returns:
            True  if motion stopped because limit_check_fn() returned True.
            False if the motor was stopped by external means (e.g. timeout
                  handler cancelled the operation).
        """
        config = self._settings.get_motor_config(motor_id)
        start_delay = config.step_delay_us
        end_delay = max(start_delay // 4, _MIN_STEP_DELAY_US)

        step_count = 0

        logger.info(
            "Motor %s: running to limit (batch=%d, ramp=%d steps)",
            motor_id.value, batch_size, ramp_steps,
        )

        while True:
            # Check limit switch before each batch
            if limit_check_fn():
                logger.info(
                    "Motor %s: limit switch triggered after %d steps",
                    motor_id.value, step_count,
                )
                return True

            # Determine step delay for this batch (ramp logic)
            if ramp_steps > 0 and step_count < ramp_steps:
                t = step_count / ramp_steps
                batch_delay = int(start_delay + (end_delay - start_delay) * t)
                batch_delay = max(batch_delay, _MIN_STEP_DELAY_US)
            else:
                batch_delay = end_delay

            # Fire a hardware pulse batch
            self.run_steps(motor_id, batch_size, step_delay_us=batch_delay)
            step_count += batch_size

            # Check limit switch immediately after batch completes
            if limit_check_fn():
                logger.info(
                    "Motor %s: limit switch triggered after %d steps",
                    motor_id.value, step_count,
                )
                return True

        # Unreachable — caller's timeout/cancellation mechanism is responsible
        # for breaking out of this loop if needed (e.g. run in a thread).

    # ----- Limit Switch Reading -----

    def read_limit_switch(self, pin: int) -> bool:
        """
        Read a limit switch with software debouncing.

        Returns True if the switch is confirmed triggered (debounced).
        Debounce strategy: record the time of first state change, only
        accept the new state after it has been stable for debounce_ms.
        Does NOT reset the timer on every fluctuation.
        """
        raw_level = self._read(pin)
        raw_triggered = (raw_level == 0) if self._switch_config.active_low \
                        else (raw_level == 1)

        now = time.monotonic()
        debounce_s = self._switch_config.debounce_ms / 1000.0

        with self._switch_lock:
            confirmed = self._switch_last_state.get(pin, False)
            pending_since = self._switch_last_change.get(pin, None)

            if raw_triggered == confirmed:
                # Still matches confirmed state — reset any pending transition
                self._switch_last_change[pin] = None
                return confirmed

            # raw differs from confirmed — we have a pending transition
            if pending_since is None:
                # First time we saw this new state — record when it started
                self._switch_last_change[pin] = now
                return confirmed  # not yet confirmed

            if (now - pending_since) >= debounce_s:
                # New state has been stable long enough — confirm it
                self._switch_last_state[pin] = raw_triggered
                self._switch_last_change[pin] = None
                logger.debug(
                    "Limit switch GPIO %d confirmed → %s",
                    pin, "TRIGGERED" if raw_triggered else "RELEASED"
                )
                return raw_triggered

            # Still within debounce window
            return confirmed

    def is_open_limit_triggered(self) -> bool:
        return self.read_limit_switch(self._settings.limit_switch_pins.open_limit)

    def is_close_limit_triggered(self) -> bool:
        return self.read_limit_switch(self._settings.limit_switch_pins.close_limit)

    def is_lift_upper_triggered(self) -> bool:
        return self.read_limit_switch(self._settings.limit_switch_pins.lift_upper)

    def is_lift_lower_triggered(self) -> bool:
        return self.read_limit_switch(self._settings.limit_switch_pins.lift_lower)

    def confirmed(
        self,
        check_fn: Callable[[], bool],
        samples: int = 3,
        interval_s: float = 0.15,
        patience_s: float = 2.0,
    ) -> bool:
        """
        True once `check_fn()` (typically one of the is_X_triggered methods
        above) reads True on `samples` *consecutive* calls within
        `patience_s` total.

        A single instantaneous read — even the debounced ones above — isn't
        trustworthy enough to gate an irreversible action (arming a motor,
        skipping a motor move entirely because it's "already there",
        telling the drone dock to release/lock): this hardware has a known
        wiring-reliability issue where a switch can briefly, spuriously
        misreport in either direction. A disagreeing sample only resets the
        consecutive count here, it doesn't fail immediately — that was
        tried first and made things worse, since it meant a switch that had
        genuinely, correctly settled true could still fail an entire
        open()/close() sequence over one momentary blip. Failing still
        requires that `samples` consecutive true reads never happen within
        the whole patience window — a switch stuck genuinely false, or one
        flickering continuously, correctly still fails.
        """
        deadline = time.monotonic() + patience_s
        consecutive = 0
        while time.monotonic() < deadline:
            if check_fn():
                consecutive += 1
                if consecutive >= samples:
                    return True
            else:
                consecutive = 0
            time.sleep(interval_s)
        return False

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
        if self._simulation:
            self._sim.write(pin, level)
        else:
            lgpio.gpio_write(self._handle, pin, level)

    def _read(self, pin: int) -> int:
        if self._simulation:
            return self._sim.read(pin)
        else:
            return lgpio.gpio_read(self._handle, pin)

    @staticmethod
    def _sleep_us(us: int) -> None:
        """
        Busy-wait for `us` microseconds.

        Uses time.perf_counter() for sub-millisecond accuracy, bypassing
        the OS scheduler entirely. Burns CPU for the wait duration — only
        use for short delays (< 5 ms). For longer waits use time.sleep().
        """
        if us <= 0:
            return
        deadline = time.perf_counter() + us * 1e-6
        while time.perf_counter() < deadline:
            pass

    # ----- Simulation Helpers -----

    def sim_trigger_switch(self, pin: int, triggered: bool = True) -> None:
        """Trigger a simulated limit switch (simulation mode only)."""
        if self._sim is not None:
            self._sim.trigger_switch(pin, triggered)
            with self._switch_lock:
                self._switch_last_state[pin] = triggered
                self._switch_last_change[pin] = time.monotonic()
        else:
            logger.warning("sim_trigger_switch called but not in simulation mode")