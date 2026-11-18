"""
Drone Pad API Service — Configuration Module

Centralized configuration for GPIO pin assignments, motor parameters,
and operational settings. All values can be overridden via environment
variables for deployment flexibility.
"""

from __future__ import annotations

import os
import logging
from dataclasses import dataclass, field
from enum import Enum, IntEnum
from typing import Optional

logger = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# Enumerations
# ---------------------------------------------------------------------------

class PadState(str, Enum):
    """All possible states of the drone pad."""
    CLOSED = "CLOSED"
    OPENING_SLIDE = "OPENING_SLIDE"
    OPENING_LIFT = "OPENING_LIFT"
    UNDOCKING = "UNDOCKING"  # Pad physically open, waiting for the dock controller's UNDOCK to complete
    OPEN = "OPEN"
    DOCKING = "DOCKING"      # Close requested; waiting for the dock controller's DOCK to complete before moving
    CLOSING_LIFT = "CLOSING_LIFT"
    CLOSING_SLIDE = "CLOSING_SLIDE"
    ERROR = "ERROR"


class MotorId(str, Enum):
    """Identifiers for the two motors."""
    OPENING = "OPENING"  # NEMA 23 — slide open/close
    LIFT = "LIFT"        # NEMA 17 — lift up/down


class Direction(IntEnum):
    """Motor direction represented as GPIO level."""
    FORWARD = 1
    REVERSE = 0


class DockStatus(str, Enum):
    """
    Status strings reported by the ESP32 dock-lock controller over USB serial.

    DOCKED/UNDOCKED are the two stable resting states — the firmware
    transitions directly into them (verified against its limit switches,
    never assumed) and stays there; there's no separate transient
    "*_COMPLETE" pulse that then reverts to a generic idle. UNKNOWN means
    the firmware itself can't currently verify its position from the
    switches — treat that the same as ERROR for any safety decision.
    """
    UNKNOWN = "UNKNOWN"     # Position not verified — parse failures also map here
    DOCKED = "DOCKED"        # Stable: drone secured
    UNDOCKED = "UNDOCKED"     # Stable: drone released, safe to fly
    UNDOCKING_M1 = "UNDOCKING_M1"
    UNDOCKING_M2 = "UNDOCKING_M2"
    DOCKING_M2 = "DOCKING_M2"
    DOCKING_M1 = "DOCKING_M1"
    DOCKING_PROP_OPEN = "DOCKING_PROP_OPEN"
    DOCKING_PROP_CLOSE = "DOCKING_PROP_CLOSE"
    PROP_CYCLE_OPEN = "PROP_CYCLE_OPEN"    # PROPCYCLE: propeller closer opening (M1/M2 untouched)
    PROP_CYCLE_CLOSE = "PROP_CYCLE_CLOSE"  # PROPCYCLE: propeller closer closing again
    TESTING = "TESTING"      # Bench jog (JOG1/2/3) in progress — firmware is busy
    ERROR = "ERROR"


class DockMode(str, Enum):
    """
    What the pad's open/close sequences ask of the dock controller.

    PROPELLER_ONLY: the two docking motors (M1/M2) are never used.
        open  = slide open -> lift up                  (no UNDOCK)
        close = propeller closer cycle -> lift down -> slide close
    FULL: the original behaviour.
        open  = slide open -> lift up -> UNDOCK
        close = DOCK (M2 -> M1 -> propeller open -> close) -> lift down -> slide close
    """
    PROPELLER_ONLY = "propeller_only"
    FULL = "full"


# ---------------------------------------------------------------------------
# Pin Configuration Dataclasses
# ---------------------------------------------------------------------------

DEFAULT_DOCK_PORT = "/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0"


@dataclass(frozen=True)
class StepperPins:
    """GPIO pin assignments for a stepper motor driver."""
    step: int
    direction: int
    enable: int
    direction_inverted: bool = False  # True if DIR wiring is reversed on this driver


@dataclass(frozen=True)
class LimitSwitchPins:
    """GPIO pin assignments for all limit switches."""
    open_limit: int       # Triggered when pad is fully slid open
    close_limit: int      # Triggered when pad is fully slid closed
    lift_upper: int       # Triggered when pad is fully lifted up
    lift_lower: int       # Triggered when pad is fully lowered down


@dataclass(frozen=True)
class MotorConfig:
    """Operational parameters for a stepper motor."""
    step_delay_us: int          # Microseconds between step pulses
    timeout_seconds: float      # Max time before watchdog kills the operation
    enable_active_low: bool     # True = pull LOW to enable the driver


@dataclass(frozen=True)
class SwitchConfig:
    """Operational parameters for limit switches."""
    debounce_ms: int            # Debounce time in milliseconds
    active_low: bool            # True = switch pulls pin LOW when triggered


@dataclass(frozen=True)
class DockingConfig:
    """Connection parameters for the ESP32 dock-lock controller (USB serial)."""
    # Stable udev name of the ESP32 board's CP2102 USB-UART. If it's missing,
    # DockingController falls back to the single CP210x device present.
    port: str = DEFAULT_DOCK_PORT
    baudrate: int = 115200
    # Total time connect() keeps asking STATUS before giving up. Covers the
    # firmware's ~3 s boot delay, during which it ignores serial input.
    connect_timeout_seconds: float = 10.0
    # Max wait for DOCKED / UNDOCKED / ERROR. A full DOCK cycle
    # (M1 -> M2 -> PROP_OPEN -> PROP_CLOSE) has been observed taking
    # ~60-90s on real hardware — 45s was set before the propeller-closer
    # motor (M3) was wired up and cut this off mid-cycle. Generous margin
    # over the real timing, not a tight bound.
    command_timeout_seconds: float = 120.0


# ---------------------------------------------------------------------------
# Main Settings
# ---------------------------------------------------------------------------

@dataclass
class Settings:
    """
    Application-wide settings. Constructed from environment variables
    with sensible defaults for production hardware.
    """

    # --- Server ---
    host: str = "0.0.0.0"
    port: int = 8000
    log_level: str = "INFO"

    # --- Simulation ---
    simulation_mode: bool = False  # Auto-detected if RPi.GPIO unavailable

    # --- GPIO Pin Assignments ---
    opening_motor_pins: StepperPins = field(
        default_factory=lambda: StepperPins(step=14, direction=16, enable=12, direction_inverted=True)
    )
    lift_motor_pins: StepperPins = field(
        default_factory=lambda: StepperPins(step=26, direction=13, enable=11)
    )
    limit_switch_pins: LimitSwitchPins = field(
        default_factory=lambda: LimitSwitchPins(
            open_limit=4,
            close_limit=10,
            lift_upper=9,
            lift_lower=3,
        )
    )

    # --- Motor Parameters ---
    opening_motor_config: MotorConfig = field(
        default_factory=lambda: MotorConfig(
            step_delay_us=750,        # 750μs/step — was 500 (the enforced floor)
            timeout_seconds=60.0,     # 60s max for full slide travel
            enable_active_low=True,
        )
    )
    lift_motor_config: MotorConfig = field(
        default_factory=lambda: MotorConfig(
            step_delay_us=550,        # 550μs/step — was 1100. Driver DIP switches
                                       # changed (microstepping), so re-tuned faster.
                                       # Keep above _MIN_STEP_DELAY_US (500) in
                                       # gpio_manager.py or it's silently clamped.
            timeout_seconds=30.0,     # 30s max for full lift travel
            enable_active_low=True,
        )
    )

    # --- Limit Switch Parameters ---
    switch_config: SwitchConfig = field(
        default_factory=lambda: SwitchConfig(
            debounce_ms=50,           # 50ms debounce
            active_low=True,          # Switch connects GPIO to GND
        )
    )

    # --- Docking System (ESP32, USB serial) ---
    docking_config: DockingConfig = field(default_factory=DockingConfig)

    # Startup default for the dock mode. A mode chosen at runtime through
    # PUT /api/dock/mode is saved to dock_mode_file and takes precedence.
    dock_mode: DockMode = DockMode.PROPELLER_ONLY

    # --- Robot E-stop (physical button, served by the robot control system) ---
    # The pad follows the robot's physical emergency stop: it stops and locks
    # while the button is pressed and resets when it is released. See
    # estop_monitor.py.
    estop_enabled: bool = True
    estop_url: str = "http://127.0.0.1:8770/estop"
    estop_poll_interval_s: float = 0.1
    # True: an unreachable E-stop hub counts as a pressed E-stop.
    # False: keep the last known state and only log a warning.
    estop_fail_safe: bool = False

    dock_mode_file: str = field(
        default_factory=lambda: os.path.join(os.path.dirname(os.path.abspath(__file__)), "dock_mode.json")
    )

    def get_motor_pins(self, motor_id: MotorId) -> StepperPins:
        """Return the pin configuration for a given motor."""
        if motor_id == MotorId.OPENING:
            return self.opening_motor_pins
        return self.lift_motor_pins

    def get_motor_config(self, motor_id: MotorId) -> MotorConfig:
        """Return the operational config for a given motor."""
        if motor_id == MotorId.OPENING:
            return self.opening_motor_config
        return self.lift_motor_config


def _env_int(name: str, default: int) -> int:
    """Read an integer from an environment variable."""
    raw = os.environ.get(name)
    if raw is None:
        return default
    try:
        return int(raw)
    except ValueError:
        logger.warning(
            "Invalid integer for env var %s=%r, using default %d",
            name, raw, default,
        )
        return default


def _env_float(name: str, default: float) -> float:
    """Read a float from an environment variable."""
    raw = os.environ.get(name)
    if raw is None:
        return default
    try:
        return float(raw)
    except ValueError:
        logger.warning(
            "Invalid float for env var %s=%r, using default %f",
            name, raw, default,
        )
        return default


def _env_bool(name: str, default: bool) -> bool:
    """Read a boolean from an environment variable."""
    raw = os.environ.get(name)
    if raw is None:
        return default
    return raw.strip().lower() in ("1", "true", "yes")


def _env_dock_mode(name: str, default: DockMode) -> DockMode:
    """Read a DockMode from an environment variable."""
    raw = os.environ.get(name)
    if raw is None:
        return default
    try:
        return DockMode(raw.strip().lower())
    except ValueError:
        logger.warning(
            "Invalid dock mode %s=%r (use %s), using default %s",
            name, raw, " or ".join(m.value for m in DockMode), default.value,
        )
        return default


def load_saved_dock_mode(path: str) -> Optional[DockMode]:
    """Return the dock mode saved by PUT /api/dock/mode, or None if there isn't one."""
    import json
    try:
        with open(path, "r") as f:
            return DockMode(json.load(f)["mode"])
    except FileNotFoundError:
        return None
    except Exception as exc:
        logger.warning("Ignoring unreadable dock mode file %s: %s", path, exc)
        return None


def save_dock_mode(path: str, mode: DockMode) -> None:
    """Persist the dock mode atomically so it survives a service restart."""
    import json
    tmp = path + ".tmp"
    with open(tmp, "w") as f:
        json.dump({"mode": mode.value}, f)
        f.flush()
        os.fsync(f.fileno())
    os.replace(tmp, path)


def _load_dotenv() -> None:
    """
    Load `.env` from this directory into os.environ, if it exists.

    Real environment variables (e.g. systemd's Environment= lines) take
    precedence — a value already set is never overridden. This is what makes
    the documented `cp .env.example .env` workflow actually work, and gives
    the service and the tools/ scripts one shared place for local overrides
    such as re-mapped GPIO pins.
    """
    path = os.path.join(os.path.dirname(os.path.abspath(__file__)), ".env")
    if not os.path.exists(path):
        return
    try:
        from dotenv import load_dotenv
    except ImportError:
        logger.warning("%s exists but python-dotenv is not installed — ignoring it", path)
        return
    load_dotenv(path, override=False)
    logger.info("Loaded local overrides from %s", path)


def load_settings() -> Settings:
    """
    Build a Settings instance from environment variables.

    Environment variable names follow the pattern:
        DRONE_PAD_<SECTION>_<PARAM>

    Examples:
        DRONE_PAD_PORT=9000
        DRONE_PAD_OPENING_STEP_DELAY_US=400
        DRONE_PAD_SIMULATION=true
    """
    _load_dotenv()
    settings = Settings(
        host=os.environ.get("DRONE_PAD_HOST", "0.0.0.0"),
        port=_env_int("DRONE_PAD_PORT", 8000),
        log_level=os.environ.get("DRONE_PAD_LOG_LEVEL", "INFO").upper(),
        simulation_mode=_env_bool("DRONE_PAD_SIMULATION", False),
        opening_motor_pins=StepperPins(
            step=_env_int("DRONE_PAD_OPENING_STEP_PIN", 14),
            direction=_env_int("DRONE_PAD_OPENING_DIR_PIN", 16),
            enable=_env_int("DRONE_PAD_OPENING_ENABLE_PIN", 12),
            direction_inverted=_env_bool("DRONE_PAD_OPENING_DIR_INVERTED", True),
        ),
        lift_motor_pins=StepperPins(
            step=_env_int("DRONE_PAD_LIFT_STEP_PIN", 26),
            direction=_env_int("DRONE_PAD_LIFT_DIR_PIN", 13),
            enable=_env_int("DRONE_PAD_LIFT_ENABLE_PIN", 11),
        ),
        limit_switch_pins=LimitSwitchPins(
            open_limit=_env_int("DRONE_PAD_OPEN_LIMIT_PIN", 4),
            close_limit=_env_int("DRONE_PAD_CLOSE_LIMIT_PIN", 10),
            lift_upper=_env_int("DRONE_PAD_LIFT_UPPER_PIN", 9),
            lift_lower=_env_int("DRONE_PAD_LIFT_LOWER_PIN", 3),
        ),
        opening_motor_config=MotorConfig(
            step_delay_us=_env_int("DRONE_PAD_OPENING_STEP_DELAY_US", 750),
            timeout_seconds=_env_float("DRONE_PAD_OPENING_TIMEOUT_S", 160.0),
            enable_active_low=_env_bool("DRONE_PAD_OPENING_ENABLE_ACTIVE_LOW", True),
        ),
        lift_motor_config=MotorConfig(
            step_delay_us=_env_int("DRONE_PAD_LIFT_STEP_DELAY_US", 550),
            timeout_seconds=_env_float("DRONE_PAD_LIFT_TIMEOUT_S", 30.0),
            enable_active_low=_env_bool("DRONE_PAD_LIFT_ENABLE_ACTIVE_LOW", True),
        ),
        switch_config=SwitchConfig(
            debounce_ms=_env_int("DRONE_PAD_SWITCH_DEBOUNCE_MS", 50),
            active_low=_env_bool("DRONE_PAD_SWITCH_ACTIVE_LOW", False),
        ),
        docking_config=DockingConfig(
            port=os.environ.get("DRONE_PAD_DOCK_PORT", DEFAULT_DOCK_PORT),
            baudrate=_env_int("DRONE_PAD_DOCK_BAUD", 115200),
            connect_timeout_seconds=_env_float("DRONE_PAD_DOCK_CONNECT_TIMEOUT_S", 10.0),
            command_timeout_seconds=_env_float("DRONE_PAD_DOCK_COMMAND_TIMEOUT_S", 120.0),
        ),
        dock_mode=_env_dock_mode("DRONE_PAD_DOCK_MODE", DockMode.PROPELLER_ONLY),
        estop_enabled=_env_bool("DRONE_PAD_ESTOP_ENABLED", True),
        estop_url=os.environ.get("DRONE_PAD_ESTOP_URL", "http://127.0.0.1:8770/estop"),
        estop_poll_interval_s=_env_float("DRONE_PAD_ESTOP_POLL_INTERVAL_S", 0.1),
        estop_fail_safe=_env_bool("DRONE_PAD_ESTOP_FAIL_SAFE", False),
    )
    settings.dock_mode_file = os.environ.get("DRONE_PAD_DOCK_MODE_FILE", settings.dock_mode_file)

    logger.info("Settings loaded: simulation_mode=%s", settings.simulation_mode)
    logger.debug("Opening motor pins: %s", settings.opening_motor_pins)
    logger.debug("Lift motor pins: %s", settings.lift_motor_pins)
    logger.debug("Limit switch pins: %s", settings.limit_switch_pins)

    return settings
