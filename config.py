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
    OPEN = "OPEN"
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


# ---------------------------------------------------------------------------
# Pin Configuration Dataclasses
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class StepperPins:
    """GPIO pin assignments for a stepper motor driver."""
    step: int
    direction: int
    enable: int


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
        default_factory=lambda: StepperPins(step=14, direction=16, enable=12)
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
            step_delay_us=500,        # 500μs → ~1000 steps/sec
            timeout_seconds=60.0,     # 60s max for full slide travel
            enable_active_low=True,
        )
    )
    lift_motor_config: MotorConfig = field(
        default_factory=lambda: MotorConfig(
            step_delay_us=800,        # 800μs → ~625 steps/sec
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
    settings = Settings(
        host=os.environ.get("DRONE_PAD_HOST", "0.0.0.0"),
        port=_env_int("DRONE_PAD_PORT", 8000),
        log_level=os.environ.get("DRONE_PAD_LOG_LEVEL", "INFO").upper(),
        simulation_mode=_env_bool("DRONE_PAD_SIMULATION", False),
        opening_motor_pins=StepperPins(
            step=_env_int("DRONE_PAD_OPENING_STEP_PIN", 14),
            direction=_env_int("DRONE_PAD_OPENING_DIR_PIN", 16),
            enable=_env_int("DRONE_PAD_OPENING_ENABLE_PIN", 12),
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
            step_delay_us=_env_int("DRONE_PAD_OPENING_STEP_DELAY_US", 500),
            timeout_seconds=_env_float("DRONE_PAD_OPENING_TIMEOUT_S", 60.0),
            enable_active_low=_env_bool("DRONE_PAD_OPENING_ENABLE_ACTIVE_LOW", True),
        ),
        lift_motor_config=MotorConfig(
            step_delay_us=_env_int("DRONE_PAD_LIFT_STEP_DELAY_US", 800),
            timeout_seconds=_env_float("DRONE_PAD_LIFT_TIMEOUT_S", 30.0),
            enable_active_low=_env_bool("DRONE_PAD_LIFT_ENABLE_ACTIVE_LOW", True),
        ),
        switch_config=SwitchConfig(
            debounce_ms=_env_int("DRONE_PAD_SWITCH_DEBOUNCE_MS", 50),
            active_low=_env_bool("DRONE_PAD_SWITCH_ACTIVE_LOW", True),
        ),
    )

    logger.info("Settings loaded: simulation_mode=%s", settings.simulation_mode)
    logger.debug("Opening motor pins: %s", settings.opening_motor_pins)
    logger.debug("Lift motor pins: %s", settings.lift_motor_pins)
    logger.debug("Limit switch pins: %s", settings.limit_switch_pins)

    return settings
