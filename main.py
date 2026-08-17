"""
Drone Pad API Service — FastAPI Application

Production-ready REST API for controlling the drone pad. Provides
endpoints for open/close/toggle commands, status monitoring, emergency
stop, and event log retrieval.
"""

from __future__ import annotations

import asyncio
import logging
import sys
import os
from contextlib import asynccontextmanager
from datetime import datetime, timezone
from typing import Literal, Optional, List

from fastapi import FastAPI, HTTPException, Request
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import JSONResponse
from pydantic import BaseModel, Field

from config import Direction, MotorId, PadState, load_settings, Settings
from gpio_manager import GPIOManager
from motor_controller import MotorController
from pad_state_machine import PadStateMachine, PadStatus, EventEntry
from docking_controller import DockingController, DockingError
from debug_controller import DebugMotorController

# ---------------------------------------------------------------------------
# Logging Setup
# ---------------------------------------------------------------------------

def setup_logging(level: str = "INFO") -> None:
    """Configure structured logging for the application."""
    numeric_level = getattr(logging, level.upper(), logging.INFO)
    logging.basicConfig(
        level=numeric_level,
        format="%(asctime)s | %(levelname)-8s | %(name)-28s | %(message)s",
        datefmt="%Y-%m-%d %H:%M:%S",
        stream=sys.stdout,
    )
    # Reduce noise from third-party loggers
    logging.getLogger("uvicorn.access").setLevel(logging.WARNING)


# ---------------------------------------------------------------------------
# Pydantic Response Models
# ---------------------------------------------------------------------------

class CommandResponse(BaseModel):
    """Standard response for all command endpoints."""
    success: bool = Field(..., description="Whether the command was accepted")
    message: str = Field(..., description="Human-readable result message")
    state: str = Field(..., description="Current pad state after the command")
    timestamp: str = Field(..., description="ISO 8601 UTC timestamp")

    class Config:
        json_schema_extra = {
            "example": {
                "success": True,
                "message": "Open sequence started",
                "state": "OPENING_SLIDE",
                "timestamp": "2026-06-05T12:00:00+00:00",
            }
        }


class StatusResponse(BaseModel):
    """Detailed pad status response."""
    state: str = Field(..., description="Current pad state")
    is_paused: bool = Field(..., description="Whether operation is currently paused")
    active_motor: Optional[str] = Field(None, description="Currently active motor ID")
    error_message: Optional[str] = Field(None, description="Error details if in ERROR state")
    last_command: Optional[str] = Field(None, description="Last command received")
    last_command_time: Optional[str] = Field(None, description="Timestamp of last command")
    uptime_seconds: float = Field(..., description="Service uptime in seconds")
    simulation_mode: bool = Field(..., description="Whether running in GPIO simulation mode")
    timestamp: str = Field(..., description="ISO 8601 UTC timestamp of this response")


class HealthResponse(BaseModel):
    """Health check response."""
    status: str = Field(..., description="Service health status")
    service: str = Field(default="drone-pad-api", description="Service name")
    version: str = Field(default="1.0.0", description="Service version")
    simulation_mode: bool = Field(..., description="GPIO simulation mode")
    pad_state: str = Field(..., description="Current pad state")
    uptime_seconds: float = Field(..., description="Service uptime in seconds")
    timestamp: str = Field(..., description="ISO 8601 UTC timestamp")


class EventLogEntry(BaseModel):
    """Single event log entry."""
    timestamp: str
    event: str
    from_state: str
    to_state: str


class EventLogResponse(BaseModel):
    """Event log response."""
    count: int = Field(..., description="Number of events returned")
    events: List[EventLogEntry] = Field(..., description="Event log entries")


class SimTriggerRequest(BaseModel):
    """Request to trigger a simulated limit switch (simulation mode only)."""
    pin: int = Field(..., description="GPIO pin number of the limit switch")
    triggered: bool = Field(True, description="True to trigger, False to release")


class DockStatusResponse(BaseModel):
    """Status of the BLE connection to the dock-lock (ESP32) controller."""
    available: bool = Field(..., description="Whether the docking controller could be initialized (bleak installed)")
    connected: bool = Field(..., description="Whether the BLE link is currently up")
    status: str = Field(..., description="Last known status reported by the dock controller")
    device_mac: Optional[str] = Field(None, description="BLE MAC address of the dock controller")
    timestamp: str = Field(..., description="ISO 8601 UTC timestamp of this response")


class DockCommandResponse(BaseModel):
    """Standard response for dock/undock/reset commands."""
    success: bool = Field(..., description="Whether the command was accepted")
    message: str = Field(..., description="Human-readable result message")
    status: str = Field(..., description="Dock controller status at the time of response")
    timestamp: str = Field(..., description="ISO 8601 UTC timestamp")


class DebugJogRequest(BaseModel):
    """Request to jog a single motor toward one of its limit switches."""
    direction: Literal["FORWARD", "REVERSE"] = Field(
        ...,
        description=(
            "OPENING: FORWARD=slide open, REVERSE=slide close. "
            "LIFT: FORWARD=lift up, REVERSE=lift down."
        ),
    )


class DebugCommandResponse(BaseModel):
    """Standard response for debug jog/stop commands."""
    success: bool = Field(..., description="Whether the command was accepted")
    message: str = Field(..., description="Human-readable result message")
    timestamp: str = Field(..., description="ISO 8601 UTC timestamp")


class DebugJogStatusResponse(BaseModel):
    """Status of the current/last single-axis debug jog."""
    running: bool = Field(..., description="Whether a jog is currently in progress")
    motor_id: Optional[str] = Field(None, description="Motor being jogged (OPENING or LIFT)")
    direction: Optional[str] = Field(None, description="FORWARD or REVERSE")
    started_at: Optional[str] = Field(None, description="ISO 8601 UTC timestamp the jog started")
    elapsed_seconds: Optional[float] = Field(None, description="Seconds elapsed, while running")
    last_result: Optional[str] = Field(None, description="success | timeout | cancelled | error")
    last_message: Optional[str] = Field(None, description="Detail for the last result")
    timestamp: str = Field(..., description="ISO 8601 UTC timestamp of this response")


class LimitSwitchStatusResponse(BaseModel):
    """Raw debounced state of all four limit switches — no motor movement."""
    open_limit: bool = Field(..., description="OPENING motor's slide-open limit")
    close_limit: bool = Field(..., description="OPENING motor's slide-close limit")
    lift_upper: bool = Field(..., description="LIFT motor's up limit")
    lift_lower: bool = Field(..., description="LIFT motor's down limit")
    timestamp: str = Field(..., description="ISO 8601 UTC timestamp")


# ---------------------------------------------------------------------------
# Application Globals
# ---------------------------------------------------------------------------

settings: Optional[Settings] = None
gpio: Optional[GPIOManager] = None
motor: Optional[MotorController] = None
pad: Optional[PadStateMachine] = None
docking: Optional[DockingController] = None
debug_motor: Optional[DebugMotorController] = None

logger = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# Lifespan Handler
# ---------------------------------------------------------------------------

@asynccontextmanager
async def lifespan(app: FastAPI):
    """Manage application startup and shutdown lifecycle."""
    global settings, gpio, motor, pad, docking, debug_motor

    # --- Startup ---
    settings = load_settings()
    setup_logging(settings.log_level)

    logger.info("=" * 60)
    logger.info("Drone Pad API Service starting up")
    logger.info("=" * 60)

    gpio = GPIOManager(settings)
    gpio.initialize()

    motor = MotorController(gpio, settings)
    debug_motor = DebugMotorController(gpio, motor, settings)

    # Docking (BLE) is a separate subsystem — its absence shouldn't stop
    # the pad's motors from working standalone, so failures here are
    # logged, not raised. PadStateMachine handles docking=None safely by
    # refusing open()/close() with a clear message (see _docking_precheck).
    try:
        docking = DockingController.from_settings(settings)
        await docking.start()
        logger.info(
            "Docking controller starting — connecting to %s in the background",
            settings.docking_config.device_mac,
        )
    except ImportError as exc:
        logger.warning("Docking controller unavailable: %s", exc)
        docking = None

    # PadStateMachine bridges into docking's async calls via
    # run_coroutine_threadsafe(), which needs the actual running loop.
    loop = asyncio.get_running_loop()
    pad = PadStateMachine(gpio, motor, settings, docking=docking, loop=loop)

    logger.info(
        "Service ready — listening on %s:%d (simulation=%s)",
        settings.host, settings.port, gpio.is_simulation,
    )

    yield  # Application runs here

    # --- Shutdown ---
    logger.info("Shutting down — cleaning up resources")

    if pad is not None:
        try:
            await _run_blocking(pad.emergency_stop)
        except Exception:
            pass

    if docking is not None:
        try:
            await docking.stop()
        except Exception:
            pass

    if gpio is not None:
        gpio.cleanup()

    logger.info("Shutdown complete")


# ---------------------------------------------------------------------------
# FastAPI Application
# ---------------------------------------------------------------------------

app = FastAPI(
    title="Drone Pad API Service",
    description=(
        "REST API for controlling a robotic drone pad with dual stepper motors "
        "(NEMA 23 opening motor + NEMA 17 lift motor) and limit switch safety."
    ),
    version="1.0.0",
    lifespan=lifespan,
)

# CORS — allow all origins for embedded/IoT use cases
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)


# ---------------------------------------------------------------------------
# Global Exception Handler
# ---------------------------------------------------------------------------

@app.exception_handler(Exception)
async def global_exception_handler(request: Request, exc: Exception):
    logger.exception("Unhandled exception on %s %s", request.method, request.url.path)
    return JSONResponse(
        status_code=500,
        content={
            "success": False,
            "message": f"Internal server error: {type(exc).__name__}",
            "state": pad.get_status().state.value if pad else "UNKNOWN",
            "timestamp": datetime.now(timezone.utc).isoformat(),
        },
    )


# ---------------------------------------------------------------------------
# Helper
# ---------------------------------------------------------------------------

def _now_iso() -> str:
    return datetime.now(timezone.utc).isoformat()


async def _run_blocking(fn, *args):
    """
    Run a synchronous PadStateMachine/DebugMotorController call off the
    event loop.

    Several of these methods (emergency_stop(), close()'s interrupt path,
    debug jog stop()) call thread.join()/future.result() internally —
    genuinely blocking calls. Calling them directly from an `async def`
    endpoint blocks the entire event loop for that duration, which is
    also where DockingController's BLE coroutines run — including any
    docking command the blocked method itself schedules via
    run_coroutine_threadsafe(). That's a real deadlock: the scheduled
    coroutine can't run until the blocking call returns, but the blocking
    call is waiting on that same coroutine's result. Offloading to the
    default thread pool keeps the loop free the whole time.
    """
    loop = asyncio.get_running_loop()
    return await loop.run_in_executor(None, fn, *args)


def _command_response(result: dict) -> CommandResponse:
    """Build a CommandResponse from a state machine result dict."""
    status = pad.get_status()
    return CommandResponse(
        success=result["success"],
        message=result["message"],
        state=status.state.value,
        timestamp=_now_iso(),
    )


# ---------------------------------------------------------------------------
# Endpoints
# ---------------------------------------------------------------------------

@app.get("/api/health", response_model=HealthResponse, tags=["monitoring"])
async def health_check():
    """Service health check endpoint."""
    status = pad.get_status()
    return HealthResponse(
        status="healthy",
        simulation_mode=gpio.is_simulation,
        pad_state=status.state.value,
        uptime_seconds=status.uptime_seconds,
        timestamp=_now_iso(),
    )


@app.get("/api/status", response_model=StatusResponse, tags=["monitoring"])
async def get_status():
    """Get detailed pad status including motor activity and error info."""
    status = pad.get_status()
    return StatusResponse(
        state=status.state.value,
        is_paused=status.is_paused,
        active_motor=status.active_motor,
        error_message=status.error_message,
        last_command=status.last_command,
        last_command_time=status.last_command_time,
        uptime_seconds=status.uptime_seconds,
        simulation_mode=gpio.is_simulation,
        timestamp=_now_iso(),
    )


@app.post("/api/open", response_model=CommandResponse, tags=["commands"])
async def open_pad():
    """
    Start the pad open sequence.

    Sequence: Slide open (NEMA 23) → Lift up (NEMA 17) → UNDOCK over BLE.
    The pad only reaches OPEN once the docking system confirms
    UNDOCKING_COMPLETE — UNDOCK is never sent unless lift_upper is
    confirmed triggered.

    Self-healing: this can be called from any idle state, including
    ERROR (e.g. after a power cycle whose limit switches hadn't settled
    yet) — every stage live-verifies the pad's actual position via its
    limit switches rather than trusting remembered state, and safely
    no-ops any stage that's already done. No manual reset required.

    Returns 200 with success=false only if the pad is genuinely
    mid-close (a real close sequence is in progress) or a debug axis jog
    (POST /api/debug/motor/{motor_id}/jog) is running.
    """
    if debug_motor is not None and debug_motor.get_status().running:
        return CommandResponse(
            success=False,
            message="A debug motor jog is in progress — stop it first (POST /api/debug/motor/stop)",
            state=pad.get_status().state.value,
            timestamp=_now_iso(),
        )
    result = await _run_blocking(pad.open)
    return _command_response(result)


@app.post("/api/close", response_model=CommandResponse, tags=["commands"])
async def close_pad():
    """
    Start the pad close sequence, or interrupt an in-progress open.

    Sequence: DOCK over BLE (only if currently lifted) → Lift down
    (NEMA 17) → Slide close (NEMA 23). DOCK is skipped entirely if the
    pad isn't confirmed lifted right now (already closed, or an open
    interrupted before the lift finished) — there's nothing to dock in
    that case. When DOCK does run, it must complete (DOCKING_COMPLETE)
    before either motor moves.

    Self-healing: this can be called from any idle state, including
    ERROR — every stage live-verifies the pad's actual position via its
    limit switches, so it correctly recognizes "already closed" and
    resolves in under a second, or drives whatever's left to a known
    limit. No manual reset required.

    Interrupting mid-UNDOCK/DOCK is rejected — wait for it to finish or
    issue emergency-stop. Returns success=false only if the pad is
    genuinely mid-close already, or a debug axis jog is running.
    """
    if debug_motor is not None and debug_motor.get_status().running:
        return CommandResponse(
            success=False,
            message="A debug motor jog is in progress — stop it first (POST /api/debug/motor/stop)",
            state=pad.get_status().state.value,
            timestamp=_now_iso(),
        )
    result = await _run_blocking(pad.close)
    return _command_response(result)


@app.post("/api/toggle", response_model=CommandResponse, tags=["commands"])
async def toggle_pause():
    """
    Pause or resume the currently running operation.

    If an open or close sequence is running:
      - First toggle → pauses the motor
      - Second toggle → resumes from the same position

    Returns success=false if no operation is in progress.
    """
    result = await _run_blocking(pad.toggle)
    return _command_response(result)


@app.post("/api/emergency-stop", response_model=CommandResponse, tags=["safety"])
async def emergency_stop():
    """
    Immediately stop all motors and enter ERROR state.

    All motor drivers are disabled at the hardware level. The pad
    must be reset (POST /api/reset) before normal commands will
    be accepted again. Also cancels any in-progress debug axis jog.
    """
    if debug_motor is not None:
        await _run_blocking(debug_motor.stop)
    result = await _run_blocking(pad.emergency_stop)
    return _command_response(result)


@app.post("/api/reset", response_model=CommandResponse, tags=["safety"])
async def reset_pad():
    """
    Reset the pad from ERROR state back to CLOSED.

    Only available when the pad is in ERROR state and no sequence
    thread is running.
    """
    result = await _run_blocking(pad.reset_from_error)
    return _command_response(result)


@app.get("/api/events", response_model=EventLogResponse, tags=["monitoring"])
async def get_events():
    """
    Get the recent event log.

    Returns up to the last 100 state transitions, commands, and errors.
    """
    events = pad.get_event_log()
    return EventLogResponse(
        count=len(events),
        events=[
            EventLogEntry(
                timestamp=e.timestamp,
                event=e.event,
                from_state=e.from_state,
                to_state=e.to_state,
            )
            for e in events
        ],
    )


# ---------------------------------------------------------------------------
# Simulation-Only Endpoints
# ---------------------------------------------------------------------------

@app.post("/api/sim/trigger", tags=["simulation"])
async def sim_trigger_switch(req: SimTriggerRequest):
    """
    Trigger a simulated limit switch (simulation mode only).

    Use this during development to test state transitions without
    physical hardware.
    """
    if not gpio.is_simulation:
        raise HTTPException(
            status_code=403,
            detail="Simulation endpoints are only available in simulation mode",
        )

    gpio.sim_trigger_switch(req.pin, req.triggered)
    status = pad.get_status()

    return {
        "success": True,
        "message": f"GPIO {req.pin} {'triggered' if req.triggered else 'released'}",
        "state": status.state.value,
        "timestamp": _now_iso(),
    }


# ---------------------------------------------------------------------------
# Docking Endpoints (BLE, ESP32 dock-lock controller)
# ---------------------------------------------------------------------------

def _require_docking() -> DockingController:
    if docking is None:
        raise HTTPException(
            status_code=503,
            detail="Docking controller is not available (bleak not installed on this host)",
        )
    return docking


def _dock_command_response(success: bool, message: str) -> DockCommandResponse:
    return DockCommandResponse(
        success=success,
        message=message,
        status=docking.last_status.value,
        timestamp=_now_iso(),
    )


@app.get("/api/dock/status", response_model=DockStatusResponse, tags=["docking"])
async def get_dock_status():
    """
    Get the current state of the BLE link and the dock-lock controller.

    `status` reflects the last value reported by the ESP32 over BLE
    notifications, including in-progress states like DOCKING_M1 or
    UNDOCKING_PROP_OPEN — poll this while a dock/undock command is running.
    """
    return DockStatusResponse(
        available=docking is not None,
        connected=docking.is_connected if docking is not None else False,
        status=docking.last_status.value if docking is not None else "UNAVAILABLE",
        device_mac=docking.device_mac if docking is not None else None,
        timestamp=_now_iso(),
    )


@app.post("/api/dock/dock", response_model=DockCommandResponse, tags=["docking"])
async def dock_command():
    """
    Start the docking sequence on the ESP32 dock-lock controller.

    Runs in the background — this returns immediately once the command is
    sent; poll GET /api/dock/status to watch progress and see the final
    DOCKING_COMPLETE (or ERROR) status.
    """
    controller = _require_docking()

    if not controller.is_connected:
        return _dock_command_response(False, "Not connected to dock controller")
    if controller.is_busy:
        return _dock_command_response(False, "A dock/undock command is already in progress")

    asyncio.create_task(_run_dock_command(controller.dock, "DOCK"))
    return _dock_command_response(True, "Dock sequence started")


@app.post("/api/dock/undock", response_model=DockCommandResponse, tags=["docking"])
async def undock_command():
    """
    Start the undocking sequence on the ESP32 dock-lock controller.

    Runs in the background — this returns immediately once the command is
    sent; poll GET /api/dock/status to watch progress and see the final
    UNDOCKING_COMPLETE (or ERROR) status.
    """
    controller = _require_docking()

    if not controller.is_connected:
        return _dock_command_response(False, "Not connected to dock controller")
    if controller.is_busy:
        return _dock_command_response(False, "A dock/undock command is already in progress")

    asyncio.create_task(_run_dock_command(controller.undock, "UNDOCK"))
    return _dock_command_response(True, "Undock sequence started")


@app.post("/api/dock/reset", response_model=DockCommandResponse, tags=["docking"])
async def dock_reset_command():
    """Send RESET to the dock-lock controller — stops its motors and returns it to IDLE."""
    controller = _require_docking()

    if not controller.is_connected:
        return _dock_command_response(False, "Not connected to dock controller")

    try:
        await controller.reset()
        return _dock_command_response(True, "RESET sent")
    except DockingError as exc:
        return _dock_command_response(False, str(exc))


async def _run_dock_command(command_coro, label: str) -> None:
    """Await a dock()/undock() call started from a request handler and log the outcome."""
    try:
        result = await command_coro()
        logger.info("%s finished: %s", label, result.value)
    except DockingError as exc:
        logger.error("%s failed: %s", label, exc)


# ---------------------------------------------------------------------------
# Debug Endpoints — isolated single-axis motor testing
# ---------------------------------------------------------------------------

@app.get("/api/debug/limits", response_model=LimitSwitchStatusResponse, tags=["debug"])
async def get_limit_switches():
    """
    Read all four limit switches directly — no motor movement.

    Trigger them by hand to confirm wiring/polarity before jogging a motor.
    """
    return LimitSwitchStatusResponse(
        open_limit=gpio.is_open_limit_triggered(),
        close_limit=gpio.is_close_limit_triggered(),
        lift_upper=gpio.is_lift_upper_triggered(),
        lift_lower=gpio.is_lift_lower_triggered(),
        timestamp=_now_iso(),
    )


@app.post("/api/debug/motor/{motor_id}/jog", response_model=DebugCommandResponse, tags=["debug"])
async def jog_motor(motor_id: MotorId, req: DebugJogRequest):
    """
    Jog a single motor toward one limit switch, independent of the full
    pad sequence. Runs in the background — poll GET /api/debug/motor/status
    to watch it and see the final result (success/timeout/cancelled/error).

    Allowed when the pad is CLOSED, OPEN, or in ERROR — ERROR is exactly
    when this is needed most, to manually jog to a known limit before
    POST /api/reset can verify the pad's real position. Rejected during
    an active sequence stage (OPENING_SLIDE, UNDOCKING, etc.), where a
    real motor move or BLE operation could genuinely be in flight, or if
    another debug jog is already running.
    """
    pad_state = pad.get_status().state
    if pad_state not in (PadState.CLOSED, PadState.OPEN, PadState.ERROR):
        return DebugCommandResponse(
            success=False,
            message=f"Pad must be idle or in ERROR to jog a debug axis (current state: {pad_state.value})",
            timestamp=_now_iso(),
        )

    direction = Direction.FORWARD if req.direction == "FORWARD" else Direction.REVERSE
    result = await _run_blocking(debug_motor.jog, motor_id, direction)
    return DebugCommandResponse(
        success=result["success"], message=result["message"], timestamp=_now_iso(),
    )


@app.post("/api/debug/motor/stop", response_model=DebugCommandResponse, tags=["debug"])
async def stop_jog():
    """Cancel the current debug jog, if any, and disable the motor driver."""
    result = await _run_blocking(debug_motor.stop)
    return DebugCommandResponse(
        success=result["success"], message=result["message"], timestamp=_now_iso(),
    )


@app.get("/api/debug/motor/status", response_model=DebugJogStatusResponse, tags=["debug"])
async def get_jog_status():
    """Get the status of the current/last debug jog."""
    status = debug_motor.get_status()
    return DebugJogStatusResponse(
        running=status.running,
        motor_id=status.motor_id,
        direction=status.direction,
        started_at=status.started_at,
        elapsed_seconds=status.elapsed_seconds,
        last_result=status.last_result,
        last_message=status.last_message,
        timestamp=_now_iso(),
    )


# ---------------------------------------------------------------------------
# Entry Point
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    import uvicorn

    _settings = load_settings()
    setup_logging(_settings.log_level)

    uvicorn.run(
        "main:app",
        host=_settings.host,
        port=_settings.port,
        log_level=_settings.log_level.lower(),
        access_log=True,
    )
