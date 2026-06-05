"""
Drone Pad API Service — FastAPI Application

Production-ready REST API for controlling the drone pad. Provides
endpoints for open/close/toggle commands, status monitoring, emergency
stop, and event log retrieval.
"""

from __future__ import annotations

import logging
import sys
import os
from contextlib import asynccontextmanager
from datetime import datetime, timezone
from typing import Optional, List

from fastapi import FastAPI, HTTPException, Request
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import JSONResponse
from pydantic import BaseModel, Field

from config import PadState, load_settings, Settings
from gpio_manager import GPIOManager
from motor_controller import MotorController
from pad_state_machine import PadStateMachine, PadStatus, EventEntry

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


# ---------------------------------------------------------------------------
# Application Globals
# ---------------------------------------------------------------------------

settings: Optional[Settings] = None
gpio: Optional[GPIOManager] = None
motor: Optional[MotorController] = None
pad: Optional[PadStateMachine] = None

logger = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# Lifespan Handler
# ---------------------------------------------------------------------------

@asynccontextmanager
async def lifespan(app: FastAPI):
    """Manage application startup and shutdown lifecycle."""
    global settings, gpio, motor, pad

    # --- Startup ---
    settings = load_settings()
    setup_logging(settings.log_level)

    logger.info("=" * 60)
    logger.info("Drone Pad API Service starting up")
    logger.info("=" * 60)

    gpio = GPIOManager(settings)
    gpio.initialize()

    motor = MotorController(gpio, settings)
    pad = PadStateMachine(gpio, motor, settings)

    logger.info(
        "Service ready — listening on %s:%d (simulation=%s)",
        settings.host, settings.port, gpio.is_simulation,
    )

    yield  # Application runs here

    # --- Shutdown ---
    logger.info("Shutting down — cleaning up resources")

    if pad is not None:
        try:
            pad.emergency_stop()
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

    Sequence: Slide open (NEMA 23) → Lift up (NEMA 17)

    Returns 200 with success=false if the command cannot be executed
    in the current state (e.g., pad is closing or in error).
    """
    result = pad.open()
    return _command_response(result)


@app.post("/api/close", response_model=CommandResponse, tags=["commands"])
async def close_pad():
    """
    Start the pad close sequence, or interrupt an in-progress open.

    Sequence: Lift down (NEMA 17) → Slide close (NEMA 23)

    If the pad is currently opening, this will interrupt the open
    sequence and begin closing from the current position.
    """
    result = pad.close()
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
    result = pad.toggle()
    return _command_response(result)


@app.post("/api/emergency-stop", response_model=CommandResponse, tags=["safety"])
async def emergency_stop():
    """
    Immediately stop all motors and enter ERROR state.

    All motor drivers are disabled at the hardware level. The pad
    must be reset (POST /api/reset) before normal commands will
    be accepted again.
    """
    result = pad.emergency_stop()
    return _command_response(result)


@app.post("/api/reset", response_model=CommandResponse, tags=["safety"])
async def reset_pad():
    """
    Reset the pad from ERROR state back to CLOSED.

    Only available when the pad is in ERROR state and no sequence
    thread is running.
    """
    result = pad.reset_from_error()
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
