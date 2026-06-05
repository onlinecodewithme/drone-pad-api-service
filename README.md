# Drone Pad API Service

REST API service for controlling a robotic drone pad with dual stepper motors on Raspberry Pi.

## Hardware Overview

The drone pad uses two stepper motors driven by **TB6600 stepper drivers** powered at **24V**, and four limit switches:

| Component | Type | Driver | Function |
|---|---|---|---|
| Opening Motor | NEMA 23 | TB6600 (24V) | Slides the pad open/closed |
| Lift Motor | NEMA 17 | TB6600 (24V) | Raises/lowers the pad |
| Open Limit Switch | NC/NO | — | Detects fully open position |
| Close Limit Switch | NC/NO | — | Detects fully closed position |
| Lift Upper Switch | NC/NO | — | Detects fully raised position |
| Lift Lower Switch | NC/NO | — | Detects fully lowered position |

> **TB6600 Driver Notes**: Enable is active-LOW (ENA- to GND, ENA+ from GPIO). Signal pins (PUL+, DIR+, ENA+) connect to RPi GPIO; negative pins (PUL-, DIR-, ENA-) connect to RPi GND.

## GPIO Pin Assignments (BCM)

| Component | Signal | GPIO Pin |
|---|---|---|
| Opening Motor (NEMA 23) | STEP | 14 |
| | DIR | 16 |
| | ENABLE | 12 |
| Lift Motor (NEMA 17) | STEP | 26 |
| | DIR | 13 |
| | ENABLE | 11 |
| Open Limit Switch | INPUT | 4 |
| Close Limit Switch | INPUT | 10 |
| Lift Upper Limit Switch | INPUT | 9 |
| Lift Lower Limit Switch | INPUT | 3 |

> All limit switch inputs use internal pull-up resistors. Switches should connect GPIO to GND when triggered (active LOW).

## Operation Sequences

### Open Sequence
```
CLOSED → [NEMA 23 forward → open limit] → [NEMA 17 forward → lift upper limit] → OPEN
```

### Close Sequence
```
OPEN → [NEMA 17 reverse → lift lower limit] → [NEMA 23 reverse → close limit] → CLOSED
```

### Interrupt (Close during Open)
If a close command is received while opening, the current motor stops immediately and the close sequence begins from the current stage.

### Toggle (Pause/Resume)
The toggle command pauses or resumes any in-progress operation. Motors hold position while paused.

## State Machine

```
CLOSED ──open──► OPENING_SLIDE ──limit──► OPENING_LIFT ──limit──► OPEN
  ▲                    │                       │                    │
  │                    │close                   │close               │close
  │                    ▼                       ▼                    ▼
  └──limit── CLOSING_SLIDE ◄──limit── CLOSING_LIFT ◄───────────────┘
```

All states except `CLOSED` and `OPEN` can be `PAUSED` via the toggle command.

## Installation

```bash
# Clone or copy to Raspberry Pi
cd /home/x4mc1/Documents/drone-pad-api-service

# Create virtual environment and install dependencies
python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt

# Copy environment config
cp .env.example .env
# Edit .env as needed
```

## Running

### Direct
```bash
source venv/bin/activate
python3 main.py
```

### As a systemd Service
```bash
# Copy the service file
sudo cp drone-pad.service /etc/systemd/system/

# Reload systemd
sudo systemctl daemon-reload

# Enable and start
sudo systemctl enable drone-pad.service
sudo systemctl start drone-pad.service

# Check status
sudo systemctl status drone-pad.service

# View logs
sudo journalctl -u drone-pad -f
```

## API Reference

Base URL: `http://<raspberry-pi-ip>:8000`

Interactive API docs available at: `http://<raspberry-pi-ip>:8000/docs`

### Commands

| Method | Endpoint | Description |
|---|---|---|
| `POST` | `/api/open` | Start the open sequence |
| `POST` | `/api/close` | Start close sequence (or interrupt open) |
| `POST` | `/api/toggle` | Pause/resume current operation |
| `POST` | `/api/emergency-stop` | Immediately stop all motors |
| `POST` | `/api/reset` | Reset from ERROR state to CLOSED |

### Monitoring

| Method | Endpoint | Description |
|---|---|---|
| `GET` | `/api/health` | Service health check |
| `GET` | `/api/status` | Detailed pad status |
| `GET` | `/api/events` | Recent event log (last 100) |

### Simulation (dev only)

| Method | Endpoint | Description |
|---|---|---|
| `POST` | `/api/sim/trigger` | Trigger a simulated limit switch |

### Example Usage

```bash
# Open the pad
curl -X POST http://localhost:8000/api/open

# Check status
curl http://localhost:8000/api/status

# Pause the current operation
curl -X POST http://localhost:8000/api/toggle

# Resume the operation
curl -X POST http://localhost:8000/api/toggle

# Close the pad (or interrupt opening)
curl -X POST http://localhost:8000/api/close

# Emergency stop
curl -X POST http://localhost:8000/api/emergency-stop

# Reset after emergency stop
curl -X POST http://localhost:8000/api/reset
```

### Response Format

All command endpoints return:
```json
{
  "success": true,
  "message": "Open sequence started",
  "state": "OPENING_SLIDE",
  "timestamp": "2026-06-05T12:00:00+00:00"
}
```

## Configuration

All settings can be overridden via environment variables. See `.env.example` for the complete list.

Key parameters:

| Variable | Default | Description |
|---|---|---|
| `DRONE_PAD_PORT` | `8000` | API server port |
| `DRONE_PAD_SIMULATION` | `false` | Run without GPIO hardware |
| `DRONE_PAD_OPENING_STEP_DELAY_US` | `500` | NEMA 23 step delay (μs) |
| `DRONE_PAD_LIFT_STEP_DELAY_US` | `800` | NEMA 17 step delay (μs) |
| `DRONE_PAD_OPENING_TIMEOUT_S` | `60.0` | NEMA 23 operation timeout |
| `DRONE_PAD_LIFT_TIMEOUT_S` | `30.0` | NEMA 17 operation timeout |
| `DRONE_PAD_SWITCH_DEBOUNCE_MS` | `50` | Limit switch debounce time |

## Wiring Diagram

```
                            TB6600 #1 — NEMA 23 (Opening Motor) [24V]
Raspberry Pi (BCM)          ─────────────────────────────────────────
GPIO 14 ────────────────►   PUL+  (Step pulse)
GPIO 16 ────────────────►   DIR+  (Direction)
GPIO 12 ────────────────►   ENA+  (Enable, active LOW)
GND     ────────────────►   PUL-, DIR-, ENA-
24V PSU ────────────────►   VCC / Motor power

                            TB6600 #2 — NEMA 17 (Lift Motor) [24V]
Raspberry Pi (BCM)          ─────────────────────────────────────────
GPIO 26 ────────────────►   PUL+  (Step pulse)
GPIO 13 ────────────────►   DIR+  (Direction)
GPIO 11 ────────────────►   ENA+  (Enable, active LOW)
GND     ────────────────►   PUL-, DIR-, ENA-
24V PSU ────────────────►   VCC / Motor power

Raspberry Pi (BCM)          Limit Switches
─────────────────           ──────────────
GPIO 4  ◄──────────── NO ── Open Limit Switch ── GND
GPIO 10 ◄──────────── NO ── Close Limit Switch ── GND
GPIO 9  ◄──────────── NO ── Lift Upper Switch ── GND
GPIO 3  ◄──────────── NO ── Lift Lower Switch ── GND
```

## Development

### Simulation Mode

On a non-Raspberry Pi system, the service automatically enters simulation mode. You can also force it:

```bash
DRONE_PAD_SIMULATION=true python3 main.py
```

In simulation mode, use the `/api/sim/trigger` endpoint to simulate limit switch events:

```bash
# Simulate open limit switch triggering (GPIO 4)
curl -X POST http://localhost:8000/api/sim/trigger \
  -H "Content-Type: application/json" \
  -d '{"pin": 4, "triggered": true}'
```

## License

Internal use only.
