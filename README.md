# Drone Pad API Service

REST API service for controlling a robotic drone pad with dual stepper motors on Raspberry Pi, integrated with a separate ESP32-based dock-lock controller over BLE.

## Hardware Overview

| Component | Type | Driver | Function |
|---|---|---|---|
| Opening Motor | NEMA 23 | TB6600 (24V) | Slides the pad open/closed |
| Lift Motor | NEMA 17 | TB6600 (24V) | Raises/lowers the pad |
| Open Limit Switch | NC/NO | — | Detects fully open (slide) position |
| Close Limit Switch | NC/NO | — | Detects fully closed (slide) position |
| Lift Upper Switch | NC/NO | — | Detects fully raised position |
| Lift Lower Switch | NC/NO | — | Detects fully lowered position |
| Dock-Lock Controller | ESP32 + 3 steppers | — | Docks/undocks the drone once the pad is lifted, over BLE |

> **TB6600 Driver Notes**: Enable is active-LOW (ENA- to GND, ENA+ from GPIO). Signal pins (PUL+, DIR+, ENA+) connect to RPi GPIO; negative pins (PUL-, DIR-, ENA-) connect to RPi GND.

The dock-lock controller is a physically separate device (firmware at `reference/drone_dock_locking_system/`, own repo) — the Pi talks to it purely over Bluetooth Low Energy, no GPIO wiring between the two boards.

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

> All limit switch inputs use internal pull-up resistors. Switch polarity is configurable via `DRONE_PAD_SWITCH_ACTIVE_LOW`.

## Docking System (BLE)

The ESP32 dock-lock controller docks/undocks the drone itself — separate motors, separate firmware, separate power. The Pi connects to it as a BLE GATT client (`docking_controller.py`) and:

- Auto-connects on service startup and auto-reconnects on any BLE drop (the ESP32 re-advertises on disconnect), with a bounded connect timeout enforced on the Pi side so a wedged BLE/D-Bus call can't silently freeze all future reconnect attempts (see `docking_controller.py`'s `connect()`)
- Sends `DOCK` / `UNDOCK` / `RESET` commands and tracks progress via BLE notifications (`UNKNOWN`, `UNDOCKING_M1/M2`, `DOCKING_M2/M1/PROP_OPEN/PROP_CLOSE`, the stable resting states `DOCKED`/`UNDOCKED`, `ERROR`)
- Is fully integrated into the pad's own open/close sequence (see State Machine below) — you don't call the dock endpoints directly in normal operation, `/api/open` and `/api/close` do it for you at the right moment

`/api/dock/*` endpoints exist for manual testing/debugging the BLE link independent of the pad sequence (see API Reference).

## Operation Sequences

### Open Sequence
```
CLOSED → [NEMA 23 forward → open limit] → [NEMA 17 forward → lift upper limit]
       → [UNDOCK over BLE → UNDOCKED] → OPEN
```
UNDOCK is only ever sent once `lift_upper` is independently re-confirmed triggered — not just because the previous stage returned.

### Close Sequence
```
OPEN → [DOCK over BLE → DOCKED, only if currently lifted]
     → [NEMA 17 reverse → lift lower limit] → [NEMA 23 reverse → close limit] → CLOSED
```
DOCK is skipped if the pad isn't confirmed lifted right now (e.g. already closed, or an open interrupted before the lift finished) — there's nothing to dock in that case. When DOCK does run, it must complete before either motor moves; if it fails or times out, the pad stays open and reports `ERROR` rather than lifting/closing an undocked drone.

### Interrupt (Close during Open)
A close command received during `OPENING_SLIDE`/`OPENING_LIFT` cancels the current motor move and closes from wherever the pad physically is. Interrupting `UNDOCKING`/`DOCKING` is rejected — those are live BLE operations with the dock mechanism, not safely abortable mid-flight. Use emergency-stop instead.

### Toggle (Pause/Resume)
Pauses or resumes an in-progress *motor* move. Rejected during `UNDOCKING`/`DOCKING` — those are BLE waits, not something pausing on the Pi side can affect.

## State Machine

```
CLOSED ──open──► OPENING_SLIDE ──limit──► OPENING_LIFT ──limit──► UNDOCKING ──BLE──► OPEN
  ▲                                                                                    │
  │                                                                                    │close
  │                                                                                    ▼
  └──limit── CLOSING_SLIDE ◄──limit── CLOSING_LIFT ◄──BLE── DOCKING ◄──────────────────┘
```

`ERROR` is reachable from any state (motor timeout, a failed hard-safety-gate, a failed/timed-out DOCK or UNDOCK, or emergency-stop) and disables both pad motors immediately.

## Safety Architecture

This system controls physical mechanical hardware with a drone potentially attached, so several layers of defense are built in:

1. **Hard safety gates, not just sequencing.** Before arming the lift motor, the code independently re-reads `open_limit` rather than trusting that the previous stage returned without an exception. Before sending `UNDOCK`/`DOCK`, it independently re-reads `lift_upper`. These checks use plain `if`/`return`, never `assert` — `assert` is silently stripped when Python runs with `-O`, which would be a real, easy-to-miss hole in a systemd deployment.
2. **Multi-sample confirmation.** A single instantaneous limit-switch read isn't trusted for these gates — each check requires the switch to read consistently across 3 samples spaced ~150ms apart before it's acted on. This exists because this hardware has a known wiring-reliability history (see `git log` on `pad_state_machine.py`/`gpio_manager.py`) where switches could briefly, simultaneously misreport.
3. **Self-healing, never a remembered-state guess.** `open()`/`close()` never trust `self._state` to answer "are we already there" — every call live-verifies the pad's actual position via its limit switches and safely no-ops any stage already satisfied. This means the pad recovers from `ERROR` (including a fresh power-cycle before its switches have settled) automatically the moment you call the command you actually want — no manual reset dance required. `reset_from_error()`/`POST /api/reset` still exist for cases where you want to *tell* the software a position without moving any motors (e.g. after manually verifying by hand).
4. **Emergency-stop reaches both systems.** `POST /api/emergency-stop` disables both pad motors at the hardware level *and* sends `RESET` to the docking controller over BLE, best-effort — "stop everything" means both boards, not just the Pi's own two motors.
5. **Blocking calls never run on the event loop.** `emergency_stop()`, `close()`'s interrupt path, and the debug jog stop all contain `thread.join()`/`future.result()` — genuinely blocking calls. These are offloaded to a thread pool executor (`_run_blocking()` in `main.py`) rather than called directly from an `async def` handler, which would otherwise freeze the entire event loop — including the very BLE coroutines a blocked call might itself be waiting on.
6. **BLE connection failures can't silently wedge the reconnect loop.** `DockingController.connect()`'s underlying BlueZ/D-Bus call doesn't reliably honor its own `timeout=`, and a stuck call there previously froze every future reconnect attempt indefinitely, with no error and no log — a real incident where the pad opened and lifted fine (no BLE involved) but then failed `UNDOCK` because the link had been silently dead for hours. `connect()` now enforces the timeout itself via `asyncio.wait_for()`. Separately, `_docking_precheck()` gives an in-flight reconnect a bounded grace window (`_DOCKING_CONNECT_WAIT_S`) before failing a DOCK/UNDOCK outright, since the background reconnect loop's own backoff can otherwise cause a command to fail just seconds before a retry would have landed.

## Installation

```bash
# Clone or copy to Raspberry Pi
cd /home/x4mc1/Documents/drone-pad-api-service

# Create virtual environment and install dependencies
python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt   # includes bleak, for the BLE docking link

# Copy environment config
cp .env.example .env
# Edit .env as needed — in particular DRONE_PAD_DOCK_MAC (see docking_controller.py's
# CLI: `python docking_controller.py status` prints the ESP32's advertised MAC,
# or read it from its serial boot log: "[BLE] MAC Address: ...")
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
| `POST` | `/api/open` | Open sequence: slide → lift → UNDOCK. Self-healing (works from ERROR). |
| `POST` | `/api/close` | Close sequence: DOCK (if lifted) → lift down → slide close. Self-healing, or interrupts an in-progress open. |
| `POST` | `/api/toggle` | Pause/resume the current motor move |

### Safety

| Method | Endpoint | Description |
|---|---|---|
| `POST` | `/api/emergency-stop` | Stop both pad motors immediately + best-effort RESET to the docking system |
| `POST` | `/api/reset` | Manually set state from ERROR to a limit-switch-verified OPEN/CLOSED, without moving motors |

### Docking (BLE) — manual/testing

| Method | Endpoint | Description |
|---|---|---|
| `GET` | `/api/dock/status` | BLE connection state + last known dock-controller status |
| `POST` | `/api/dock/dock` | Send DOCK directly (background; poll status). Normally driven by `/api/close` instead. |
| `POST` | `/api/dock/undock` | Send UNDOCK directly (background; poll status). Normally driven by `/api/open` instead. |
| `POST` | `/api/dock/reset` | Send RESET to the dock controller |

### Debug — isolated single-axis motor testing

| Method | Endpoint | Description |
|---|---|---|
| `GET` | `/api/debug/limits` | Raw debounced limit-switch states, no motor movement |
| `POST` | `/api/debug/motor/{motor_id}/jog` | Jog `OPENING` or `LIFT` toward one limit switch (`{"direction": "FORWARD"\|"REVERSE"}`). Works from CLOSED/OPEN/ERROR. |
| `POST` | `/api/debug/motor/stop` | Cancel the current jog |
| `GET` | `/api/debug/motor/status` | Current/last jog result |

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
# Open the pad (slides, lifts, undocks the drone)
curl -X POST http://localhost:8000/api/open

# Check status
curl http://localhost:8000/api/status

# Close the pad (docks the drone first, then lowers/closes)
curl -X POST http://localhost:8000/api/close

# Emergency stop — both the pad and the dock controller
curl -X POST http://localhost:8000/api/emergency-stop

# Check raw limit switches without moving anything
curl http://localhost:8000/api/debug/limits

# Jog just the lift motor down, independent of the full sequence
curl -X POST http://localhost:8000/api/debug/motor/LIFT/jog \
  -H "Content-Type: application/json" -d '{"direction": "REVERSE"}'
```

### Response Format

Command endpoints return:
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
| `DRONE_PAD_OPENING_STEP_DELAY_US` | `750` | NEMA 23 step delay (μs) |
| `DRONE_PAD_LIFT_STEP_DELAY_US` | `550` | NEMA 17 step delay (μs) |
| `DRONE_PAD_OPENING_TIMEOUT_S` | `160.0` | NEMA 23 operation timeout |
| `DRONE_PAD_LIFT_TIMEOUT_S` | `30.0` | NEMA 17 operation timeout |
| `DRONE_PAD_SWITCH_DEBOUNCE_MS` | `50` | Limit switch debounce time |
| `DRONE_PAD_DOCK_MAC` | *(ESP32's BLE MAC)* | Dock controller BLE address |
| `DRONE_PAD_DOCK_CONNECT_TIMEOUT_S` | `10.0` | BLE connect timeout |
| `DRONE_PAD_DOCK_COMMAND_TIMEOUT_S` | `120.0` | Max wait for DOCK/UNDOCK to complete (a full DOCK cycle has been observed taking ~60-90s on real hardware) |

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

Dock-Lock Controller (ESP32) ── BLE ── Raspberry Pi
No physical wiring — connects over Bluetooth Low Energy only.
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

### Debugging the BLE Link Standalone

`docking_controller.py` doubles as a CLI for testing the dock controller without the full service running (stop `drone-pad.service` first — the ESP32 only accepts one BLE connection at a time):

```bash
python docking_controller.py status
python docking_controller.py dock
python docking_controller.py undock
python docking_controller.py reset --mac AA:BB:CC:DD:EE:FF -v
```

### Firmware

The ESP32 dock-lock controller's firmware lives in its own repository, checked out locally at `reference/drone_dock_locking_system/` (gitignored here — it has its own remote). See that repo for firmware-side changes; `pad_state_machine.py`'s docstrings and this README describe the Pi-side protocol contract it expects (`DOCK`/`UNDOCK`/`RESET` commands, `DOCKED`/`UNDOCKED` stable terminal statuses, `ERROR`).

## License

Internal use only.
