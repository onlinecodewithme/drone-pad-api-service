# Drone Pad API Service

REST API service for controlling a robotic drone pad with dual stepper motors on Raspberry Pi, integrated with a separate ESP32-based dock-lock controller over USB serial.

## Hardware Overview

| Component | Type | Driver | Function |
|---|---|---|---|
| Opening Motor | NEMA 23 | TB6600 (24V) | Slides the pad open/closed |
| Lift Motor | NEMA 17 | TB6600 (24V) | Raises/lowers the pad |
| Open Limit Switch | NC/NO | — | Detects fully open (slide) position |
| Close Limit Switch | NC/NO | — | Detects fully closed (slide) position |
| Lift Upper Switch | NC/NO | — | Detects fully raised position |
| Lift Lower Switch | NC/NO | — | Detects fully lowered position |
| Dock-Lock Controller | ESP32 + 3 steppers | — | Docks/undocks the drone once the pad is lifted, over USB serial |

> **TB6600 Driver Notes**: Enable is active-LOW (ENA- to GND, ENA+ from GPIO). Signal pins (PUL+, DIR+, ENA+) connect to RPi GPIO; negative pins (PUL-, DIR-, ENA-) connect to RPi GND.

The dock-lock controller is a physically separate device (firmware at `reference/drone_dock_locking_system/`, own repo) — the Pi talks to it over a USB cable (the board's CP2102 USB-UART), no GPIO wiring between the two boards. BLE was removed on both sides in November 2026.

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

### Local pin overrides (`.env`)

The table above lists the **defaults** in `config.py`. `config.load_settings()` also loads a git-ignored `.env` file from the service folder, so one robot's wiring can differ without code changes. Real environment variables (for example systemd `Environment=` lines) take precedence over `.env`. The `tools/` scripts read the same config.

**This robot overrides the lift's step and direction pins** (since 2026-11-17, in `.env`):

| Signal | Default | This robot | Header pin |
|---|---|---|---|
| Lift STEP (TB6600 PUL+) | GPIO 26 | **GPIO 7** | 26 |
| Lift DIR (TB6600 DIR+) | GPIO 13 | **GPIO 8** | 24 |
| Lift ENABLE (TB6600 ENA+) | GPIO 11 | unchanged | 23 |

The Pi's GPIO 26 and GPIO 13 pads are damaged (see the fault note below), so the two signals were physically moved to these healthy pins.

A quick, motion-free health check for any driver signal pin: set it as an input and read it with the pull-up on. A pin wired to a TB6600 input (GPIO to the + terminal, − terminal to ground) reads **0** even with the pull-up; an unconnected pin reads 1. Then drive it high as an output: a healthy pin reads 1.

```bash
sudo pinctrl set 26 ip pu && pinctrl lev 26     # 0 = driver input attached, 1 = open circuit
```

**Known hardware fault (2026-11-17):** on this Pi, GPIO 26 measures 0 at its pad even when the RP1 chip sends "high, output enabled" at 12 mA drive, and GPIO 13 stays high even when driven low at 12 mA. Both were confirmed with the PCB disconnected, so the pads themselves are damaged; do not use header pins 37 and 33 for outputs. After moving the lift's step and direction lines to GPIO 7 and 8 (and correcting the motor direction at the driver), the lift lowered to its limit switch and stopped: 11,545 steps in 8.8 s.

## Docking System (USB serial)

The ESP32 dock-lock controller docks/undocks the drone itself — separate motors, separate firmware, separate power. The Pi talks to it over USB serial (`docking_controller.py`, 115200 8N1) and:

- Opens `/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0` on startup (override with `DRONE_PAD_DOCK_PORT`; if that path is missing it falls back to the only CP210x device present), confirms the docking firmware answers `STATUS`, and auto-reconnects if the cable is unplugged and plugged back in
- Sends `DOCK` / `UNDOCK` / `RESET` / `STATUS` as text lines and tracks progress from the firmware's `STATE:<state>` lines (`UNKNOWN`, `UNDOCKING_M1/M2`, `DOCKING_M2/M1/PROP_OPEN/PROP_CLOSE`, `PROP_CYCLE_OPEN/CLOSE`, `TESTING`, the stable resting states `DOCKED`/`UNDOCKED`, `ERROR`)
- Is fully integrated into the pad's own open/close sequence (see State Machine below) — you don't call the dock endpoints directly in normal operation, `/api/open` and `/api/close` do it for you at the right moment

### Serial protocol

| Direction | Line | Meaning |
|---|---|---|
| Pi → ESP32 | `DOCK`, `UNDOCK`, `PROPCYCLE`, `RESET`, `STATUS` | Commands. `PROPCYCLE` runs only the propeller closer (open → close). `JOG1`/`JOG2`/`JOG3` exist for bench tests only. |
| ESP32 → Pi | `STATE:<state>` | Sent on every state change, and once at boot |
| ESP32 → Pi | `[STATUS] State=<state> M1_undock=... Prop_close=...` | Reply to `STATUS`, includes limit switch readings |
| ESP32 → Pi | `System busy, cannot dock.` / `Unknown command...` | Command rejected — the Pi fails the command immediately |
| ESP32 → Pi | `=== DOCKING SYSTEM STARTING ===` | Board rebooted — any in-flight command is failed |

Everything else the firmware prints is logged by the service as `Dock firmware: ...`.

A DOCK/UNDOCK only succeeds if it ends in the state it was meant to reach (`DOCKED` / `UNDOCKED`). Ending in `ERROR`, or in the opposite resting state, is a failure. While a command runs, the Pi also polls `STATUS` every 5 s as a backstop: a command the firmware never started fails within about 5 s instead of waiting out the full 120 s timeout.

**Opening the port must not reset the ESP32.** The dev board's auto-reset circuit only fires when DTR and RTS differ. The kernel raises both together on open and drops both together on close, and `docking_controller.py` leaves pyserial's defaults alone. Don't toggle DTR/RTS individually on this port.

`/api/dock/*` endpoints exist for manual testing/debugging the serial link independent of the pad sequence (see API Reference).

## Operation Sequences

### Dock modes

What open and close ask of the dock controller depends on the **dock mode**. Switch it at any time the pad is idle; no restart is needed.

| Mode | Open | Close |
|---|---|---|
| `propeller_only` (default) | Slide open → lift up. Done. | Propeller closer cycle (open → close) → lift down → slide close |
| `full` (original behaviour) | Slide open → lift up → UNDOCK | DOCK (M2 → M1 → propeller open → close) → lift down → slide close |

In `propeller_only` mode the two docking motors (M1/M2) never move. The dock step uses the firmware's `PROPCYCLE` command, which runs the same two propeller stages DOCK ends with, on their own.

```bash
curl localhost:8000/api/dock/mode                                    # current mode
curl -X PUT localhost:8000/api/dock/mode -H 'Content-Type: application/json' -d '{"mode":"full"}'
curl -X PUT localhost:8000/api/dock/mode -H 'Content-Type: application/json' -d '{"mode":"propeller_only"}'
```

- A change applies from the **next** open or close. Each sequence reads the mode once when it starts, so switching can never change a sequence halfway.
- A change is refused (`409`) while a pad sequence or dock command is running.
- A mode set through the API is saved to `dock_mode.json` (git-ignored) and survives restarts. Without that file, the startup default is `DRONE_PAD_DOCK_MODE` (default `propeller_only`). Delete the file to go back to the environment default.
- The current mode is also in `GET /api/dock/status`.

### Open Sequence
```
propeller_only:  CLOSED → [NEMA 23 forward → open limit] → [NEMA 17 forward → lift upper limit] → OPEN
full:            CLOSED → [NEMA 23 forward → open limit] → [NEMA 17 forward → lift upper limit]
                        → [UNDOCK over USB serial → UNDOCKED] → OPEN
```
The pad only reports `OPEN` (and, in `full` mode, only sends UNDOCK) once `lift_upper` is independently re-confirmed triggered — not just because the previous stage returned.

### Close Sequence
```
propeller_only:  OPEN → [PROPCYCLE: propeller open → close, only if currently lifted]
                      → [NEMA 17 reverse → lift lower limit] → [NEMA 23 reverse → close limit] → CLOSED
full:            OPEN → [DOCK over USB serial → DOCKED, only if currently lifted]
                      → [NEMA 17 reverse → lift lower limit] → [NEMA 23 reverse → close limit] → CLOSED
```
The dock step is skipped if the pad isn't confirmed lifted right now (e.g. already closed, or an open interrupted before the lift finished). When it runs, it must succeed before either pad motor moves; if it fails or times out, the pad stays open and reports `ERROR`.

- `full`: success means the dock ends `DOCKED`.
- `propeller_only`: success means the cycle finishes without `ERROR` **and** a `STATUS` read afterwards shows the propeller closer's close limit (`Prop_close`) triggered. The docking motors weren't moved, so the dock itself may report `DOCKED`, `UNDOCKED` or `UNKNOWN`.

### Interrupt (Close during Open)
A close command received during `OPENING_SLIDE`/`OPENING_LIFT` cancels the current motor move and closes from wherever the pad physically is. Interrupting `UNDOCKING`/`DOCKING` is rejected — those are live operations of the dock mechanism, not safely abortable mid-flight. Use emergency-stop instead.

### Toggle (Pause/Resume)
Pauses or resumes an in-progress *motor* move. Rejected during `UNDOCKING`/`DOCKING` — those are waits on the dock controller, not something pausing on the Pi side can affect.

## State Machine

```
CLOSED ──open──► OPENING_SLIDE ──limit──► OPENING_LIFT ──limit──► UNDOCKING ──USB──► OPEN
  ▲                                                     │  (full mode only)            │
  │                                                     └──── propeller_only ─────────►│
  │                                                                                    │close
  │                                                                                    ▼
  └──limit── CLOSING_SLIDE ◄──limit── CLOSING_LIFT ◄──USB── DOCKING ◄──────────────────┘
```

`DOCKING` is the dock step in both modes: the full DOCK sequence in `full`, the propeller closer cycle in `propeller_only`.

`ERROR` is reachable from any state (motor timeout, a failed hard-safety-gate, a failed/timed-out dock step or UNDOCK, or emergency-stop) and disables both pad motors immediately.

## Safety Architecture

This system controls physical mechanical hardware with a drone potentially attached, so several layers of defense are built in:

1. **Hard safety gates, not just sequencing.** Before arming the lift motor, the code independently re-reads `open_limit` rather than trusting that the previous stage returned without an exception. Before sending `UNDOCK`/`DOCK`, it independently re-reads `lift_upper`. These checks use plain `if`/`return`, never `assert` — `assert` is silently stripped when Python runs with `-O`, which would be a real, easy-to-miss hole in a systemd deployment.
2. **Multi-sample confirmation.** A single instantaneous limit-switch read isn't trusted for these gates — each check requires the switch to read consistently across 3 samples spaced ~150ms apart before it's acted on. This exists because this hardware has a known wiring-reliability history (see `git log` on `pad_state_machine.py`/`gpio_manager.py`) where switches could briefly, simultaneously misreport.
3. **Self-healing, never a remembered-state guess.** `open()`/`close()` never trust `self._state` to answer "are we already there" — every call live-verifies the pad's actual position via its limit switches and safely no-ops any stage already satisfied. This means the pad recovers from `ERROR` (including a fresh power-cycle before its switches have settled) automatically the moment you call the command you actually want — no manual reset dance required. `reset_from_error()`/`POST /api/reset` still exist for cases where you want to *tell* the software a position without moving any motors (e.g. after manually verifying by hand).
4. **Emergency-stop reaches both systems.** `POST /api/emergency-stop` disables both pad motors at the hardware level *and* sends `RESET` to the docking controller over USB serial, best-effort — "stop everything" means both boards, not just the Pi's own two motors.
5. **Blocking calls never run on the event loop.** `emergency_stop()`, `close()`'s interrupt path, and the debug jog stop all contain `thread.join()`/`future.result()` — genuinely blocking calls. These are offloaded to a thread pool executor (`_run_blocking()` in `main.py`) rather than called directly from an `async def` handler, which would otherwise freeze the entire event loop — including the very docking coroutines a blocked call might itself be waiting on.
6. **A dead docking link fails fast instead of hanging.** Serial reads run on a dedicated thread; an unplugged cable, a firmware reboot, or a rejected command fails the in-flight DOCK/UNDOCK immediately with a clear message, and the background loop reconnects on its own. (The previous BLE link needed `asyncio.wait_for()` guards because BlueZ/D-Bus calls could hang forever with no error; serial has no equivalent failure mode.) `_docking_precheck()` still gives an in-flight reconnect a bounded grace window (`_DOCKING_CONNECT_WAIT_S`) before failing a DOCK/UNDOCK outright.
7. **The result must match the request.** An UNDOCK that ends `DOCKED` (or a DOCK that ends `UNDOCKED`) is reported as a failure, never as success.

## Installation

```bash
# Clone or copy to Raspberry Pi
cd /home/x4mc1/Documents/drone-pad-api-service

# Create virtual environment and install dependencies
python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt   # includes pyserial, for the USB docking link

# Copy environment config
cp .env.example .env
# Edit .env as needed (it is loaded automatically by config.py; real environment
# variables win). The dock controller's serial port defaults to the
# CP2102's /dev/serial/by-id/ name; set DRONE_PAD_DOCK_PORT only if yours differs
# (ls -l /dev/serial/by-id/).
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

### Docking (USB serial) — manual/testing

| Method | Endpoint | Description |
|---|---|---|
| `GET` | `/api/dock/status` | Serial link state, port, and last known dock-controller status |
| `POST` | `/api/dock/dock` | Send DOCK directly (background; poll status). Normally driven by `/api/close` instead. |
| `POST` | `/api/dock/undock` | Send UNDOCK directly (background; poll status). Normally driven by `/api/open` instead. |
| `POST` | `/api/dock/propeller-cycle` | Run only the propeller closer (open → close); docking motors don't move. Normally driven by `/api/close` in `propeller_only` mode. |
| `GET` | `/api/dock/mode` | Current dock mode, and whether it is saved |
| `PUT` | `/api/dock/mode` | `{"mode": "propeller_only" \| "full"}` — switch mode (saved; refused with 409 while running) |
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
| `DRONE_PAD_DOCK_MODE` | `propeller_only` | Startup dock mode (`propeller_only` or `full`). A mode saved through `PUT /api/dock/mode` takes precedence. |
| `DRONE_PAD_DOCK_MODE_FILE` | `<service dir>/dock_mode.json` | Where the API-chosen dock mode is saved |
| `DRONE_PAD_DOCK_PORT` | `/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0` | Dock controller serial port |
| `DRONE_PAD_DOCK_BAUD` | `115200` | Dock controller baud rate |
| `DRONE_PAD_DOCK_CONNECT_TIMEOUT_S` | `10.0` | How long connect keeps asking `STATUS` before giving up (covers the firmware's ~3 s boot delay) |
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

Dock-Lock Controller (ESP32) ── USB cable (CP2102 USB-UART) ── Raspberry Pi
No GPIO wiring between the boards.
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

### Debugging the Docking Link Standalone

`docking_controller.py` doubles as a CLI for testing the dock controller without the full service running. Stop `drone-pad.service` first — the service holds the serial port exclusively:

```bash
python docking_controller.py status
python docking_controller.py dock
python docking_controller.py undock
python docking_controller.py propcycle
python docking_controller.py reset --port /dev/ttyUSB1 -v
```

### Firmware

The ESP32 dock-lock controller's firmware lives in its own repository, checked out locally at `reference/drone_dock_locking_system/` (gitignored here — it has its own remote). See that repo for firmware-side changes; `pad_state_machine.py`'s docstrings and this README describe the Pi-side protocol contract it expects (see Serial protocol above). Flashing the ESP32 also needs the port, so stop `drone-pad.service` first.

## License

Internal use only.
