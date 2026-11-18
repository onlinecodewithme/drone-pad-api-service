"""
Robot E-stop integration check — simulation only, no hardware, no serial port.

Starts a fake E-stop hub on a test port, a simulated pad, and the real
EStopMonitor between them, then presses/releases the fake E-stop.

    DRONE_PAD_SIMULATION=true venv/bin/python tests/test_estop.py
"""
import json, os, sys, threading, time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

os.environ["DRONE_PAD_SIMULATION"] = "true"
os.environ["DRONE_PAD_SWITCH_ACTIVE_LOW"] = "true"   # matches the simulator's trigger = pull LOW
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from config import PadState, load_settings
from gpio_manager import GPIOManager
from motor_controller import MotorController
from pad_state_machine import PadStateMachine
from estop_monitor import EStopMonitor

PORT = 18771
hub = {"active": False, "seq": 0}
FAILURES = []


def check(cond, msg):
    print(("  PASS  " if cond else "  FAIL  ") + msg)
    if not cond:
        FAILURES.append(msg)


class Handler(BaseHTTPRequestHandler):
    def do_GET(self):
        body = json.dumps(hub).encode()
        self.send_response(200); self.send_header("Content-Length", str(len(body))); self.end_headers()
        self.wfile.write(body)
    def log_message(self, *a): pass


def set_hub(active):
    hub["active"] = active; hub["seq"] += 1


server = ThreadingHTTPServer(("127.0.0.1", PORT), Handler)
threading.Thread(target=server.serve_forever, daemon=True).start()

settings = load_settings()
assert settings.simulation_mode
gpio = GPIOManager(settings); gpio.initialize()
assert gpio.is_simulation, "refusing to run against real GPIO"
sw = settings.limit_switch_pins
gpio.sim_trigger_switch(sw.close_limit, True)      # pad physically closed
gpio.sim_trigger_switch(sw.lift_lower, True)
motor = MotorController(gpio, settings)
pad = PadStateMachine(gpio, motor, settings, docking=None, loop=None)

monitor = EStopMonitor(
    url=f"http://127.0.0.1:{PORT}/estop",
    on_engage=lambda reason: pad.set_external_estop(True, reason),
    on_release=lambda: pad.set_external_estop(False),
    poll_interval_s=0.05,
)
monitor.start(); time.sleep(0.3)

print("\n=== 1. Idle pad: press locks it, release resets it ===")
check(pad.get_status().state == PadState.CLOSED, f"starts CLOSED (got {pad.get_status().state.value})")
set_hub(True); time.sleep(0.5)
st = pad.get_status()
check(st.external_estop and st.state == PadState.ERROR, "E-stop latched, state ERROR")
check("E-stop" in (st.error_message or ""), f"error message explains it: {st.error_message!r}")
for name in ("open", "close", "toggle", "reset_from_error"):
    r = getattr(pad, name)()
    check(r["success"] is False and "E-stop" in r["message"], f"{name}() refused while pressed")
check(pad.get_status().state == PadState.ERROR, "still ERROR — open()/close() did not self-heal past the E-stop")
set_hub(False); time.sleep(0.5)
st = pad.get_status()
check(not st.external_estop and st.state == PadState.CLOSED, f"released -> reset to CLOSED (got {st.state.value})")

print("\n=== 2. Pressed mid-sequence: motor stops, stays stopped ===")
r = pad.open()
check(r["success"], f"open accepted: {r['message']}")
time.sleep(0.4)
check(pad.get_status().state == PadState.OPENING_SLIDE, f"sequence running (got {pad.get_status().state.value})")
set_hub(True); time.sleep(3.0)      # the simulator sleeps out a whole pulse chunk before it sees the cancel
st = pad.get_status()
check(st.state == PadState.ERROR and st.active_motor is None, "sequence aborted, no active motor")
check(not (pad._sequence_thread and pad._sequence_thread.is_alive()), "sequence thread exited")
set_hub(False); time.sleep(6.0)
st = pad.get_status()
check(not st.external_estop, "unlatched on release")
print(f"         state after release: {st.state.value} — {st.error_message}")

print("\n=== 3. Press + release between two polls still stops the pad ===")
events_before = len([e for e in pad.get_event_log() if e.event == "ROBOT_ESTOP_ENGAGED"])
hub["seq"] += 2            # hub saw press and release; `active` is False again
time.sleep(7.0)
events_after = len([e for e in pad.get_event_log() if e.event == "ROBOT_ESTOP_ENGAGED"])
check(events_after == events_before + 1, "brief press detected through seq")
check(not pad.get_status().external_estop, "and released again")

print("\n=== 4. Hub unreachable ===")
server.shutdown(); server.server_close(); time.sleep(0.5)
ms = monitor.status()
check(ms["hub_reachable"] is False and ms["active"] is False, "default: reported unreachable, last state kept")
monitor.stop()
fs = EStopMonitor(url=f"http://127.0.0.1:{PORT}/estop",
                  on_engage=lambda reason: pad.set_external_estop(True, reason),
                  on_release=lambda: pad.set_external_estop(False),
                  poll_interval_s=0.05, fail_safe=True, unreachable_grace_s=0.3)
fs.start(); time.sleep(1.0)
check(pad.get_status().external_estop, "fail_safe=True: unreachable hub locks the pad")
fs.stop()

gpio.cleanup()
print()
if FAILURES:
    print(f"{len(FAILURES)} FAILURE(S)"); sys.exit(1)
print("ALL PASSED")
