#!/usr/bin/env python3
"""
Motor spin test — step a pad motor continuously until stopped.
No limit switches are read at all.

For checking a TB6600 driver, its wiring and the motor by hand. Standalone:
it drives the GPIO pins directly with lgpio, so drone-pad.service must be
stopped first (it holds these pins):

    sudo systemctl stop drone-pad
    venv/bin/python tools/motor_spin_test.py                 # lift, until Ctrl+C
    venv/bin/python tools/motor_spin_test.py --motor slide
    sudo systemctl start drone-pad

Stop it with Ctrl+C, or from another terminal:  pkill -f motor_spin_test.py

--enable-level: which ENA pin level ENABLES the driver. Default "low", the
    same as the service (ENA+ on the GPIO, ENA- on ground: LOW = enabled).
    Use "high" only for a driver wired the other way round.
--dir: "leave" (default) does not touch the DIR pin at all, so the Pi never
    fights a DIR line held by the wiring; the motor turns whichever way the
    wiring says. "down"/"up" drive DIR (0 = down/close, 1 = up/open).

On exit (Ctrl+C, SIGTERM, error) step pulses stop and the ENA pin is put
back to the level it had BEFORE the test, so a motor that was holding
(e.g. keeping the lift up) keeps holding.
"""

import argparse
import os
import signal
import subprocess
import sys
import time

import lgpio

GPIO_CHIP = 0

# Pins come from the service's own config (config.py + .env), so a re-mapped
# pin is picked up here too instead of being hard-coded in two places.
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from config import load_settings  # noqa: E402

_settings = load_settings()
MOTORS = {
    #         STEP, DIR, ENABLE (BCM)
    "lift":  (_settings.lift_motor_pins.step, _settings.lift_motor_pins.direction, _settings.lift_motor_pins.enable),
    "slide": (_settings.opening_motor_pins.step, _settings.opening_motor_pins.direction, _settings.opening_motor_pins.enable),
}
MIN_STEP_US = 500
_stop = False


def _on_signal(signum, _frame):
    global _stop
    _stop = True


def current_level(pin: int):
    """Read a pin's present level without claiming it (pinctrl), or None."""
    try:
        out = subprocess.run(["pinctrl", "lev", str(pin)], capture_output=True, text=True, timeout=2).stdout.strip()
        return int(out) if out in ("0", "1") else None
    except Exception:
        return None


def main() -> int:
    ap = argparse.ArgumentParser(description="Step a drone-pad motor continuously (no limit switches)")
    ap.add_argument("--motor", choices=sorted(MOTORS), default="lift")
    ap.add_argument("--enable-level", choices=["high", "low"], default="low",
                    help="ENA level that enables the driver (default low, same as the service)")
    ap.add_argument("--dir", choices=["leave", "down", "up"], default="leave")
    ap.add_argument("--dir-level", type=int, choices=[0, 1], default=None,
                    help="drive DIR to this raw level (overrides --dir). Slide: 0 = open, 1 = close "
                         "(its DIR is inverted in config.py). Lift: 0 = down, 1 = up.")
    ap.add_argument("--step-us", type=int, default=550, help="step period in µs (default 550; slide uses 750)")
    ap.add_argument("--duration", type=float, default=0, help="stop after this many seconds (default 0 = until stopped)")
    args = ap.parse_args()

    step_pin, dir_pin, ena_pin = MOTORS[args.motor]
    enable = 1 if args.enable_level == "high" else 0
    half_us = max(args.step_us, MIN_STEP_US) // 2
    rate = 1_000_000 / (half_us * 2)

    before = current_level(ena_pin)
    restore = before if before is not None else (1 - enable)
    print(f"{args.motor}: STEP GPIO{step_pin}, DIR GPIO{dir_pin}, ENA GPIO{ena_pin}. "
          f"ENA was {before if before is not None else 'unknown'} before the test; will restore {restore} on exit.",
          flush=True)

    signal.signal(signal.SIGINT, _on_signal)
    signal.signal(signal.SIGTERM, _on_signal)

    h = lgpio.gpiochip_open(GPIO_CHIP)
    claimed = []
    try:
        try:
            # Claim ENA at its existing level so claiming it changes nothing yet.
            lgpio.gpio_claim_output(h, ena_pin, restore); claimed.append(ena_pin)
            lgpio.gpio_claim_output(h, step_pin, 0); claimed.append(step_pin)
            dir_level = args.dir_level if args.dir_level is not None else (
                None if args.dir == "leave" else (0 if args.dir == "down" else 1))
            if dir_level is not None:
                lgpio.gpio_claim_output(h, dir_pin, dir_level); claimed.append(dir_pin)
                time.sleep(0.00001)  # TB6600 DIR setup time
        except lgpio.error as exc:
            print(f"Could not claim GPIO ({exc}). Is drone-pad.service still running? "
                  "Stop it first: sudo systemctl stop drone-pad", file=sys.stderr)
            return 1

        lgpio.gpio_write(h, ena_pin, enable)
        time.sleep(0.01)
        lgpio.tx_pulse(h, step_pin, half_us, half_us, 0, 0)   # continuous, hardware-timed
        print(f"Running: ENA={enable} ({args.enable_level} = enabled), "
              f"DIR={'untouched' if dir_level is None else dir_level}, "
              f"{half_us * 2} µs/step (~{rate:.0f} steps/s). No limit switches. Ctrl+C / SIGTERM to stop.",
              flush=True)

        t0 = time.time()
        next_report = t0 + 10
        while not _stop:
            time.sleep(0.05)
            if args.duration and time.time() - t0 >= args.duration:
                break
            if time.time() >= next_report:
                el = time.time() - t0
                print(f"[{el:6.0f}s] running, ~{int(el * rate)} steps sent", flush=True)
                next_report += 10
        el = time.time() - t0
        print(f"Stopping after {el:.1f} s, ~{int(el * rate)} steps.", flush=True)
        return 0
    finally:
        try:
            lgpio.tx_pulse(h, step_pin, 0, 0)
        except Exception:
            pass
        try:
            if step_pin in claimed:
                lgpio.gpio_write(h, step_pin, 0)
            if ena_pin in claimed:
                lgpio.gpio_write(h, ena_pin, restore)
        except Exception:
            pass
        for pin in claimed:
            try:
                lgpio.gpio_free(h, pin)
            except Exception:
                pass
        lgpio.gpiochip_close(h)
        print(f"Pulses stopped, ENA restored to {restore}, GPIO released.", flush=True)


if __name__ == "__main__":
    sys.exit(main())
