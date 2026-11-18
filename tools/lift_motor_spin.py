#!/usr/bin/env python3
"""
Lift motor spin test — keep the lift (NEMA 17, TB6600) stepping continuously
so its driver, wiring and mechanics can be checked by hand.

Standalone: talks to the GPIO pins directly with lgpio, the same way
gpio_manager.py does. drone-pad.service must be STOPPED first, because it
holds these pins:

    sudo systemctl stop drone-pad
    sudo venv/bin/python tools/lift_motor_spin.py            # down, until Ctrl+C
    sudo systemctl start drone-pad

Pins are read from config.py / .env (lift STEP, DIR, ENABLE (active LOW),
lift_upper and lift_lower switches; inputs with pull-up; HIGH means
triggered, matching DRONE_PAD_SWITCH_ACTIVE_LOW=false and the lift's
normally-closed switch wiring).

Safety:
  * Stops by itself when the limit switch in the direction of travel
    triggers (lift_lower going down, lift_upper going up), unless
    --ignore-limits is given.
  * Ctrl+C, SIGTERM, --duration running out, or any error stops the pulses
    and disables the driver (ENABLE high) before exiting.
"""

import argparse
import os
import signal
import sys
import time

import lgpio

GPIO_CHIP = 0

# Pins come from the service's own config (config.py + .env), so a re-mapped
# pin is picked up here too.
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from config import load_settings  # noqa: E402

_settings = load_settings()
STEP_PIN = _settings.lift_motor_pins.step
DIR_PIN = _settings.lift_motor_pins.direction
ENABLE_PIN = _settings.lift_motor_pins.enable          # TB6600 ENA: LOW = enabled
LIFT_UPPER_PIN = _settings.limit_switch_pins.lift_upper
LIFT_LOWER_PIN = _settings.limit_switch_pins.lift_lower
DIR_LEVEL = {"down": 0, "up": 1}   # Direction.REVERSE = 0 (down), FORWARD = 1 (up); lift not inverted
MIN_STEP_US = 500                  # same floor as gpio_manager._MIN_STEP_DELAY_US
LIMIT_CONFIRM_READS = 3            # consecutive reads before a limit counts as triggered

_stop = False


def _on_signal(signum, _frame):
    global _stop
    _stop = True


def main() -> int:
    ap = argparse.ArgumentParser(description="Keep the drone-pad lift motor stepping for wiring checks")
    ap.add_argument("--direction", choices=["down", "up"], default="down")
    ap.add_argument("--step-us", type=int, default=550,
                    help="step period in microseconds (default 550, the service's lift setting)")
    ap.add_argument("--duration", type=float, default=0,
                    help="stop after this many seconds (default 0 = run until Ctrl+C / SIGTERM)")
    ap.add_argument("--ignore-limits", action="store_true",
                    help="do NOT stop at the limit switch (only for checking a motor with nothing attached)")
    ap.add_argument("--no-dir", action="store_true",
                    help="leave the DIR pin untouched (as an input); the motor turns whichever way the "
                         "DIR wiring currently says. Use when DIR is externally held at a level, so the "
                         "Pi never fights it.")
    args = ap.parse_args()

    step_us = max(args.step_us, MIN_STEP_US)
    half_us = step_us // 2
    stop_pin = LIFT_LOWER_PIN if args.direction == "down" else LIFT_UPPER_PIN
    stop_name = "lift_lower" if args.direction == "down" else "lift_upper"

    signal.signal(signal.SIGINT, _on_signal)
    signal.signal(signal.SIGTERM, _on_signal)

    h = lgpio.gpiochip_open(GPIO_CHIP)
    claimed = []
    try:
        try:
            outputs = [(STEP_PIN, 0), (ENABLE_PIN, 1)]
            if not args.no_dir:
                outputs.insert(1, (DIR_PIN, DIR_LEVEL[args.direction]))
            for pin, level in outputs:
                lgpio.gpio_claim_output(h, pin, level)
                claimed.append(pin)
            for pin in (LIFT_UPPER_PIN, LIFT_LOWER_PIN):
                lgpio.gpio_claim_input(h, pin, lgpio.SET_PULL_UP)
                claimed.append(pin)
            if args.no_dir:
                # Read-only: input with no pull, so the Pi never drives or loads the DIR line.
                lgpio.gpio_claim_input(h, DIR_PIN, lgpio.SET_PULL_NONE)
                claimed.append(DIR_PIN)
        except lgpio.error as exc:
            print(f"Could not claim GPIO ({exc}). Is drone-pad.service still running? "
                  "Stop it first: sudo systemctl stop drone-pad", file=sys.stderr)
            return 1
        time.sleep(0.02)  # let pull-ups settle

        def sw(pin):
            return lgpio.gpio_read(h, pin) == 1  # HIGH = triggered

        print(f"lift_upper={int(sw(LIFT_UPPER_PIN))} lift_lower={int(sw(LIFT_LOWER_PIN))} "
              f"(1 = triggered)", flush=True)
        if not args.ignore_limits and sw(stop_pin):
            print(f"{stop_name} is already triggered — the lift is already at that end. Not starting.", flush=True)
            return 0

        rate = 1_000_000 / step_us
        if args.no_dir:
            print(f"DIR pin left untouched (reads {'HIGH' if lgpio.gpio_read(h, DIR_PIN) else 'LOW'} "
                  "if readable) — direction is whatever the wiring sets.", flush=True)
        print(f"Running lift {'(DIR untouched)' if args.no_dir else args.direction.upper()} at {step_us} µs/step (~{rate:.0f} steps/s), "
              f"{'until Ctrl+C' if not args.duration else f'for {args.duration:.0f} s'}"
              f"{'' if args.ignore_limits else f', stopping at {stop_name}'}.", flush=True)

        if not args.no_dir:
            lgpio.gpio_write(h, DIR_PIN, DIR_LEVEL[args.direction])
            time.sleep(0.00001)                   # TB6600 DIR setup time
        lgpio.gpio_write(h, ENABLE_PIN, 0)        # enable driver
        time.sleep(0.005)
        lgpio.tx_pulse(h, STEP_PIN, half_us, half_us, 0, 0)  # 0 cycles = continuous, hardware-timed
        t0 = time.time()
        next_report = t0
        hits = 0
        reason = "stopped by Ctrl+C / SIGTERM"
        last_sw = None

        while not _stop:
            now = time.time()
            upper, lower = sw(LIFT_UPPER_PIN), sw(LIFT_LOWER_PIN)
            if (upper, lower) != last_sw:
                print(f"[{now - t0:7.1f}s] lift_upper={int(upper)} lift_lower={int(lower)}", flush=True)
                last_sw = (upper, lower)
            if not args.ignore_limits:
                hits = hits + 1 if sw(stop_pin) else 0
                if hits >= LIMIT_CONFIRM_READS:
                    reason = f"{stop_name} triggered — reached the end of travel"
                    break
            if args.duration and now - t0 >= args.duration:
                reason = f"duration of {args.duration:.0f} s reached"
                break
            if now >= next_report:
                print(f"[{now - t0:7.1f}s] running, ~{int((now - t0) * rate)} steps sent", flush=True)
                next_report = now + 5
            time.sleep(0.02)

        elapsed = time.time() - t0
        print(f"Stopping: {reason}. Ran {elapsed:.1f} s, ~{int(elapsed * rate)} steps.", flush=True)
        return 0
    finally:
        # Always: stop pulses, step line low, driver disabled, release pins.
        try:
            lgpio.tx_pulse(h, STEP_PIN, 0, 0)
        except Exception:
            pass
        for pin, level in ((STEP_PIN, 0), (ENABLE_PIN, 1)):
            try:
                if pin in claimed:
                    lgpio.gpio_write(h, pin, level)
            except Exception:
                pass
        for pin in claimed:
            try:
                lgpio.gpio_free(h, pin)
            except Exception:
                pass
        lgpio.gpiochip_close(h)
        print("Driver disabled, GPIO released.", flush=True)


if __name__ == "__main__":
    sys.exit(main())
