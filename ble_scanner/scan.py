#!/usr/bin/env python3
"""Scan for nearby BLE devices and list them (name, address, RSSI)."""

import argparse
import asyncio

from bleak import BleakScanner


def format_device(address: str, name: str, rssi) -> str:
    rssi_str = f"{rssi:>5} dBm" if rssi is not None else "  n/a    "
    return f"{address:<20} {rssi_str:<10} {name}"


async def scan_once(duration: float) -> None:
    print(f"Scanning for {duration:.1f}s...\n")
    devices = await BleakScanner.discover(timeout=duration, return_adv=True)

    if not devices:
        print("No BLE devices found.")
        return

    rows = []
    for address, (device, adv) in devices.items():
        name = device.name or adv.local_name or "(unknown)"
        rows.append((address, name, adv.rssi))

    rows.sort(key=lambda r: r[2] if r[2] is not None else -999, reverse=True)

    print(f"{'ADDRESS':<20} {'RSSI':<10} NAME")
    print("-" * 60)
    for address, name, rssi in rows:
        print(format_device(address, name, rssi))
    print(f"\nFound {len(rows)} device(s).")


async def scan_live(interval: float) -> None:
    seen = {}

    def on_detect(device, adv):
        seen[device.address] = (device.name or adv.local_name or "(unknown)", adv.rssi)

    scanner = BleakScanner(detection_callback=on_detect)
    print(f"Scanning continuously (refreshing every {interval:.1f}s). Press Ctrl+C to stop.\n")

    await scanner.start()
    try:
        while True:
            await asyncio.sleep(interval)
            rows = sorted(
                ((addr, name, rssi) for addr, (name, rssi) in seen.items()),
                key=lambda r: r[2] if r[2] is not None else -999,
                reverse=True,
            )
            print(f"\n{'ADDRESS':<20} {'RSSI':<10} NAME")
            print("-" * 60)
            for address, name, rssi in rows:
                print(format_device(address, name, rssi))
            print(f"({len(rows)} device(s) seen so far)")
    except asyncio.CancelledError:
        pass
    finally:
        await scanner.stop()


def main() -> None:
    parser = argparse.ArgumentParser(description="Scan and list nearby BLE devices")
    parser.add_argument(
        "-d", "--duration", type=float, default=5.0,
        help="Scan duration in seconds for a single scan (default: 5.0)",
    )
    parser.add_argument(
        "-l", "--live", action="store_true",
        help="Scan continuously, printing an updated device list at each interval",
    )
    args = parser.parse_args()

    try:
        if args.live:
            asyncio.run(scan_live(args.duration))
        else:
            asyncio.run(scan_once(args.duration))
    except KeyboardInterrupt:
        print("\nStopped.")


if __name__ == "__main__":
    main()
