#!/usr/bin/env python3
"""
robot_link.py  --  Bluetooth link from the Mac to the Keyestudio rescue tank

The tank's HM-10 module is Bluetooth Low Energy, so it does not show up or pair
in macOS Bluetooth settings. This script finds it by name ("HMSoft"), connects,
and passes along commands:

    UDP :12003 in   "RESCUE", "DRIVE F|B|L|R", "STOP"   (from pose_bridge.py)
    UDP :12000 out  "R;<status>"                       (to FallGuard.pde HUD)

pose_bridge.py starts this automatically. If macOS blocks Bluetooth for
Processing, double-click "Start Robot Link.command" in this folder instead.
"""

import argparse
import asyncio
import os
import socket
import sys
import threading

try:
    from bleak import BleakClient, BleakScanner
except ImportError:
    print("Missing package (bleak). In Terminal run: pip install -r requirements.txt", flush=True)
    sys.exit(1)

HM10_CHAR = "0000ffe1-0000-1000-8000-00805f9b34fb"   # HM-10 serial characteristic
NAMES = ("hmsoft", "hm-10", "hm10", "bt05", "bt-05", "jdy", "mlt-bt05", "keyes")
SKETCH = ("127.0.0.1", 12000)
LISTEN = ("127.0.0.1", 12003)
COMMANDS = {"RESCUE": b"X", "STOP": b"S", "DRIVE F": b"F", "DRIVE B": b"B",
            "DRIVE L": b"L", "DRIVE R": b"R",
            "TEST 1": b"1", "TEST 2": b"2", "TEST 3": b"3", "TEST 4": b"4",
            "DRIVE a": b"a", "DRIVE s": b"s", "DRIVE d": b"d", "DRIVE g": b"g"}

out = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)


def status(msg: str) -> None:
    print(f"[robot] {msg}", flush=True)
    try:
        out.sendto(f"R;{msg}".encode(), SKETCH)
    except OSError:
        pass


class Inbox(asyncio.DatagramProtocol):
    def __init__(self, queue: asyncio.Queue):
        self.queue = queue

    def datagram_received(self, data, addr):
        self.queue.put_nowait(data.decode(errors="ignore").strip())


async def find_tank():
    devices = await BleakScanner.discover(timeout=6.0)
    for d in devices:
        if d.name and any(n in d.name.lower() for n in NAMES):
            return d
    return None


async def main(args) -> None:
    loop = asyncio.get_running_loop()
    queue: asyncio.Queue = asyncio.Queue()
    try:
        await loop.create_datagram_endpoint(lambda: Inbox(queue), local_addr=LISTEN)
    except OSError:
        print("[robot] another robot link is already running", flush=True)
        return

    while True:
        status("searching for the tank (is it switched on?)")
        try:
            dev = await find_tank()
        except Exception as e:  # Bluetooth off or not allowed
            status(f"Bluetooth error: {e.__class__.__name__}. Is Bluetooth on?")
            await asyncio.sleep(5)
            continue
        if dev is None:
            status("tank not found, retrying")
            await asyncio.sleep(2)
            continue
        try:
            async with BleakClient(dev) as client:
                status(f"connected to {dev.name}")
                while client.is_connected:
                    try:
                        cmd = await asyncio.wait_for(queue.get(), timeout=2.0)
                    except asyncio.TimeoutError:
                        status(f"connected to {dev.name}")   # keep the HUD fresh
                        continue
                    data = COMMANDS.get(cmd)
                    if data:
                        await client.write_gatt_char(HM10_CHAR, data, response=False)
                        if cmd == "RESCUE" or cmd.startswith("TEST"):
                            status(f"{cmd} sent to tank")
        except Exception as e:
            status(f"lost the tank ({e.__class__.__name__}), reconnecting")
            await asyncio.sleep(1)


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--managed", action="store_true", help="quit when the parent quits")
    a = ap.parse_args()
    if a.managed:
        def watch_parent():
            sys.stdin.read()
            os._exit(0)
        threading.Thread(target=watch_parent, daemon=True).start()
    try:
        asyncio.run(main(a))
    except KeyboardInterrupt:
        pass
