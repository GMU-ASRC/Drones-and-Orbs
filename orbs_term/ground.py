#!/usr/bin/env python3
import argparse
import asyncio
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from orbs.server import Server
from orbs.state import Fleet


async def headless(host, port, inbox):
    fleet = Fleet()
    server = Server(fleet, host, port, inbox)
    print(f"[ground] listening on ws://{host}:{port}")
    print(f"[ground] archives land in {inbox}")
    seen = 0

    async def report():
        nonlocal seen
        while True:
            await asyncio.sleep(0.5)
            fleet.prune()
            fleet.poll_targets()
            entries = list(fleet.log)
            for stamp, drone_id, source, text in entries[seen:]:
                drone = fleet.get(drone_id)
                label = drone.tag if drone else drone_id
                print(f"{time.strftime('%H:%M:%S', time.localtime(stamp))} "
                      f"{label:<16} {source:<8} {text}", flush=True)
            seen = len(entries)

    await asyncio.gather(server.run(), report())


def main():
    parser = argparse.ArgumentParser(
        description="ORBS ground station: websocket server plus live console.")
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--inbox", default=os.path.join(
        os.path.dirname(os.path.abspath(__file__)), "inbox"))
    parser.add_argument("--headless", action="store_true",
                        help="server only, no console. Use this when running "
                             "in the background or under systemd; the console "
                             "needs a terminal it owns.")
    args = parser.parse_args()
    inbox = os.path.abspath(args.inbox)

    if args.headless or not sys.stdout.isatty():
        try:
            asyncio.run(headless(args.host, args.port, inbox))
        except KeyboardInterrupt:
            print("\n[ground] stopped")
        return 0

    from orbs.app import Ground
    Ground(args.host, args.port, inbox).run()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
