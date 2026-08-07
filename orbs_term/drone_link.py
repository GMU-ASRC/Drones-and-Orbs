#!/usr/bin/env python3
import argparse
import asyncio
import csv
import hashlib
import json
import os
import sys
import time
import zipfile

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from orbs import identity, protocol, single

from websockets.asyncio.client import connect

STORED_SUFFIXES = (".h264", ".mp4", ".jpg", ".jpeg", ".png", ".gz", ".zip")

STATE_FIELDS = ["state", "state_t", "cmd", "vx", "yaw_rate", "ang_x",
                "range_m", "margin_m", "gap_m", "v_safe", "span_used",
                "span_floor", "n_corners", "quality", "weak", "fresh",
                "confident", "alt_agl", "yaw_deg", "loop_lag_ms", "cam_fps"]
VISION_FIELDS = ["fps", "cv_ms", "accepted", "n_found", "n_corners", "quality",
                 "weak", "span_px", "span_floor", "mask_px", "roi", "state"]
SYSTEM_FIELDS = ["cpu_pct", "cpu_temp_c", "cpu_freq_mhz", "mem_avail_mb",
                 "throttled_now", "uv_now", "wifi_level_dbm", "ping_rtt_ms",
                 "disk_free_mb", "rx_bps", "tx_bps"]


def newest_session(root):
    try:
        entries = [os.path.join(root, d) for d in os.listdir(root)]
    except OSError:
        return None
    dirs = [d for d in entries if os.path.isdir(d)]
    return max(dirs, key=os.path.getmtime) if dirs else None


def numeric(value):
    if value in ("", None):
        return None
    try:
        number = float(value)
    except (TypeError, ValueError):
        return value
    return int(number) if number.is_integer() else round(number, 4)


class Tail:
    def __init__(self, path, fields):
        self.path = path
        self.fields = fields
        self.offset = 0
        self.header = None
        self.latest = None

    def poll(self):
        try:
            size = os.path.getsize(self.path)
        except OSError:
            return None
        if size < self.offset:
            self.offset, self.header = 0, None
        if size == self.offset:
            return None
        with open(self.path, "rb") as handle:
            if self.header is None:
                line = handle.readline()
                if not line.endswith(b"\n"):
                    return None
                self.header = next(
                    csv.reader([line.decode("utf-8", "replace")]), None)
                self.offset = handle.tell()
                if self.header is None:
                    return None
            handle.seek(self.offset)
            block = handle.read()
        if block.endswith(b"\n"):
            self.offset += len(block)
        else:
            block, _, _ = block.rpartition(b"\n")
            self.offset += (len(block) + 1) if block else 0
        text = block.decode("utf-8", "replace")
        rows = [r for r in csv.reader(text.splitlines())
                if len(r) == len(self.header)]
        if not rows:
            return None
        row = dict(zip(self.header, rows[-1]))
        picked = {k: numeric(row.get(k)) for k in self.fields if k in row}
        self.latest = {k: v for k, v in picked.items() if v is not None}
        return self.latest


class EventTail:
    def __init__(self, path):
        self.path = path
        self.offset = 0

    def poll(self):
        try:
            size = os.path.getsize(self.path)
        except OSError:
            return []
        if size < self.offset:
            self.offset = 0
        if size == self.offset:
            return []
        with open(self.path, "rb") as handle:
            handle.seek(self.offset)
            block = handle.read()
        if not block.endswith(b"\n"):
            block, _, _ = block.rpartition(b"\n")
            block = block + b"\n" if block else b""
        self.offset += len(block)
        lines = block.decode("utf-8", "replace").splitlines()
        return [line.strip() for line in lines if line.strip()][-20:]


class Session:
    def __init__(self, path):
        self.path = path
        self.name = os.path.basename(path)
        self.state = Tail(os.path.join(path, "behavior.csv"), STATE_FIELDS)
        self.vision = Tail(os.path.join(path, "vision.csv"), VISION_FIELDS)
        self.system = Tail(os.path.join(path, "system.csv"), SYSTEM_FIELDS)
        self.events = EventTail(os.path.join(path, "events.log"))

    def idle_for(self, watched_only=True):
        newest = 0.0
        for root, dirs, files in os.walk(self.path):
            if watched_only and "analysis" in dirs:
                dirs.remove("analysis")
            for name in files:
                try:
                    newest = max(newest,
                                 os.path.getmtime(os.path.join(root, name)))
                except OSError:
                    pass
        return time.time() - newest if newest else 0.0


def make_archive(session_path, out_dir, include_video, include_analysis):
    os.makedirs(out_dir, exist_ok=True)
    name = os.path.basename(session_path) + ".zip"
    out_path = os.path.join(out_dir, name)
    tmp_path = out_path + ".building"
    root = os.path.dirname(session_path)
    with zipfile.ZipFile(tmp_path, "w", zipfile.ZIP_DEFLATED, True, 6) as zf:
        for folder, _, files in os.walk(session_path):
            rel_folder = os.path.relpath(folder, root)
            if not include_analysis and os.path.basename(folder) == "analysis":
                continue
            for filename in sorted(files):
                if not include_video and filename.startswith("video."):
                    continue
                full = os.path.join(folder, filename)
                arc = os.path.join(rel_folder, filename)
                stored = filename.lower().endswith(STORED_SUFFIXES)
                zf.write(full, arc,
                         zipfile.ZIP_STORED if stored else zipfile.ZIP_DEFLATED)
    os.replace(tmp_path, out_path)
    return out_path


def sha256(path):
    hasher = hashlib.sha256()
    with open(path, "rb") as handle:
        for block in iter(lambda: handle.read(1 << 20), b""):
            hasher.update(block)
    return hasher.hexdigest()


class Uploaded:
    def __init__(self, path):
        self.path = path
        self.done = set()
        if os.path.exists(path):
            try:
                self.done = set(json.load(open(path)))
            except Exception:
                self.done = set()

    def add(self, name):
        self.done.add(name)
        try:
            json.dump(sorted(self.done), open(self.path, "w"))
        except OSError:
            pass


class Link:
    def __init__(self, args):
        self.args = args
        self.ident = identity.load_identity(args.identity)
        self.session = None
        self.uploaded = Uploaded(os.path.join(args.logs, ".uploaded.json"))
        self.pending = []
        self.accepted = None
        self.send_lock = None

    async def send(self, socket, payload):
        async with self.send_lock:
            await socket.send(payload)

    async def run(self):
        delay = 1.0
        while True:
            try:
                await self.session_loop()
                delay = 1.0
            except (OSError, asyncio.TimeoutError) as error:
                print(f"[link] {type(error).__name__}: {error}; "
                      f"retry in {delay:.0f}s", flush=True)
            except Exception as error:
                print(f"[link] {type(error).__name__}: {error}; "
                      f"retry in {delay:.0f}s", flush=True)
            await asyncio.sleep(delay)
            delay = min(delay * 1.8, 30.0)

    async def session_loop(self):
        url = f"ws://{self.args.server}"
        self.send_lock = asyncio.Lock()
        self.sent = {}
        self.keyframe_at = 0.0
        async with connect(url, max_size=None, ping_interval=20,
                           extensions=[protocol.client_deflate()]) as socket:
            await socket.send(protocol.hello(
                self.ident["id"], self.ident["name"],
                color=self.ident["color"], marker=self.ident["marker"]))
            print(f"[link] connected to {url} as "
                  f"{identity.tag(self.ident['id'], self.ident['name'])}",
                  flush=True)
            await asyncio.gather(self.telemetry(socket),
                                 self.archives(socket),
                                 self.drain(socket))

    async def drain(self, socket):
        async for raw in socket:
            message = protocol.decode(raw) if isinstance(raw, str) else None
            if message is None:
                continue
            if message["t"] == protocol.ARCHIVE_ACCEPT:
                self.accepted = message
            elif message["t"] == protocol.ARCHIVE_OK:
                print(f"[link] archive stored: {message.get('path')}",
                      flush=True)

    async def telemetry(self, socket):
        period = 1.0 / max(self.args.rate, 0.5)
        while True:
            self.rotate_session()
            if self.session is not None:
                keyframe = (time.monotonic() - self.keyframe_at
                            > self.args.keyframe)
                if keyframe:
                    self.keyframe_at = time.monotonic()
                for kind, tail, builder in (
                        ("state", self.session.state, protocol.state),
                        ("vision", self.session.vision, protocol.vision),
                        ("system", self.session.system, protocol.system)):
                    latest = tail.poll()
                    if not latest:
                        continue
                    payload = self.changed(kind, latest, keyframe)
                    if payload:
                        await self.send(socket, builder(**payload))
                for line in self.session.events.poll():
                    await self.send(socket, protocol.event("drone", line))
            await asyncio.sleep(period)

    def changed(self, kind, latest, keyframe):
        previous = self.sent.get(kind)
        if keyframe or previous is None:
            self.sent[kind] = dict(latest)
            return latest
        delta = {k: v for k, v in latest.items() if previous.get(k) != v}
        previous.update(delta)
        return delta

    def rotate_session(self):
        newest = newest_session(self.args.logs)
        if newest is None:
            return
        if self.session is None or self.session.path != newest:
            if self.session is not None:
                self.pending.append(self.session.path)
            self.session = Session(newest)
            print(f"[link] following {self.session.name}", flush=True)

    async def archives(self, socket):
        while True:
            await asyncio.sleep(self.args.settle / 2.0)
            candidates = list(self.pending)
            self.pending.clear()
            if (self.session is not None
                    and self.session.idle_for() > self.args.settle):
                candidates.append(self.session.path)
            for path in candidates:
                name = os.path.basename(path)
                if self.args.include_analysis:
                    analysis = os.path.join(path, "analysis")
                    if os.path.isdir(analysis):
                        session = Session(path)
                        if session.idle_for(watched_only=False) < self.args.settle:
                            self.pending.append(path)
                            continue
                if name in self.uploaded.done:
                    continue
                try:
                    await self.send_archive(socket, path)
                    self.uploaded.add(name)
                except Exception as error:
                    print(f"[link] archive {name} failed: {error}", flush=True)
                    self.pending.append(path)

    async def send_archive(self, socket, session_path):
        print(f"[link] zipping {os.path.basename(session_path)}", flush=True)
        archive = make_archive(session_path, self.args.spool,
                               self.args.include_video,
                               self.args.include_analysis)
        size = os.path.getsize(archive)
        digest = sha256(archive)
        self.accepted = None
        await self.send(socket, protocol.archive_offer(
            os.path.basename(archive), size, digest,
            os.path.basename(session_path)))
        for _ in range(300):
            if self.accepted is not None:
                break
            await asyncio.sleep(0.1)
        start = int(self.accepted.get("at", 0)) if self.accepted else 0
        print(f"[link] uploading {size / 1e6:.1f} MB from {start / 1e6:.1f} MB",
              flush=True)
        began = time.monotonic()
        with open(archive, "rb") as handle:
            handle.seek(start)
            while True:
                block = handle.read(protocol.CHUNK_BYTES)
                if not block:
                    break
                await self.send(socket, block)
        await self.send(socket, protocol.archive_done(
            os.path.basename(archive)))
        seconds = max(time.monotonic() - began, 1e-6)
        print(f"[link] sent {(size - start) / 1e6:.1f} MB in {seconds:.1f}s "
              f"({(size - start) * 8 / seconds / 1e6:.2f} Mbit/s)", flush=True)
        if not self.args.keep_zip:
            os.remove(archive)


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    default_logs = os.path.abspath(
        os.path.join(here, "..", "Horus-drones", "main", "logs"))
    parser = argparse.ArgumentParser(
        description="Stream telemetry to the ground station and upload the "
                    "run archive after landing.")
    parser.add_argument("server", nargs="?", default="127.0.0.1:8765",
                        help="host:port of the ground station")
    parser.add_argument("--logs", default=default_logs)
    parser.add_argument("--spool", default="/tmp/orbs_spool")
    parser.add_argument("--identity", default=os.path.join(here, "drone.conf"))
    parser.add_argument("--rate", type=float, default=4.0,
                        help="telemetry polls per second")
    parser.add_argument("--keyframe", type=float, default=30.0,
                        help="seconds between full snapshots; between them "
                             "only changed fields are sent")
    parser.add_argument("--settle", type=float, default=20.0,
                        help="seconds of no writes before a run counts as over")
    parser.add_argument("--include-video", action="store_true")
    parser.add_argument("--include-analysis", action="store_true")
    parser.add_argument("--keep-zip", action="store_true")
    parser.add_argument("--upload", metavar="SESSION",
                        help="upload one session and exit")
    parser.add_argument("--allow-duplicate", action="store_true",
                        help="skip the single-instance lock")
    args = parser.parse_args()

    guard = None
    if not args.allow_duplicate and not args.upload:
        guard = single.acquire()
        if guard is None:
            print(f"[link] already running as pid {single.holder()}; exiting",
                  flush=True)
            return 0

    link = Link(args)
    if args.upload:
        path = args.upload
        if not os.path.isdir(path):
            path = os.path.join(args.logs, args.upload)

        async def once():
            url = f"ws://{args.server}"
            link.send_lock = asyncio.Lock()
            async with connect(url, max_size=None,
                               extensions=[protocol.client_deflate()]) as socket:
                await socket.send(protocol.hello(
                    link.ident["id"], link.ident["name"],
                    color=link.ident["color"], marker=link.ident["marker"]))
                drain = asyncio.create_task(link.drain(socket))
                await link.send_archive(socket, path)
                await asyncio.sleep(0.5)
                drain.cancel()
        try:
            asyncio.run(once())
        except KeyboardInterrupt:
            print("\n[link] stopped")
        return 0

    try:
        asyncio.run(link.run())
    except KeyboardInterrupt:
        print("\n[link] stopped")
    finally:
        if guard is not None:
            guard.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
