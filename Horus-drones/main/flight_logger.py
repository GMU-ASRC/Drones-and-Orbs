#!/usr/bin/env python3
import csv
import json
import os
import threading
import time


FLUSH_INTERVAL_S = 2.0
DEFAULT_LOG_ROOT = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "logs")


class _CsvStream:

    def __init__(self, path: str, fieldnames):
        self._path = path
        self._lock = threading.Lock()
        self._fh = open(path, "w", newline="", buffering=1 << 16)
        self._w = csv.DictWriter(self._fh, fieldnames=fieldnames,
                                 restval="", extrasaction="ignore")
        self._w.writeheader()
        self._last_flush = time.monotonic()
        self.rows = 0

    def write(self, **row):
        try:
            with self._lock:
                self._w.writerow(row)
                self.rows += 1
                now = time.monotonic()
                if now - self._last_flush >= FLUSH_INTERVAL_S:
                    self._fh.flush()
                    self._last_flush = now
        except Exception as e:
            print(f"[logger] csv write failed ({self._path}): {e}")

    def flush(self):
        try:
            with self._lock:
                self._fh.flush()
        except Exception:
            pass

    def close(self):
        try:
            with self._lock:
                self._fh.flush()
                self._fh.close()
        except Exception:
            pass


class FlightLogger:

    def __init__(self, root: str = DEFAULT_LOG_ROOT, tag: str = "flight"):
        stamp = time.strftime("%Y%m%d_%H%M%S")
        self.dir = os.path.join(root, f"{tag}_{stamp}")
        os.makedirs(self.dir, exist_ok=True)
        self.snap_dir = os.path.join(self.dir, "snaps")
        os.makedirs(self.snap_dir, exist_ok=True)

        self.t0_mono = time.monotonic()
        self._streams = {}
        self._meta = {
            "session": os.path.basename(self.dir),
            "t0_mono": self.t0_mono,
            "t0_wall": time.time(),
            "t0_wall_iso": time.strftime("%Y-%m-%dT%H:%M:%S"),
        }
        self._ev_lock = threading.Lock()
        self._ev = open(os.path.join(self.dir, "events.log"), "w",
                        buffering=1)
        self.event("logger", f"session -> {self.dir}")


    def csv(self, name: str, fieldnames) -> _CsvStream:
        if name not in self._streams:
            path = os.path.join(self.dir, f"{name}.csv")
            self._streams[name] = _CsvStream(path, fieldnames)
        return self._streams[name]

    def path(self, filename: str) -> str:
        return os.path.join(self.dir, filename)


    def add_meta(self, section: str, data: dict):
        self._meta[section] = data
        self._write_meta()

    def _write_meta(self):
        try:
            with open(os.path.join(self.dir, "session.json"), "w") as f:
                json.dump(self._meta, f, indent=2, default=str)
        except Exception as e:
            print(f"[logger] session.json write failed: {e}")


    def event(self, tag: str, msg: str, echo: bool = True):
        t = time.monotonic() - self.t0_mono
        line = f"[{t:8.3f}] {tag:<9} {msg}"
        if echo:
            print(f"[{tag}] {msg}", flush=True)
        try:
            with self._ev_lock:
                self._ev.write(line + "\n")
        except Exception:
            pass


    def close(self):
        self._meta["duration_s"] = round(time.monotonic() - self.t0_mono, 3)
        self._meta["rows"] = {n: s.rows for n, s in self._streams.items()}
        self._write_meta()
        for s in self._streams.values():
            s.close()
        try:
            self._ev.close()
        except Exception:
            pass
        print(f"[logger] session closed: {self.dir}")


class NullLogger:

    dir = None
    snap_dir = None
    t0_mono = 0.0

    class _Sink:
        rows = 0

        def write(self, **row):
            pass

        def flush(self):
            pass

        def close(self):
            pass

    def csv(self, name, fieldnames):
        return self._Sink()

    def path(self, filename):
        return os.devnull

    def add_meta(self, section, data):
        pass

    def event(self, tag, msg, echo=True):
        if echo:
            print(f"[{tag}] {msg}", flush=True)

    def close(self):
        pass
