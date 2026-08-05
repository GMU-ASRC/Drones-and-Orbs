#!/usr/bin/env python3
"""
flight_logger.py -- one directory per flight, several CSV streams + an event log.

Why a separate module: three subsystems (camera, drone, system monitor) each
need to record at their own natural rate, and the flight loop must never block
on an SD-card write. So:

  * every stream is a buffered CSV flushed on a timer (FLUSH_INTERVAL_S), not
    per row -- the Pi Zero 2W's SD card can stall for tens of milliseconds and
    that would show up as jitter in the 10 Hz behavior loop.
  * every write is wrapped in try/except. A logging bug must never take down
    a flight.
  * every row carries t_mono (time.monotonic()) so the streams join exactly.
    t_wall is recorded once in session.json to map onto wall-clock time.

Layout produced:

  logs/flight_20260805_141233/
      session.json   static device + config info, captured at start
      events.log     human-readable timeline (also echoed to stdout)
      vision.csv     one row per camera frame
      behavior.csv   one row per behavior-loop iteration
      system.csv     device stats (cpu/mem/temp/throttle/wifi), 1 Hz
      link.csv       mavlink connectivity stats, 1 Hz
      video.h264     hardware-encoded camera stream (see camera_controller)
      snaps/         event-triggered annotated JPEGs

Everything downstream (analysis/analyze_flight.py) reads this directory.
"""

import csv
import json
import os
import threading
import time

# --------------------------- TUNABLES ---------------------------
FLUSH_INTERVAL_S = 2.0     # buffered rows are pushed to card this often
DEFAULT_LOG_ROOT = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "logs")
# ----------------------------------------------------------------


class _CsvStream:
    """A buffered DictWriter. Tolerant by design: unknown keys are dropped and
    missing keys become blanks, so a caller that adds a field mid-flight logs a
    degraded row instead of raising into the flight loop."""

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
        except Exception as e:                      # never propagate
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
    """Owns the session directory. Hand the same instance to CameraController,
    DroneController and SystemMonitor so every stream lands together."""

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

    # ------------------------- streams -------------------------
    def csv(self, name: str, fieldnames) -> _CsvStream:
        """Create (or return) a CSV stream called <name>.csv."""
        if name not in self._streams:
            path = os.path.join(self.dir, f"{name}.csv")
            self._streams[name] = _CsvStream(path, fieldnames)
        return self._streams[name]

    def path(self, filename: str) -> str:
        return os.path.join(self.dir, filename)

    # ------------------------- metadata -------------------------
    def add_meta(self, section: str, data: dict):
        """Record static info (device model, tunables, camera config...).
        Written to session.json immediately so it survives a hard crash."""
        self._meta[section] = data
        self._write_meta()

    def _write_meta(self):
        try:
            with open(os.path.join(self.dir, "session.json"), "w") as f:
                json.dump(self._meta, f, indent=2, default=str)
        except Exception as e:
            print(f"[logger] session.json write failed: {e}")

    # -------------------------- events --------------------------
    def event(self, tag: str, msg: str, echo: bool = True):
        """Timestamped human-readable timeline entry. These are what you read
        first when reviewing a flight -- state changes, warnings, link drops."""
        t = time.monotonic() - self.t0_mono
        line = f"[{t:8.3f}] {tag:<9} {msg}"
        if echo:
            print(f"[{tag}] {msg}", flush=True)
        try:
            with self._ev_lock:
                self._ev.write(line + "\n")
        except Exception:
            pass

    # -------------------------- teardown -------------------------
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
    """Drop-in no-op, so every subsystem can take logger=None and not branch."""

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
