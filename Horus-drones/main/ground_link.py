#!/usr/bin/env python3
import fcntl
import os
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ORBS_DIR = os.path.abspath(os.path.join(HERE, "..", "..", "orbs_term"))
LINK_SCRIPT = os.path.join(ORBS_DIR, "drone_link.py")
DRONE_CONF = os.path.join(ORBS_DIR, "drone.conf")
LINK_LOG = os.path.join(HERE, "logs", "link.log")
SPAWN_LOCK = "/tmp/orbs_spawn.lock"

sys.path.insert(0, ORBS_DIR)


def _single():
    from orbs import single
    return single


def running():
    return _single().holder()


def scan_processes():
    found = []
    self_pid = os.getpid()
    for entry in os.listdir("/proc"):
        if not entry.isdigit():
            continue
        pid = int(entry)
        if pid == self_pid:
            continue
        try:
            with open(f"/proc/{pid}/cmdline", "rb") as handle:
                parts = handle.read().split(b"\0")
        except OSError:
            continue
        if any(part.endswith(b"drone_link.py") for part in parts):
            found.append(pid)
    return found


def configured_server():
    env = os.environ.get("ORBS_GS", "").strip()
    if env:
        return env
    try:
        with open(DRONE_CONF) as handle:
            for line in handle:
                line = line.split("#", 1)[0].strip()
                if line.startswith("server") and "=" in line:
                    return line.split("=", 1)[1].strip()
    except OSError:
        pass
    return ""


def start(server=None, logs=None, extra=()):
    server = server or configured_server()
    if not server:
        return 0, "no ground station configured (ORBS_GS or drone.conf)"
    if not os.path.exists(LINK_SCRIPT):
        return 0, f"{LINK_SCRIPT} not found"

    try:
        gate = open(SPAWN_LOCK, "a+")
    except OSError as error:
        return 0, f"cannot open spawn lock: {error}"

    try:
        fcntl.flock(gate.fileno(), fcntl.LOCK_EX)
        existing = running()
        if existing:
            return existing, f"already running as pid {existing}"

        command = [sys.executable, LINK_SCRIPT, server]
        if logs:
            command += ["--logs", logs]
        command += list(extra)

        os.makedirs(os.path.dirname(LINK_LOG), exist_ok=True)
        try:
            stream = open(LINK_LOG, "a", buffering=1)
            process = subprocess.Popen(
                command, stdout=stream, stderr=subprocess.STDOUT,
                stdin=subprocess.DEVNULL, start_new_session=True,
                cwd=ORBS_DIR)
        except OSError as error:
            return 0, f"spawn failed: {error}"

        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            holder = running()
            if holder:
                return holder, f"started pid {holder} -> {server}"
            if process.poll() is not None:
                return 0, (f"link exited immediately (rc={process.returncode}), "
                           f"see {LINK_LOG}")
            time.sleep(0.05)
        return process.pid, (f"started pid {process.pid} -> {server} "
                             f"(lock not confirmed)")
    finally:
        fcntl.flock(gate.fileno(), fcntl.LOCK_UN)
        gate.close()


def stop(timeout=10.0):
    pid = running()
    if not pid or pid < 0:
        return False
    try:
        os.kill(pid, 15)
    except OSError:
        return False
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if not running():
            return True
        time.sleep(0.1)
    return not running()


def main():
    import argparse
    parser = argparse.ArgumentParser(
        description="Start, stop or query the ground-station link.")
    parser.add_argument("action", choices=["start", "stop", "status"])
    parser.add_argument("server", nargs="?")
    args = parser.parse_args()

    if args.action == "status":
        pid = running()
        strays = [p for p in scan_processes() if p != pid]
        if pid > 0:
            print(f"link running as pid {pid}")
        elif pid < 0:
            print("link running, pid unknown")
        else:
            print("link not running")
        if strays:
            print(f"WARNING: unlocked drone_link processes: {strays}")
        return 0
    if args.action == "stop":
        print("stopped" if stop() else "not running")
        return 0
    pid, note = start(args.server)
    print(note)
    return 0 if pid else 1


if __name__ == "__main__":
    raise SystemExit(main())
