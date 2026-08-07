#!/usr/bin/env python3
import os
import subprocess
import sys
import tempfile
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.abspath(
    os.path.join(HERE, "..", "Horus-drones", "main")))

LOCK = os.path.join(tempfile.mkdtemp(), "link.lock")
os.environ["ORBS_LINK_LOCK"] = LOCK

from orbs import single           # noqa: E402
import ground_link                # noqa: E402

FAILURES = []


def check(label, got, want):
    ok = got == want
    print(f"  {'PASS' if ok else 'FAIL'}  {label:<52} "
          f"got={got!r} want={want!r}")
    if not ok:
        FAILURES.append(label)


print("== lock is exclusive within one process ==")
first = single.acquire(LOCK)
check("first acquire succeeds", first is not None, True)
check("holder reports our pid", single.holder(LOCK), os.getpid())
check("second acquire refused", single.acquire(LOCK), None)
first.close()
check("released after close", single.holder(LOCK), 0)

print("\n== lock is exclusive across processes ==")
holder_code = (
    "import os,sys,time;"
    f"os.environ['ORBS_LINK_LOCK']={LOCK!r};"
    f"sys.path.insert(0,{HERE!r});"
    "from orbs import single;"
    f"h=single.acquire({LOCK!r});"
    "print('got' if h else 'refused', flush=True);"
    "time.sleep(4)")
child = subprocess.Popen([sys.executable, "-c", holder_code],
                         stdout=subprocess.PIPE, text=True)
check("child acquired", child.stdout.readline().strip(), "got")
check("parent refused while child holds", single.acquire(LOCK), None)
check("holder reports child pid", single.holder(LOCK), child.pid)
child.terminate()
child.wait()
time.sleep(0.2)
check("lock free after child dies", single.holder(LOCK), 0)

print("\n== lock survives SIGKILL without going stale ==")
child = subprocess.Popen([sys.executable, "-c", holder_code],
                         stdout=subprocess.PIPE, text=True)
child.stdout.readline()
check("held before kill", single.holder(LOCK) == child.pid, True)
child.kill()
child.wait()
time.sleep(0.2)
check("no stale lock after SIGKILL", single.holder(LOCK), 0)

print("\n== ground_link spawns exactly one link, concurrently ==")
fake_dir = tempfile.mkdtemp()
fake_link = os.path.join(fake_dir, "drone_link.py")
with open(fake_link, "w") as handle:
    handle.write(
        "import os,sys,time\n"
        f"sys.path.insert(0,{HERE!r})\n"
        f"os.environ['ORBS_LINK_LOCK']={LOCK!r}\n"
        "from orbs import single\n"
        f"g=single.acquire({LOCK!r})\n"
        "if g is None:\n"
        "    print('duplicate refused', flush=True); raise SystemExit(0)\n"
        "print('link up', flush=True)\n"
        "time.sleep(30)\n")

ground_link.LINK_SCRIPT = fake_link
ground_link.ORBS_DIR = fake_dir
ground_link.LINK_LOG = os.path.join(fake_dir, "link.log")
ground_link.SPAWN_LOCK = os.path.join(fake_dir, "spawn.lock")
os.environ["ORBS_GS"] = "127.0.0.1:8765"

results = []
for attempt in range(5):
    results.append(ground_link.start())
pids = {pid for pid, _ in results}
check("all five calls report the same pid", len(pids), 1)
print("      notes:")
for pid, note in results:
    print(f"        {note}")

alive = ground_link.scan_processes()
running_fake = [p for p in alive if p in pids]
check("exactly one link process alive", len(running_fake), 1)

print("\n== stop works, and a later start makes a new one ==")
check("stop returns True", ground_link.stop(), True)
check("nothing running after stop", ground_link.running(), 0)
pid2, note2 = ground_link.start()
check("restart got a different pid", pid2 not in pids and pid2 > 0, True)
ground_link.stop()

print("\n== no server configured is a soft failure ==")
os.environ.pop("ORBS_GS")
ground_link.DRONE_CONF = os.path.join(fake_dir, "missing.conf")
pid3, note3 = ground_link.start()
check("returns 0 without raising", pid3, 0)
print(f"      note: {note3}")

print()
if FAILURES:
    print(f"{len(FAILURES)} FAILED: {FAILURES}")
    raise SystemExit(1)
print("all single-instance checks passed")
