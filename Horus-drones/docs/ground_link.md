# Ground-station link

The mission starts the link itself. `clump_declump.py` spawns
`orbs_term/drone_link.py` on startup, so flying is the only thing you have to
remember to do.

```
clump_declump.py
      │ spawns (detached, once)
      ▼
  drone_link.py ──tails──► logs/clump_<stamp>/*.csv ──ws──► ground station
```

## Configure the address once

Either:

```bash
export ORBS_GS=10.0.0.5:8765
```

or add a line to `orbs_term/drone.conf`, which already holds the drone identity:

```
name = alpha
server = 10.0.0.5:8765
```

Then just fly:

```bash
python3 clump_declump.py
python3 clump_declump.py --link 10.0.0.9:8765     # override for one run
python3 clump_declump.py --no-link                # fly with no telemetry
```

If no address is configured the mission logs `no ground station configured` and
flies normally. A missing or unreachable ground station never blocks a flight.

## The link outlives the mission, on purpose

This is the part that would be easy to get wrong. The archive is uploaded
**after** the run ends — the link waits for `--settle` seconds of no writes
before zipping. If the mission killed the link on exit, the upload would never
happen.

So the link is spawned with `start_new_session=True`, which detaches it into its
own process group. It survives the mission exiting, Ctrl-C on the mission, and
the terminal closing, and keeps running to finish the upload.

That also means Ctrl-C on the mission does **not** stop the link. Stop it
explicitly if you want it gone:

```bash
python3 ground_link.py status
python3 ground_link.py stop
```

## One link, not one per flight

A pidfile at `/tmp/orbs_link.pid` records the running link. Before spawning, the
mission checks whether that pid is alive **and** whether its
`/proc/<pid>/cmdline` still contains `drone_link.py` — a bare liveness check
would be fooled by pid reuse after a reboot, and would silently attach the
mission to whatever unrelated process inherited the number.

If a link is already up, the mission logs `already running as pid N` and leaves
it alone. Fly ten missions in a row and you get one link, which then picks up
each new session directory automatically.

## Where its output goes

`Horus-drones/main/logs/link.log`, appended across runs. That is where to look
for connection failures, upload progress and throughput:

```
[link] connected to ws://10.0.0.5:8765 as alpha.7f3c
[link] following clump_20260806_231204
[link] zipping clump_20260806_231204
[link] uploading 0.16 MB from 0.0 MB
[link] sent 0.16 MB in 1.4s (0.91 Mbit/s)
```

## Annotation delays the upload

`post_run.annotate_run()` writes into `analysis/` for roughly the length of the
flight after landing, and the settle timer watches the whole session directory.
So with annotation on, the upload starts a couple of minutes after touchdown.

That is correct — do not zip a directory that is still being written — but if
you want the logs promptly, fly with `--no-annotate` and re-render on the
laptop. That is also the bandwidth-optimal path, since `analysis/` is excluded
from the archive anyway.

## Manual control

`ground_link.py` is usable on its own:

```bash
python3 ground_link.py start 10.0.0.5:8765
python3 ground_link.py status
python3 ground_link.py stop
```

## If you would rather it started at boot

The systemd unit in `orbs_term/docs/usage.md` does the same job earlier. The two
are compatible: with the service installed, the mission finds the link already
running and does nothing.
