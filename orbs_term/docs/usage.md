# Using the console

```bash
python3 ground.py [--host 0.0.0.0] [--port 8765] [--inbox ./inbox]
```

## Give the console its own terminal

Textual takes over the terminal, so a backgrounded `ground.py &` is stopped by
the kernel (`SIGTTIN`) as soon as it reads the keyboard — and because the
suspension happens during startup, the websocket server never binds. Drones then
see `ConnectionRefusedError` and retry forever against a socket nobody is
listening on.

`--headless` runs the server and prints events to stdout with no UI. That is the
form to background, pipe, or run under systemd:

```bash
python3 ground.py --headless &
python3 ground.py --headless | tee ground.log
```

It also engages automatically when stdout is not a terminal, so piping does the
right thing without the flag.

Both forms share the same server, inbox and protocol; only the display differs.

## Layout

```
┌───────────────────────────────────────────────────────────────┐
│ fleet table: one row per drone, colour-coded                  │
├───────────────────────────┬───────────────────────────────────┤
│ detail: selected drone    │ event log: every drone, prefixed  │
│  mission state track      │  with its colour and marker       │
│  control + safety numbers │                                   │
│  archive progress         │                                   │
└───────────────────────────┴───────────────────────────────────┘
```

## Keys

| key | does |
|---|---|
| up / down, j / k | move the selection |
| f | log follows the selected drone only |
| c | clear the log |
| q | quit |

## The fleet table

| column | read it as |
|---|---|
| marker | the drone's stable letter and colour |
| drone | `name.id` |
| link | `live` under 3 s, `stale` over, `red` when disconnected |
| TARGET | whether this drone can see another drone right now |
| bearing | where in the field of view, `▲` sliding left to right |
| state | mission state; blank means the flight script is not running |
| in | seconds in that state |
| range / span / corners / q | what the detector currently believes |
| fps | camera rate. Below ~8 means the CV loop is behind and bearings are stale |
| cpu / temp / wifi | Pi health |
| health | `ok`, or the specific problem |

`health` calls out `THROTTLED`, `UNDERVOLT`, temperature at or above 75 C, under
60 MB free, and loop lag over 150 ms. Those are the five that have actually
caused trouble on a Zero 2W.

## The TARGET indicator

The column that answers "is it seeing the other drone right now":

| shown | means |
|---|---|
| `● LOCKED` on green | fresh detection, not from the fallback mask, above `MIN_QUALITY` — good enough to start a chase |
| `◐ seen` | fresh detection that has not cleared the acquire bar |
| `◌ weak` | carried by the loose fallback mask; the bearing is usable, the identity is provisional |
| `· searching` | nothing detected |
| `-` | link is stale or down, so we do not know |

The distinction matters because the detector demonstrably locks onto clutter —
55 off-target frames on the flight2 recording, almost all of them `weak` or low
quality. A row sitting at `◌ weak` while the state is `APPROACH` means the drone
is flying on evidence it does not fully trust.

`range` goes bold white whenever something is seen, dim otherwise, so a glance
down the column separates real measurements from stale ones.

The `bearing` strip shows where the target sits across the 24.3° field of view,
`▲` sliding left to right, with the angle in degrees. A `▲` pinned to either
edge is the signature of a target about to leave frame.

Acquire and lose transitions are also written to the event log in colour, so the
history is there after the fact — `TARGET locked`, `target lost`. The detail
pane shows how long the current state has held and how many acquisitions this
drone has made in the session, which is the fastest way to spot flapping.

## The detail pane

The mission track shows all six states with the current one highlighted, so
progress through `SEARCH → APPROACH → CLUMPED → DECLUMP → DONE` is visible
without reading text.

Underneath are the control and collision numbers: `cmd`, `vx`, `yaw`, `ang_x`,
`range`, `margin`, `gap`, `v_safe`, `weak`, `alt`.

`gap` is the clear air between cage surfaces and `v_safe` is the braking-curve
speed cap — see `Horus-drones/docs/safety.md`. Watching `cmd` sit at `yaw_only`
while `weak` is 1 tells you the drone can see something and does not trust it
enough to fly at it.

## Disconnection

A drone that drops goes red and keeps its row for 20 s, then disappears. Its
history comes back intact on reconnect because the row is keyed by id.

## Running the link as a service

On each drone:

```ini
[Unit]
Description=ORBS telemetry link
After=network-online.target

[Service]
ExecStart=/usr/bin/python3 /home/pi/Drones-and-Orbs/orbs_term/drone_link.py 10.0.0.5:8765
Restart=always
RestartSec=5
User=pi
StandardOutput=journal

[Install]
WantedBy=multi-user.target
```

The link reconnects on its own, so `Restart=always` is belt and braces for the
process dying rather than the link dropping.

## Bandwidth

Telemetry is a few hundred bytes per second per drone after deflate. The archive
is the only thing that moves real data, and it is deliberately last — a run that
is still flying never competes with an upload for the radio.
