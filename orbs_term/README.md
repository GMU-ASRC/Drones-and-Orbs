# orbs_term

Terminal ground station for the Horus fleet. You run the console; the drones
connect out to it.

```
  drone A ──┐
  drone B ──┼──ws──►  ground.py  (websocket server + Textual console)
  drone C ──┘                    └── inbox/<drone>/<session>.zip
```

The drones are the websocket **clients**. That direction matters: the Pi needs
no inbound port, no static address and no port forwarding, and it reconnects by
itself when the link drops.

## Run the console

```bash
pip install -r requirements.txt
python3 ground.py                       # listens on 0.0.0.0:8765
python3 ground.py --port 9000 --inbox ~/orbs_runs
```

The console owns the terminal, so give it one of its own. Backgrounding it with
`&` suspends the process the moment it reads the keyboard, and the server never
binds. To run it in the background, or under systemd, use `--headless`, which
serves and logs to stdout with no UI:

```bash
python3 ground.py --headless &          # server only
python3 ground.py                       # console, its own terminal
```

## The drones start their own link

`clump_declump.py` spawns `drone_link.py` when the mission starts, so nothing
has to be launched by hand. Set the address once, per drone:

```bash
export ORBS_GS=10.0.0.5:8765          # or a `server =` line in drone.conf
```

See `Horus-drones/docs/ground_link.md`. To run it manually instead:

```bash
python3 drone_link.py 10.0.0.5:8765
```

It follows the newest session directory under
`Horus-drones/main/logs/`, streams telemetry while the flight runs, and once a
run has been quiet for `--settle` seconds it zips the session and uploads it.
Nothing in the flight code needs changing — the link reads the same CSVs
`FlightLogger` already writes.

```bash
python3 drone_link.py 10.0.0.5:8765 --include-video    # add video.h264
python3 drone_link.py 10.0.0.5:8765 --upload clump_20260806_2312   # one run
```

## Docs

| file | covers |
|---|---|
| [docs/usage.md](docs/usage.md) | keys, panels, deployment as a service |
| [docs/identity.md](docs/identity.md) | how drones are told apart |
| [docs/protocol.md](docs/protocol.md) | every message on the wire |
| [docs/archives.md](docs/archives.md) | the post-flight zip and upload |
| [docs/bandwidth.md](docs/bandwidth.md) | every layer that compresses, and why |
