# Wire protocol

One websocket per drone. Text frames carry JSON, binary frames carry archive
payload. `permessage-deflate` is negotiated on both ends, which matters: the
telemetry is repetitive JSON and compresses several-fold, and the Pi Zero 2W's
transmit path is the scarce resource.

Every JSON message has `t` (type) and `ts` (unix seconds).

## Drone to ground

### `hello`

First message on every connection. Until it arrives the server ignores
everything else.

```json
{"t":"hello","id":"7f3c2a","name":"alpha","v":1,"color":"#00d7ff","marker":"A"}
```

### `state`

Mission state, from `behavior.csv`. Sent at `--rate` Hz, default 4.

```json
{"t":"state","state":"APPROACH","state_t":3.4,"cmd":"approach","vx":0.35,
 "range_m":2.94,"margin_m":1.73,"gap_m":2.48,"v_safe":1.17,"n_corners":8,
 "quality":0.41,"weak":0,"alt_agl":3.02,"loop_lag_ms":12}
```

### `vision`

Detector health, from `vision.csv`: `fps`, `cv_ms`, `accepted`, `n_found`,
`span_px`, `mask_px`, `roi`.

### `system`

From `system.csv`: `cpu_pct`, `cpu_temp_c`, `mem_avail_mb`, `throttled_now`,
`uv_now`, `wifi_level_dbm`, `ping_rtt_ms`, `disk_free_mb`, `rx_bps`, `tx_bps`.

### `event`

One line from `events.log`.

```json
{"t":"event","src":"behavior","msg":"SEARCH -> APPROACH"}
```

### `archive_offer` / `archive_done`

See [archives.md](archives.md).

## Ground to drone

`archive_accept` (carrying the resume offset), `archive_ok`, `archive_fail`,
`pong`. Nothing else — the console does not command the aircraft, and adding
that would need an authentication story this does not have.

## Only the last row is sent, and only what changed

The drone tails each CSV and sends **the newest row**, not every row it has not
sent yet. A console that fell behind should show the present, not replay the
past, and dropping intermediate rows costs nothing because the full record is
already on the SD card and arrives later in the archive.

Within that row, only fields whose value **changed** since the last send go on
the wire. A full snapshot is sent on connect and every `--keyframe` seconds
(default 30). The console merges deltas into the row it already holds, so
`state` messages are cumulative, not self-contained.

An empty delta is not sent at all, so a drone holding station transmits nothing
between keyframes.

Fields that are blank in the CSV are omitted rather than sent as null, which is
most of the saving on a frame where nothing was detected.

See [bandwidth.md](bandwidth.md) for the rest of the compression story.

## Reconnection

The drone retries with exponential backoff from 1 s to 30 s, forever. On
reconnect it sends `hello` again and the console reattaches to the existing row
by id. An interrupted archive resumes from its byte offset rather than
restarting.

## What this does not do

No authentication, no TLS, no authorization. It assumes the flight-line network
is the trust boundary. Do not expose the port to anything you do not control.
