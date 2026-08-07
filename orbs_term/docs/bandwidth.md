# Bandwidth

The Pi Zero 2W's radio is a CYW43438: 2.4 GHz only, 1x1, single antenna, on
SDIO. WiFi is half-duplex, so there is no separate up and down capacity, and the
drone's **transmit** path is the weak one — every byte crosses SDIO under host
CPU scheduling, each frame has to win CSMA/CA airtime, and TX current draw is
what trips the undervoltage and throttle flags. Your access point also
out-powers and out-antennas the Pi, so the uplink degrades first with range.

Everything this tool does is the drone transmitting. So every layer compresses.

## 1. The link is deflated

`permessage-deflate` is negotiated explicitly on both ends rather than left to
defaults, with `level 6` and `memLevel 7`. Level 6 rather than 9 because the
last few percent of ratio costs disproportionate CPU on a Zero 2W, and CPU is
already the scarce resource during flight.

Deflate keeps its context across messages, so the repeated JSON keys in a
telemetry stream cost almost nothing after their first appearance.

## 2. Telemetry sends only what changed

The drone keeps the last value it sent for every field and transmits only the
differences. A full snapshot goes out on connect and every `--keyframe` seconds
(default 30) so a field can never drift out of sync.

This is the largest continuous saving, because most fields do not change
between ticks. A drone holding station in `SEARCH` has a constant `state`,
`cmd`, `alt_agl`, `range_m` and system block — those ticks now send **nothing at
all** rather than a full frame, because an empty delta is skipped entirely.

The console merges deltas into the row it already holds, so the display is
unaffected.

Blank CSV cells are dropped rather than sent as null, which is most of the
saving on a frame where nothing was detected.

## 3. Only the newest CSV row is sent

The drone tails each CSV and sends the latest row, not every row written since
the last send. A console that fell behind should show the present, not replay
the past — and nothing is lost, because the complete record is on the SD card
and arrives in the archive.

## 4. The archive compresses per file

| file type | how | why |
|---|---|---|
| `.csv`, `.log`, `.json` | deflate level 6 | repetitive numeric text, 2-4x |
| `.h264`, `.mp4`, `.jpg`, `.png`, `.gz`, `.zip` | stored | already compressed; re-compressing burns Pi CPU and usually grows the file |

## 5. The archive excludes what you can regenerate

| | size |
|---|---:|
| CSVs, events, session.json, snaps | **160 KB** |
| `video.h264` | 2.9 MB |
| `analysis/annotated.mp4` | 24 MB |

`analysis/` is never included by default. `annotated.mp4` is 24 MB against the
2.9 MB `video.h264` it was rendered from — pull the raw and re-render on the
laptop with `replay_cage.py`, which is 8x less over the air. `video.*` is also
excluded by default, since you usually only want footage for the flights that
went wrong.

That is 160 KB for a normal run against 55 MB for the whole session directory.

If you are pulling the raw video, consider flying with `--no-annotate` as well:
it stops the drone spending two minutes of CPU rendering a file you are not
going to transfer.

## Ordering

The archive is deliberately the last thing to happen. A run that is still flying
never competes with an upload for the radio, and telemetry stays live during an
upload because sends are serialised per message rather than per transfer.

## What is not compressed, and why

The archive chunks travel through the same deflated websocket, so a zip that is
already compressed is passed through the deflate context at roughly a 1.0 ratio
for some CPU. Avoiding that would mean a second, uncompressed connection, which
costs a socket, a reconnect path and a second failure mode to reason about. The
per-file choice inside the zip already does the real work: the bytes on the
wire are the smallest form of that data either way.
