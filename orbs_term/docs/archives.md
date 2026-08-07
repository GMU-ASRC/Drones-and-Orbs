# Post-flight archives

After a run, the drone zips the session and pushes it to the console over the
websocket it is already holding. No inbound port on the Pi, no `scp`, no
remembering which session you have already copied.

## When a run counts as over

The link watches the newest directory under the log root. A run is complete when
no file inside it has been written for `--settle` seconds (default 20), or when
a newer session directory appears, which means the previous flight ended.

Both are checked, because a mission that ends by landing and one that ends by
you starting the next test should behave the same way.

Uploaded sessions are recorded in `.uploaded.json` in the log root, so a
reconnect does not re-send a run you already have.

## What goes in

By default: every CSV, `events.log`, `session.json`, and `snaps/`.

Excluded by default:

| excluded | why |
|---|---|
| `analysis/` | `annotated.mp4` is 24 MB against the 2.9 MB `video.h264` it was rendered from. Re-render locally with `replay_cage.py`. |
| `video.*` | 2.9 MB per 35 s run, and you usually only want it for the flights that went wrong. |

Add them with `--include-video` and `--include-analysis`.

That leaves roughly **160 KB** for a typical run, against 55 MB for the whole
session directory.

## Compression is chosen per file

CSVs and logs are deflated; `.h264`, `.mp4`, `.jpg`, `.png`, `.gz` and `.zip`
are **stored**. Re-compressing already-compressed media wastes Pi CPU and
usually makes the file slightly larger. The CSVs are where the win is — they
compress 2 to 4 times.

## The transfer

```
drone                              ground
  │  archive_offer {name,size,sha}   │
  │ ────────────────────────────────►│  opens <name>.part, checks for a resume point
  │  archive_accept {at}             │
  │ ◄────────────────────────────────│
  │  binary chunk (64 KB)            │
  │ ────────────────────────────────►│  appends, updates the progress bar
  │  ... repeat ...                  │
  │  archive_done {name}             │
  │ ────────────────────────────────►│  verifies sha256, renames .part to final
  │  archive_ok {path}               │
  │ ◄────────────────────────────────│
```

Chunks are plain binary frames on the same socket. Telemetry keeps flowing
between them, so the console stays live while a 3 MB archive is moving.

### Resume

The offset comes from the size of the existing `.part` file, and the drone
seeks there before sending. A link that drops at 80% costs you the last chunk,
not the whole transfer. This is the feature that makes uploads practical on a
Zero 2W's 2.4 GHz radio at range.

### Integrity

sha256 is computed on the drone before sending and verified on the ground before
the `.part` file is renamed. A mismatch deletes the partial and reports
`archive_fail`, so a truncated or corrupted archive never quietly replaces a
good one.

## Where it lands

```
inbox/
  alpha.7f3c/
    clump_20260806_231204.zip
  bravo.91ab/
    clump_20260806_231540.zip
```

Keyed by `name.id`, so two drones with the same name still get separate folders.
