# Telling drones apart

Three things identify a drone, and they are deliberately layered so that losing
one still leaves the others working.

## The id

A 6-hex-character string, stable across reboots and reflashes of the SD card as
long as the machine is the same:

```
sha256(/etc/machine-id or first non-loopback MAC or hostname)[:6]
```

`machine-id` first because it survives interface renaming and USB WiFi dongles
being swapped. MAC second because it survives an OS reinstall that regenerates
`machine-id`. Hostname last so the chain always terminates.

The id is what the ground station keys on. A drone that drops and reconnects
comes back to **the same row** rather than appearing twice, and its history,
event log and archive list are preserved across the gap.

Override it in `drone.conf` next to `drone_link.py`:

```
id = alpha01
name = alpha
color = #ff5f87
marker = A
```

## The name

Human-readable, defaults to the hostname. Purely cosmetic — two drones may share
a name without confusing the console, because the id disambiguates them.

The console shows `name.xxxx`, the name plus the first four of the id. That is
the string to quote when talking about a specific aircraft.

## The colour and marker

Both are derived from the id by hash, so they are **stable without being
configured**. The same aircraft is the same colour every session, on every
ground station, with no shared registry. Ten palette entries and ten marker
letters; with a handful of drones a collision is unlikely, and if two do collide
you can pin one in `drone.conf`.

The colour is used for the drone's row, its marker glyph, its name in the detail
pane, and its prefix in the event log — so a glance at the log tells you which
aircraft said what without reading the text.

The marker is a letter rather than a filled block so the console stays readable
over SSH, in `screen`, and on terminals without truecolor.

## Why not let the ground station assign ids

It was tempting to have the console hand out numbers on connect. It breaks the
moment you restart the console mid-session: every drone reconnects and gets a
different number, and the log you were reading now refers to nothing. Deriving
the id on the drone means the mapping survives a restart of either end.
