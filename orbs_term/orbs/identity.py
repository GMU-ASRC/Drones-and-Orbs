import glob
import hashlib
import os
import socket

PALETTE = [
    "#00d7ff", "#5fff87", "#ffd75f", "#ff87d7", "#87afff",
    "#ffaf5f", "#5fffd7", "#ff5f87", "#afff5f", "#d7af5f",
]

MARKERS = "OATRIUXKVZ"


def _mac():
    for path in sorted(glob.glob("/sys/class/net/*/address")):
        if "/lo/" in path:
            continue
        try:
            with open(path) as handle:
                value = handle.read().strip()
        except OSError:
            continue
        if value and value != "00:00:00:00:00:00":
            return value
    return ""


def machine_key():
    for path in ("/etc/machine-id", "/var/lib/dbus/machine-id"):
        try:
            with open(path) as handle:
                value = handle.read().strip()
            if value:
                return value
        except OSError:
            pass
    return _mac() or socket.gethostname()


def drone_id(override=None):
    if override:
        return override.strip().lower()[:16]
    digest = hashlib.sha256(machine_key().encode()).hexdigest()
    return digest[:6]


def drone_name(override=None):
    return override.strip() if override else socket.gethostname()


def color_for(drone_id):
    index = int(hashlib.sha256(drone_id.encode()).hexdigest(), 16)
    return PALETTE[index % len(PALETTE)]


def marker_for(drone_id):
    index = int(hashlib.sha256(("m" + drone_id).encode()).hexdigest(), 16)
    return MARKERS[index % len(MARKERS)]


def tag(drone_id, name):
    return f"{name}.{drone_id[:4]}"


def load_identity(path):
    config = {}
    if path and os.path.exists(path):
        with open(path) as handle:
            for line in handle:
                line = line.split("#", 1)[0].strip()
                if "=" in line:
                    key, value = line.split("=", 1)
                    config[key.strip()] = value.strip()
    ident = drone_id(config.get("id"))
    return {
        "id": ident,
        "name": drone_name(config.get("name")),
        "color": config.get("color") or color_for(ident),
        "marker": config.get("marker") or marker_for(ident),
    }
