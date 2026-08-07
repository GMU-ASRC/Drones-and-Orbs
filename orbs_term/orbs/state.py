import time
from collections import deque

from . import identity

MISSION_STATES = ["INIT", "SEARCH", "APPROACH", "CLUMPED", "DECLUMP", "DONE"]
STALE_S = 3.0
GONE_S = 20.0


class DroneState:
    def __init__(self, drone_id, name, color=None, marker=None):
        self.id = drone_id
        self.name = name
        self.color = color or identity.color_for(drone_id)
        self.marker = marker or identity.marker_for(drone_id)
        self.connected = False
        self.first_seen = time.time()
        self.last_msg = 0.0
        self.remote = ""
        self.state = {}
        self.vision = {}
        self.system = {}
        self.events = deque(maxlen=400)
        self.range_history = deque(maxlen=120)
        self.fps_history = deque(maxlen=120)
        self.archive = None
        self.archives = []
        self.messages = 0
        self.bytes_in = 0
        self.target = "none"
        self.target_since = time.time()
        self.acquisitions = 0

    @property
    def tag(self):
        return identity.tag(self.id, self.name)

    @property
    def age(self):
        return time.time() - self.last_msg if self.last_msg else 1e9

    @property
    def link(self):
        if not self.connected:
            return "offline"
        if self.age > STALE_S:
            return "stale"
        return "live"

    @property
    def forgettable(self):
        return not self.connected and self.age > GONE_S

    def touch(self, nbytes=0):
        self.last_msg = time.time()
        self.messages += 1
        self.bytes_in += nbytes

    def sighting(self):
        if self.link != "live":
            return "unknown"
        if not self.state.get("fresh"):
            return "none"
        if self.state.get("weak"):
            return "weak"
        if self.state.get("confident"):
            return "locked"
        return "seen"

    def update_target(self):
        current = self.sighting()
        if current == self.target:
            return None
        previous, self.target = self.target, current
        self.target_since = time.time()
        if current in ("locked", "seen", "weak") and previous in ("none",
                                                                  "unknown"):
            self.acquisitions += 1
            return f"TARGET {current}"
        if current == "none" and previous in ("locked", "seen", "weak"):
            return "target lost"
        return None

    def apply(self, message):
        kind = message.get("t")
        if kind == "state":
            self.state.update(message)
            value = message.get("range_m")
            if isinstance(value, (int, float)):
                self.range_history.append(float(value))
        elif kind == "vision":
            self.vision.update(message)
            value = message.get("fps")
            if isinstance(value, (int, float)):
                self.fps_history.append(float(value))
        elif kind == "system":
            self.system.update(message)
        elif kind == "event":
            self.events.append((time.time(), message.get("src", "-"),
                                message.get("msg", "")))

    def mission_index(self):
        current = self.state.get("state")
        return MISSION_STATES.index(current) if current in MISSION_STATES else -1

    def health(self):
        problems = []
        system = self.system
        if system.get("throttled_now"):
            problems.append("THROTTLED")
        if system.get("uv_now"):
            problems.append("UNDERVOLT")
        temp = system.get("cpu_temp_c")
        if isinstance(temp, (int, float)) and temp >= 75:
            problems.append(f"{temp:.0f}C")
        free = system.get("mem_avail_mb")
        if isinstance(free, (int, float)) and free < 60:
            problems.append(f"{free:.0f}MB")
        lag = self.state.get("loop_lag_ms")
        if isinstance(lag, (int, float)) and lag > 150:
            problems.append(f"lag {lag:.0f}ms")
        return problems


class Fleet:
    def __init__(self):
        self.drones = {}
        self.log = deque(maxlen=1000)

    def order(self):
        return sorted(self.drones.values(), key=lambda d: (not d.connected,
                                                           d.name, d.id))

    def get(self, drone_id):
        return self.drones.get(drone_id)

    def join(self, drone_id, name, remote, color=None, marker=None):
        drone = self.drones.get(drone_id)
        if drone is None:
            drone = DroneState(drone_id, name, color, marker)
            self.drones[drone_id] = drone
        drone.name = name or drone.name
        if color:
            drone.color = color
        if marker:
            drone.marker = marker
        drone.connected = True
        drone.remote = remote
        drone.touch()
        self.note(drone, "link", f"connected from {remote}")
        return drone

    def leave(self, drone_id, reason=""):
        drone = self.drones.get(drone_id)
        if drone is None:
            return
        drone.connected = False
        self.note(drone, "link", f"disconnected {reason}".strip())

    def note(self, drone, source, text):
        entry = (time.time(), drone.id if drone else "-", source, text)
        self.log.append(entry)
        if drone is not None:
            drone.events.append((time.time(), source, text))

    def prune(self):
        for drone_id in [k for k, v in self.drones.items() if v.forgettable]:
            del self.drones[drone_id]

    def poll_targets(self):
        for drone in self.drones.values():
            change = drone.update_target()
            if change:
                self.note(drone, "target", change)
