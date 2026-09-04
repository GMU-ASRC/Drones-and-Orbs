#!/usr/bin/env python3
"""
Horus Ground Station
- Multi-drone telemetry dashboard
- Per-drone and swarm commands
- SSH into each Pi to list and run Luis's behavior scripts

Usage:
    pip install pymavlink fastapi uvicorn paramiko
    python3 ground_station.py
    open http://localhost:8000
"""

import math
import os
import threading
import time
import traceback

import paramiko
import uvicorn
from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import HTMLResponse
from pydantic import BaseModel
from pymavlink import mavutil

app = FastAPI()
app.add_middleware(CORSMiddleware, allow_origins=["*"], allow_methods=["*"], allow_headers=["*"])

# ─────────────────────────────────────────────────────────────────
# CONFIG
# ─────────────────────────────────────────────────────────────────
DRONES = {
    "horus1": {
        "udp_port":       14550,
        "pi_host":        "192.168.1.124",
        "pi_user":        "orb1",
        "pi_password":    "afcentpi",
        "pi_keyfile":     None,
        "behaviors_path": [
            "/home/orb1/Drones-and-Orbs/Horus-drones/pymavlink-tests",
            "/home/orb1/Drones-and-Orbs/Horus-drones/main",
        ],
    },
    "horus2": {
        "udp_port":       14552,
        "pi_host":        "192.168.1.22",
        "pi_user":        "asrc",
        "pi_password":    "afcentpi",
        "pi_keyfile":     None,
        "behaviors_path": [
            "/home/asrc/Drones-and-Orbs/Horus-drones/pymavlink-tests",
            "/home/asrc/Drones-and-Orbs/Horus-drones/main",
        ],
    },
    "horus3": {
        "udp_port":       14554,
        "pi_host":        "192.168.1.118",
        "pi_user":        "asrc",
        "pi_password":    "afcentpi",
        "pi_keyfile":     None,
        "behaviors_path": [
            "/home/asrc/Drones-and-Orbs/Horus-drones/pymavlink-tests",
            "/home/asrc/Drones-and-Orbs/Horus-drones/main",
        ],
    },
}

# ─────────────────────────────────────────────────────────────────
# Shared state
# ─────────────────────────────────────────────────────────────────
state:  dict[str, dict] = {}
agents: dict[str, "DroneAgent"] = {}

behavior_thread:     threading.Thread | None = None
behavior_stop_event: threading.Event        = threading.Event()
behavior_status = {"running": False, "name": None, "drone": None, "error": None, "output": []}


# ─────────────────────────────────────────────────────────────────
# SSH helper
# ─────────────────────────────────────────────────────────────────
def _ssh_client(cfg: dict) -> paramiko.SSHClient:
    c = paramiko.SSHClient()
    c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    kwargs = dict(hostname=cfg["pi_host"], username=cfg["pi_user"], timeout=5)
    if cfg.get("pi_keyfile"):
        kwargs["key_filename"] = cfg["pi_keyfile"]
    else:
        kwargs["password"] = cfg["pi_password"]
    c.connect(**kwargs)
    return c


def ssh_list_behaviors(cfg: dict) -> list[str]:
    try:
        c = _ssh_client(cfg)
        paths = cfg["behaviors_path"]
        if isinstance(paths, str):
            paths = [paths]
        all_files = []
        for path in paths:
            _, out, _ = c.exec_command(f"ls {path} 2>/dev/null")
            files = [f.strip() for f in out.readlines() if f.strip()]
            all_files.extend(files)
        c.close()
        return sorted(set(all_files))  # deduplicate
    except Exception as e:
        print(f"[SSH] list error: {e}")
        return []


def ssh_run_behavior(cfg: dict, script: str, stop_event: threading.Event):
    try:
        c = _ssh_client(cfg)
        paths = cfg["behaviors_path"]
        if isinstance(paths, str):
            paths = [paths]
        # find which path the script is in
        script_path = None
        for p in paths:
            _, out, _ = c.exec_command(f"ls {p}/{script} 2>/dev/null")
            if out.read().strip():
                script_path = f"{p}/{script}"
                break
        if script_path is None:
            behavior_status["error"] = f"script {script} not found in any path"
            return
        transport = c.get_transport()
        chan = transport.open_session()
        chan.exec_command(f"python3 {script_path}")

        while not stop_event.is_set():
            if chan.recv_ready():
                chunk = chan.recv(1024).decode(errors="replace")
                for line in chunk.splitlines():
                    behavior_status["output"].append(line)
                    behavior_status["output"] = behavior_status["output"][-100:]
            if chan.exit_status_ready():
                break
            time.sleep(0.1)

        if stop_event.is_set():
            c.exec_command(f"pkill -f {script}")

        chan.close()
        c.close()
    except Exception as e:
        behavior_status["error"] = str(e)
        traceback.print_exc()


# ─────────────────────────────────────────────────────────────────
# DroneAgent
# ─────────────────────────────────────────────────────────────────
class DroneAgent:
    def __init__(self, name: str, cfg: dict):
        self.name = name
        self.cfg  = cfg
        self.port = cfg["udp_port"]
        self.conn_str = f"udpin:0.0.0.0:{self.port}"
        self.m = None
        self.connected = False
        self._lock = threading.Lock()

        state[name] = {
            "connected": False, "armed": False, "flight_mode": "—",
            "x": None, "y": None, "z": None,
            "vx": None, "vy": None, "vz": None,
            "roll_deg": None, "pitch_deg": None, "yaw_deg": None,
            "battery_pct": None, "last_heartbeat": None,
            "behaviors": [],
        }

    def _connect(self):
        print(f"[{self.name}] listening on UDP :{self.port} ...")
        self.m = mavutil.mavlink_connection(self.conn_str)
        self.m.wait_heartbeat()
        self.connected = True
        state[self.name]["connected"] = True
        print(f"[{self.name}] heartbeat sys={self.m.target_system}")
        self._refresh_behaviors()

    def _refresh_behaviors(self):
        def _fetch():
            files = ssh_list_behaviors(self.cfg)
            state[self.name]["behaviors"] = files
        threading.Thread(target=_fetch, daemon=True).start()

    def _poll(self):
        msg = self.m.recv_match(
            type=["HEARTBEAT", "LOCAL_POSITION_NED", "ATTITUDE", "BATTERY_STATUS"],
            blocking=False,
        )
        if msg is None:
            return
        t = msg.get_type()
        s = state[self.name]

        if t == "HEARTBEAT":
            # only trust heartbeats from the flight controller (component ID 1)
            if msg.get_srcComponent() != 1:
                return
            s["last_heartbeat"] = time.time()
            s["armed"] = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            # decode PX4 custom mode from upper 16 bits
            main_mode = (msg.custom_mode >> 16) & 0xFF
            mode_map = {
                1: "MANUAL",    2: "ALTITUDE",  3: "POSITION",
                4: "AUTO",      5: "ACRO",      6: "OFFBOARD",
                7: "STABILIZED", 8: "RATTITUDE",
            }
            if main_mode == 4:
                sub_mode = (msg.custom_mode >> 24) & 0xFF
                auto_map = {
                    2: "AUTO.TAKEOFF", 3: "AUTO.LOITER", 4: "AUTO.MISSION",
                    5: "AUTO.RTL",     6: "AUTO.LAND",
                }
                s["flight_mode"] = auto_map.get(sub_mode, "AUTO")
            else:
                s["flight_mode"] = mode_map.get(main_mode, f"MODE {main_mode}")

        elif t == "LOCAL_POSITION_NED":
            s.update({
                "x": round(msg.x, 2), "y": round(msg.y, 2), "z": round(msg.z, 2),
                "vx": round(msg.vx, 2), "vy": round(msg.vy, 2), "vz": round(msg.vz, 2),
            })

        elif t == "ATTITUDE":
            s.update({
                "roll_deg":  round(math.degrees(msg.roll), 1),
                "pitch_deg": round(math.degrees(msg.pitch), 1),
                "yaw_deg":   round(math.degrees(msg.yaw), 1),
            })

        elif t == "BATTERY_STATUS":
            if msg.battery_remaining >= 0:
                s["battery_pct"] = msg.battery_remaining

    def send_arm(self, arm: bool) -> dict:
        with self._lock:
            if not self.connected:
                return {"ok": False, "error": "drone not connected"}
            self.m.mav.command_long_send(
                self.m.target_system, self.m.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
                1 if arm else 0, 0, 0, 0, 0, 0, 0,
            )
            ack = self.m.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
            if ack is None:
                return {"ok": False, "error": "no response from FC"}
            if ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                return {"ok": True}
            reason = self.m.recv_match(type='STATUSTEXT', blocking=True, timeout=1)
            error_msg = reason.text if reason else f"FC rejected (code {ack.result})"
            return {"ok": False, "error": error_msg}

    def send_takeoff(self, altitude: float):
        with self._lock:
            if not self.connected: return
            self.m.mav.command_long_send(
                self.m.target_system, self.m.target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                6, 0, 0, 0, 0, 0,
            )
            time.sleep(0.1)
            self.m.mav.command_long_send(
                self.m.target_system, self.m.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
                1, 0, 0, 0, 0, 0, 0,
            )
            time.sleep(0.1)
            s = state[self.name]
            x   = s.get("x") or 0.0
            y   = s.get("y") or 0.0
            yaw = math.radians(s.get("yaw_deg") or 0.0)
            self._send_setpoint(x, y, -altitude, yaw)

    def send_land(self):
        with self._lock:
            if not self.connected: return
            self.m.mav.command_long_send(
                self.m.target_system, self.m.target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                4, 6, 0, 0, 0, 0,
            )

    def send_estop(self):
        with self._lock:
            if not self.connected: return
            self.m.mav.command_long_send(
                self.m.target_system, self.m.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
                0, 21196, 0, 0, 0, 0, 0,
            )

    def send_goto(self, x: float, y: float, z_up: float):
        with self._lock:
            if not self.connected: return
            yaw = math.radians(state[self.name].get("yaw_deg") or 0.0)
            self._send_setpoint(x, y, -z_up, yaw)

    def send_flight_mode(self, mode: str):
        mode_map = {
            "manual": (1, 0), "altitude": (2, 0), "position": (3, 0),
            "offboard": (6, 0), "stabilized": (8, 0),
            "rtl": (5, 0), "land": (4, 6),
        }
        if mode.lower() not in mode_map: return
        main, sub = mode_map[mode.lower()]
        with self._lock:
            if not self.connected: return
            self.m.mav.command_long_send(
                self.m.target_system, self.m.target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                main, sub, 0, 0, 0, 0,
            )

    def _send_setpoint(self, x, y, z, yaw):
        MASK = (
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
        )
        self.m.mav.set_position_target_local_ned_send(
            0, self.m.target_system, self.m.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED, MASK,
            x, y, z, 0, 0, 0, 0, 0, 0, yaw, 0,
        )

    def refresh_behaviors(self):
        self._refresh_behaviors()

    def run_forever(self):
        while True:
            try:
                self._connect()
                while True:
                    self._poll()
                    time.sleep(0.02)
            except Exception as e:
                print(f"[{self.name}] error: {e} — reconnecting in 5s")
                self.connected = False
                state[self.name]["connected"] = False
                time.sleep(5)


# ─────────────────────────────────────────────────────────────────
# FastAPI
# ─────────────────────────────────────────────────────────────────

class TakeoffBody(BaseModel):
    altitude: float = 1.0

class GotoBody(BaseModel):
    x: float; y: float; z: float

class ModeBody(BaseModel):
    mode: str

class BehaviorBody(BaseModel):
    drone: str
    script: str


@app.get("/state")
def get_state():
    return state

@app.get("/behavior_status")
def get_behavior_status():
    return behavior_status

@app.post("/command/{drone}/arm")
def cmd_arm(drone: str):
    return _agent(drone).send_arm(True)

@app.post("/command/{drone}/disarm")
def cmd_disarm(drone: str):
    return _agent(drone).send_arm(False)

@app.post("/command/{drone}/estop")
def cmd_estop(drone: str):
    _agent(drone).send_estop(); return {"ok": True}

@app.post("/command/{drone}/land")
def cmd_land(drone: str):
    _agent(drone).send_land(); return {"ok": True}

@app.post("/command/{drone}/takeoff")
def cmd_takeoff(drone: str, body: TakeoffBody):
    _agent(drone).send_takeoff(body.altitude); return {"ok": True}

@app.post("/command/{drone}/goto")
def cmd_goto(drone: str, body: GotoBody):
    _agent(drone).send_goto(body.x, body.y, body.z); return {"ok": True}

@app.post("/command/{drone}/mode")
def cmd_mode(drone: str, body: ModeBody):
    _agent(drone).send_flight_mode(body.mode); return {"ok": True}

@app.post("/command/{drone}/refresh_behaviors")
def cmd_refresh(drone: str):
    _agent(drone).refresh_behaviors(); return {"ok": True}

@app.post("/swarm/arm")
def swarm_arm():
    for a in agents.values(): a.send_arm(True)
    return {"ok": True}

@app.post("/swarm/disarm")
def swarm_disarm():
    for a in agents.values(): a.send_arm(False)
    return {"ok": True}

@app.post("/swarm/land")
def swarm_land():
    for a in agents.values(): a.send_land()
    return {"ok": True}

@app.post("/swarm/estop")
def swarm_estop():
    for a in agents.values(): a.send_estop()
    return {"ok": True}

@app.post("/swarm/takeoff")
def swarm_takeoff(body: TakeoffBody):
    for a in agents.values(): a.send_takeoff(body.altitude)
    return {"ok": True}

@app.post("/behavior/run")
def behavior_run(body: BehaviorBody):
    global behavior_thread, behavior_stop_event

    if behavior_status["running"]:
        return {"ok": False, "error": "a behavior is already running — stop it first"}

    agent = _agent(body.drone)
    cfg   = agent.cfg

    if body.script not in state[body.drone].get("behaviors", []):
        return {"ok": False, "error": f"script '{body.script}' not found on Pi"}

    behavior_stop_event = threading.Event()
    behavior_status.update({
        "running": True, "name": body.script,
        "drone": body.drone, "error": None, "output": [],
    })

    def _run():
        try:
            ssh_run_behavior(cfg, body.script, behavior_stop_event)
        except Exception as e:
            behavior_status["error"] = str(e)
        finally:
            behavior_status.update({"running": False, "name": None, "drone": None})

    behavior_thread = threading.Thread(target=_run, daemon=True)
    behavior_thread.start()
    return {"ok": True}

@app.post("/behavior/run_all")
def behavior_run_all(body: BehaviorBody):
    global behavior_thread, behavior_stop_event

    if behavior_status["running"]:
        return {"ok": False, "error": "a behavior is already running — stop it first"}

    connected = {n: a for n, a in agents.items() if a.connected}
    if not connected:
        return {"ok": False, "error": "no drones connected"}

    for name, agent in connected.items():
        if body.script not in state[name].get("behaviors", []):
            return {"ok": False, "error": f"script '{body.script}' not found on {name}"}

    behavior_stop_event = threading.Event()
    behavior_status.update({
        "running": True, "name": body.script,
        "drone": "__all__", "error": None, "output": [],
    })

    def _run():
        try:
            threads = []
            for name, agent in connected.items():
                t = threading.Thread(
                    target=ssh_run_behavior,
                    args=(agent.cfg, body.script, behavior_stop_event),
                    daemon=True,
                )
                t.start()
                threads.append(t)
            for t in threads:
                t.join()
        except Exception as e:
            behavior_status["error"] = str(e)
        finally:
            behavior_status.update({"running": False, "name": None, "drone": None})

    behavior_thread = threading.Thread(target=_run, daemon=True)
    behavior_thread.start()
    return {"ok": True}

@app.post("/behavior/stop")
def behavior_stop():
    if not behavior_status["running"]:
        return {"ok": False, "error": "nothing running"}
    behavior_stop_event.set()
    return {"ok": True}

@app.get("/", response_class=HTMLResponse)
def dashboard():
    with open(os.path.join(os.path.dirname(__file__), "dashboard.html")) as f:
        return f.read()

def _agent(drone: str) -> DroneAgent:
    if drone not in agents:
        raise HTTPException(404, f"unknown drone: {drone}")
    return agents[drone]

def start_agents():
    for name, cfg in DRONES.items():
        agent = DroneAgent(name, cfg)
        agents[name] = agent
        threading.Thread(target=agent.run_forever, daemon=True).start()

if __name__ == "__main__":
    start_agents()
    uvicorn.run(app, host="0.0.0.0", port=8000)
