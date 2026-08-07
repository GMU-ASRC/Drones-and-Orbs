#!/usr/bin/env python3
import os
import re
import shutil
import subprocess
import threading
import time


SAMPLE_S       = 1.0
VCGENCMD_EVERY = 2
PING_S         = 5.0
PING_HOST      = None
PING_TIMEOUT_S = 2

WIFI_IFACE     = None


LAG_WARN_S     = 0.75
TEMP_WARN_C    = 75.0
MEM_WARN_MB    = 40.0
WIFI_WARN_DBM  = -75.0


SYS_FIELDS = [
    "t_mono", "t_wall", "sched_lag_s",
    "cpu_pct", "cpu0_pct", "cpu1_pct", "cpu2_pct", "cpu3_pct",
    "load1", "load5", "cpu_freq_mhz", "cpu_temp_c",
    "mem_avail_mb", "mem_free_mb", "swap_used_mb",
    "throttled_hex", "uv_now", "freq_capped_now", "throttled_now",
    "uv_occurred", "throttle_occurred", "core_volt_v",
    "link_up", "wifi_link", "wifi_level_dbm", "wifi_noise",
    "rx_bps", "tx_bps", "rx_drop", "tx_drop", "rx_err", "tx_err",
    "ping_rtt_ms", "ping_loss_pct",
    "disk_free_mb", "proc_cpu_pct", "proc_rss_mb", "proc_threads",
]


def _read(path, default=""):
    try:
        with open(path) as f:
            return f.read()
    except Exception:
        return default


def _read_int(path, default=-1):
    try:
        return int(_read(path).strip())
    except Exception:
        return default


def _default_gateway():
    try:
        for line in _read("/proc/net/route").splitlines()[1:]:
            f = line.split()
            if len(f) > 2 and f[1] == "00000000":
                g = int(f[2], 16)
                return ".".join(str((g >> (8 * i)) & 0xFF) for i in range(4))
    except Exception:
        pass
    return None


def _wifi_iface():
    try:
        for line in _read("/proc/net/wireless").splitlines()[2:]:
            if ":" in line:
                return line.split(":")[0].strip()
    except Exception:
        pass
    return None


def device_info() -> dict:
    info = {
        "hostname": (_read("/proc/sys/kernel/hostname").strip() or "?"),
        "model": _read("/proc/device-tree/model", "?").strip("\x00").strip(),
        "kernel": _read("/proc/version").strip()[:120],
        "cpu_count": os.cpu_count(),
        "wifi_iface": _wifi_iface(),
        "gateway": _default_gateway(),
    }
    try:
        info["os"] = re.search(r'PRETTY_NAME="([^"]+)"',
                               _read("/etc/os-release")).group(1)
    except Exception:
        info["os"] = "?"
    try:
        info["mem_total_mb"] = round(
            int(re.search(r"MemTotal:\s+(\d+)", _read("/proc/meminfo"))
                .group(1)) / 1024.0, 1)
    except Exception:
        pass
    for k, p in (("cpu_governor",
                  "/sys/devices/system/cpu/cpu0/cpufreq/scaling_governor"),
                 ("cpu_max_mhz",
                  "/sys/devices/system/cpu/cpu0/cpufreq/cpuinfo_max_freq")):
        v = _read(p).strip()
        if v:
            info[k] = (round(int(v) / 1000.0) if v.isdigit() else v)
    return info


class SystemMonitor:
    def __init__(self, logger, sample_s: float = SAMPLE_S):
        self._log = logger
        self._sample_s = sample_s
        self._running = False
        self._threads = []
        self._csv = None

        self._vc = shutil.which("vcgencmd")
        self._iface = WIFI_IFACE or _wifi_iface()
        self._ping_host = PING_HOST or _default_gateway()


        self._cpu_prev = None
        self._proc_prev = None
        self._net_prev = None
        self._n = 0


        self._vc_cache = {}
        self._ping = (-1.0, 100.0)


        self._alerted = set()


    def start(self):
        if self._running:
            return
        self._csv = self._log.csv("system", SYS_FIELDS)
        self._log.add_meta("device", device_info())
        self._running = True
        self._spawn(self._stats_loop)
        if PING_S > 0 and self._ping_host:
            self._spawn(self._ping_loop)
            self._log.event("sysmon", f"pinging {self._ping_host} "
                                      f"every {PING_S:.0f}s")
        elif PING_S > 0:
            self._log.event("sysmon", "no default gateway -- ping probe off")
        if not self._vc:
            self._log.event("sysmon", "vcgencmd missing -- no throttle flags")
        if not self._iface:
            self._log.event("sysmon", "no wifi interface in /proc/net/wireless")

    def _spawn(self, fn):
        t = threading.Thread(target=fn, daemon=True)
        t.start()
        self._threads.append(t)

    def stop(self):
        self._running = False
        for t in self._threads:
            t.join(timeout=PING_TIMEOUT_S + 1.0)
        self._threads = []
        if self._csv:
            self._csv.flush()


    def _stats_loop(self):
        next_t = time.monotonic()
        while self._running:
            next_t += self._sample_s
            s = next_t - time.monotonic()
            if s > 0:
                time.sleep(s)
            else:
                next_t = time.monotonic()

            now = time.monotonic()

            lag = max(0.0, now - next_t)
            try:
                row = self._sample(now, lag)
                self._csv.write(**row)
                self._alerts(row)
            except Exception as e:
                self._log.event("sysmon", f"sample failed: {e}")
            self._n += 1

    def _ping_loop(self):
        while self._running:
            rtt, loss = -1.0, 100.0
            try:
                out = subprocess.run(
                    ["ping", "-n", "-c", "1", "-W", str(PING_TIMEOUT_S),
                     self._ping_host],
                    stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
                    timeout=PING_TIMEOUT_S + 2).stdout.decode("ascii", "ignore")
                m = re.search(r"time=([\d.]+)\s*ms", out)
                if m:
                    rtt, loss = float(m.group(1)), 0.0
            except Exception:
                pass
            self._ping = (rtt, loss)
            if loss > 0 and "ping" not in self._alerted:
                self._alerted.add("ping")
                self._log.event("sysmon",
                                f"WARN gateway {self._ping_host} unreachable")
            elif loss == 0 and "ping" in self._alerted:
                self._alerted.discard("ping")
                self._log.event("sysmon", f"gateway reachable again "
                                          f"({rtt:.0f} ms)")
            for _ in range(int(PING_S * 4)):
                if not self._running:
                    return
                time.sleep(0.25)


    def _sample(self, now, lag) -> dict:
        row = {"t_mono": round(now, 3), "t_wall": round(time.time(), 3),
               "sched_lag_s": round(lag, 3)}
        row.update(self._cpu())
        row.update(self._mem())
        row.update(self._thermal())
        row.update(self._net())
        row.update(self._proc())

        if self._vc and self._n % VCGENCMD_EVERY == 0:
            self._vc_cache = self._vcgencmd()
        row.update(self._vc_cache)

        rtt, loss = self._ping
        row["ping_rtt_ms"] = rtt
        row["ping_loss_pct"] = loss

        try:
            st = os.statvfs(self._log.dir or "/")
            row["disk_free_mb"] = round(st.f_bavail * st.f_frsize / 1e6, 1)
        except Exception:
            pass
        return row

    def _cpu(self):
        out, cur = {}, {}
        try:
            for line in _read("/proc/stat").splitlines():
                if not line.startswith("cpu"):
                    break
                f = line.split()
                name = f[0]
                vals = [int(x) for x in f[1:11]]
                cur[name] = (sum(vals), vals[3] + vals[4])
            if self._cpu_prev:
                for name, (tot, idle) in cur.items():
                    ptot, pidle = self._cpu_prev.get(name, (tot, idle))
                    dt, di = tot - ptot, idle - pidle
                    pct = round(100.0 * (dt - di) / dt, 1) if dt > 0 else 0.0
                    out["cpu_pct" if name == "cpu" else f"{name}_pct"] = pct
            self._cpu_prev = cur
        except Exception:
            pass
        try:
            la = _read("/proc/loadavg").split()
            out["load1"], out["load5"] = float(la[0]), float(la[1])
        except Exception:
            pass
        khz = _read_int(
            "/sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq")
        if khz > 0:
            out["cpu_freq_mhz"] = round(khz / 1000.0)
        return out

    def _mem(self):
        out = {}
        try:
            mi = dict(re.findall(r"(\w+):\s+(\d+) kB", _read("/proc/meminfo")))
            out["mem_avail_mb"] = round(int(mi["MemAvailable"]) / 1024.0, 1)
            out["mem_free_mb"] = round(int(mi["MemFree"]) / 1024.0, 1)
            out["swap_used_mb"] = round(
                (int(mi["SwapTotal"]) - int(mi["SwapFree"])) / 1024.0, 1)
        except Exception:
            pass
        return out

    def _thermal(self):
        t = _read_int("/sys/class/thermal/thermal_zone0/temp")
        return {"cpu_temp_c": round(t / 1000.0, 1)} if t > 0 else {}

    def _vcgencmd(self):
        out = {}
        try:
            r = subprocess.run([self._vc, "get_throttled"],
                               stdout=subprocess.PIPE,
                               stderr=subprocess.DEVNULL, timeout=2)
            bits = int(r.stdout.decode().strip().split("=")[1], 16)
            out["throttled_hex"] = f"0x{bits:x}"
            out["uv_now"] = int(bool(bits & 0x1))
            out["freq_capped_now"] = int(bool(bits & 0x2))
            out["throttled_now"] = int(bool(bits & 0x4))
            out["uv_occurred"] = int(bool(bits & 0x10000))
            out["throttle_occurred"] = int(bool(bits & 0x40000))
        except Exception:
            pass
        try:
            r = subprocess.run([self._vc, "measure_volts", "core"],
                               stdout=subprocess.PIPE,
                               stderr=subprocess.DEVNULL, timeout=2)
            out["core_volt_v"] = float(
                r.stdout.decode().strip().split("=")[1].rstrip("V"))
        except Exception:
            pass
        return out

    def _net(self):
        out = {}
        if not self._iface:
            return out

        try:
            for line in _read("/proc/net/wireless").splitlines()[2:]:
                if line.strip().startswith(self._iface + ":"):
                    f = line.split()
                    out["wifi_link"] = float(f[2].rstrip("."))
                    out["wifi_level_dbm"] = float(f[3].rstrip("."))
                    out["wifi_noise"] = float(f[4].rstrip("."))
                    break
        except Exception:
            pass
        out["link_up"] = int(
            _read(f"/sys/class/net/{self._iface}/operstate").strip() == "up")

        try:
            for line in _read("/proc/net/dev").splitlines()[2:]:
                name, _, rest = line.partition(":")
                if name.strip() != self._iface:
                    continue
                f = [int(x) for x in rest.split()]
                cur = (time.monotonic(), f[0], f[8], f[2], f[10], f[3], f[11])
                if self._net_prev:
                    dt = cur[0] - self._net_prev[0]
                    if dt > 0:
                        out["rx_bps"] = round((cur[1] - self._net_prev[1]) / dt)
                        out["tx_bps"] = round((cur[2] - self._net_prev[2]) / dt)
                    out["rx_err"] = cur[3] - self._net_prev[3]
                    out["tx_err"] = cur[4] - self._net_prev[4]
                    out["rx_drop"] = cur[5] - self._net_prev[5]
                    out["tx_drop"] = cur[6] - self._net_prev[6]
                self._net_prev = cur
                break
        except Exception:
            pass
        return out

    def _proc(self):
        out = {}
        try:
            f = _read("/proc/self/stat").rsplit(") ", 1)[1].split()
            jiff = (int(f[11]) + int(f[12]))
            now = time.monotonic()
            if self._proc_prev:
                pj, pt = self._proc_prev
                dt = now - pt
                if dt > 0:
                    hz = os.sysconf("SC_CLK_TCK")
                    out["proc_cpu_pct"] = round(
                        100.0 * (jiff - pj) / hz / dt, 1)
            self._proc_prev = (jiff, now)
            out["proc_threads"] = int(f[17])
        except Exception:
            pass
        try:
            m = re.search(r"VmRSS:\s+(\d+) kB", _read("/proc/self/status"))
            out["proc_rss_mb"] = round(int(m.group(1)) / 1024.0, 1)
        except Exception:
            pass
        return out


    def _alerts(self, row):
        def edge(key, bad, msg_bad, msg_ok=None):
            if bad and key not in self._alerted:
                self._alerted.add(key)
                self._log.event("sysmon", "WARN " + msg_bad)
            elif not bad and key in self._alerted:
                self._alerted.discard(key)
                if msg_ok:
                    self._log.event("sysmon", msg_ok)

        if row.get("sched_lag_s", 0) > LAG_WARN_S:

            self._log.event("sysmon", f"WARN scheduler lag "
                                      f"{row['sched_lag_s']:.2f}s -- Pi stalled")
        edge("uv", row.get("uv_now") == 1,
             "UNDER-VOLTAGE now (5V rail sagging -- check telemetry-port power)",
             "under-voltage cleared")
        edge("thr", row.get("throttled_now") == 1,
             "CPU throttled now", "throttling cleared")
        edge("temp", (row.get("cpu_temp_c") or 0) > TEMP_WARN_C,
             f"CPU {row.get('cpu_temp_c')}C over {TEMP_WARN_C}C",
             "CPU temp back to normal")
        edge("mem", 0 < (row.get("mem_avail_mb") or 1e9) < MEM_WARN_MB,
             f"only {row.get('mem_avail_mb')}MB RAM available",
             "memory pressure eased")
        edge("linkdown", row.get("link_up") == 0,
             f"{self._iface} is DOWN", f"{self._iface} back up")
        lvl = row.get("wifi_level_dbm")
        edge("rssi", lvl is not None and lvl < WIFI_WARN_DBM,
             f"weak wifi {lvl} dBm", "wifi signal recovered")
