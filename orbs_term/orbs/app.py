import asyncio
import time

from rich.text import Text
from textual.app import App, ComposeResult
from textual.containers import Horizontal, Vertical
from textual.widgets import DataTable, Footer, Header, RichLog, Static

from .server import Server
from .state import MISSION_STATES, Fleet

COLUMNS = [
    ("", 2), ("drone", 16), ("link", 7), ("TARGET", 12), ("bearing", 9),
    ("state", 9), ("in", 6), ("range", 8), ("corners", 8), ("q", 6),
    ("fps", 6), ("cpu", 6), ("temp", 7), ("health", 20),
]

TARGET_STYLE = {
    "locked": ("● LOCKED", "bold white on dark_green"),
    "seen": ("◐ seen", "bold yellow"),
    "weak": ("◌ weak", "yellow"),
    "none": ("· searching", "dim"),
    "unknown": ("  -", "dim"),
}


def bearing_bar(angle, width=7):
    if angle is None or not isinstance(angle, (int, float)):
        return Text("   -   ", style="dim")
    half = width // 2
    slot = int(round(max(-1.0, min(1.0, angle / 12.0)) * half))
    cells = ["·"] * width
    cells[half + slot] = "▲"
    text = Text("".join(cells), style="bold")
    text.append(f"{angle:+5.1f}", style="dim")
    return text


def fmt(value, spec="{:.2f}", dash="-"):
    if value is None or value == "":
        return dash
    if isinstance(value, (int, float)):
        return spec.format(value)
    return str(value)


class FleetTable(DataTable):
    def on_mount(self):
        self.cursor_type = "row"
        self.zebra_stripes = True
        for label, width in COLUMNS:
            self.add_column(label, width=width, key=label or "dot")

    def sync(self, fleet):
        seen = set()
        for drone in fleet.order():
            seen.add(drone.id)
            values = self.row_for(drone)
            if drone.id in self.rows:
                for (label, _), value in zip(COLUMNS, values):
                    self.update_cell(drone.id, label or "dot", value,
                                     update_width=False)
            else:
                self.add_row(*values, key=drone.id)
        for key in [k for k in list(self.rows) if k.value not in seen]:
            self.remove_row(key)

    def row_for(self, drone):
        state = drone.state
        vision = drone.vision
        system = drone.system
        link_style = {"live": "green", "stale": "yellow",
                      "offline": "red"}[drone.link]
        problems = drone.health()
        label, style = TARGET_STYLE[drone.target]
        seeing = drone.target in ("locked", "seen", "weak")
        return [
            Text(drone.marker, style=f"bold {drone.color}"),
            Text(drone.tag, style=drone.color),
            Text(drone.link, style=link_style),
            Text(label, style=style),
            bearing_bar(state.get("ang_x")) if seeing else Text("", style="dim"),
            Text(str(state.get("state", "-")),
                 style="bold" if drone.link == "live" else ""),
            fmt(state.get("state_t"), "{:.0f}s"),
            Text(fmt(state.get("range_m"), "{:.2f}m"),
                 style="bold white" if seeing else "dim"),
            fmt(state.get("n_corners"), "{:.0f}"),
            fmt(state.get("quality"), "{:.2f}"),
            fmt(vision.get("fps"), "{:.1f}"),
            fmt(system.get("cpu_pct"), "{:.0f}%"),
            fmt(system.get("cpu_temp_c"), "{:.0f}C"),
            Text(" ".join(problems) if problems else "ok",
                 style="red bold" if problems else "green"),
        ]


class Detail(Static):
    def render_drone(self, drone):
        if drone is None:
            return Text("no drone selected", style="dim")
        body = Text()
        body.append(f"{drone.marker} {drone.tag}\n",
                    style=f"bold {drone.color}")
        body.append(f"{drone.remote or '-'}   {drone.messages} msgs   "
                    f"{drone.bytes_in / 1024:.0f} KB in   "
                    f"last {drone.age:.1f}s ago\n\n", style="dim")

        label, style = TARGET_STYLE[drone.target]
        held = time.time() - drone.target_since
        body.append(f"  {label:<12}", style=style)
        if drone.target in ("locked", "seen", "weak"):
            body.append(f" for {held:.0f}s")
            body.append("   bearing ")
            body.append_text(bearing_bar(drone.state.get("ang_x"), 11))
            body.append(f"   range {fmt(drone.state.get('range_m'), '{:.2f}')} m")
        else:
            body.append(f" for {held:.0f}s   "
                        f"{drone.acquisitions} acquisitions this session",
                        style="dim")
        body.append("\n\n")

        index = drone.mission_index()
        for i, name in enumerate(MISSION_STATES):
            if i == index:
                body.append(f" [{name}] ", style=f"bold black on {drone.color}")
            elif i < index:
                body.append(f"  {name}  ", style="dim")
            else:
                body.append(f"  {name}  ", style="dim white")
        body.append("\n\n")

        state = drone.state
        pairs = [
            ("cmd", state.get("cmd")), ("vx", fmt(state.get("vx"), "{:.2f}")),
            ("yaw", fmt(state.get("yaw_rate"), "{:.1f}")),
            ("ang_x", fmt(state.get("ang_x"), "{:+.1f}")),
            ("range", fmt(state.get("range_m"), "{:.2f}m")),
            ("margin", fmt(state.get("margin_m"), "{:.2f}m")),
            ("gap", fmt(state.get("gap_m"), "{:.2f}m")),
            ("v_safe", fmt(state.get("v_safe"), "{:.2f}")),
            ("weak", state.get("weak")), ("alt", fmt(state.get("alt_agl"), "{:.1f}m")),
        ]
        for i, (label, value) in enumerate(pairs):
            body.append(f"{label:>7} ", style="dim")
            body.append(f"{value!s:<9}")
            if i % 3 == 2:
                body.append("\n")
        body.append("\n")

        if drone.archive is not None:
            transfer = drone.archive
            filled = int(transfer.fraction * 30)
            body.append("\n archive ", style="bold")
            body.append("#" * filled + "." * (30 - filled),
                        style=drone.color)
            body.append(f" {transfer.fraction * 100:5.1f}%  "
                        f"{transfer.rate_mbit:.2f} Mbit/s\n")
            body.append(f"         {transfer.name}\n", style="dim")
        for done in drone.archives[-3:]:
            body.append(f" saved   {done['name']}  "
                        f"{done['size'] / 1e6:.1f} MB  "
                        f"{done['mbit_s']:.2f} Mbit/s\n", style="green")
        return body


class Ground(App):
    CSS = """
    Screen { layout: vertical; }
    #table { height: 40%; border: round $primary; }
    #lower { height: 60%; }
    #detail { width: 55%; border: round $accent; padding: 0 1; }
    #log { width: 45%; border: round $secondary; }
    """
    BINDINGS = [
        ("q", "quit", "quit"),
        ("c", "clear_log", "clear log"),
        ("f", "follow", "follow selected"),
    ]

    def __init__(self, host, port, inbox):
        super().__init__()
        self.fleet = Fleet()
        self.server = Server(self.fleet, host, port, inbox)
        self.host = host
        self.port = port
        self.inbox = inbox
        self.follow_only = False
        self.seen_log = 0

    def compose(self) -> ComposeResult:
        yield Header(show_clock=True)
        yield FleetTable(id="table")
        with Horizontal(id="lower"):
            with Vertical(id="detail"):
                yield Detail(id="detailbody")
            yield RichLog(id="log", highlight=False, markup=False,
                          wrap=True, max_lines=2000)
        yield Footer()

    async def on_mount(self):
        self.title = "ORBS ground station"
        self.sub_title = f"ws://{self.host}:{self.port}  ->  {self.inbox}"
        self.log_widget = self.query_one("#log", RichLog)
        self.log_widget.write(
            Text(f"listening on ws://{self.host}:{self.port}", style="bold"))
        self.serve_task = asyncio.create_task(self.serve())
        self.set_interval(0.25, self.tick)

    async def serve(self):
        try:
            await self.server.run()
        except Exception as error:
            self.log_widget.write(
                Text(f"server failed: {error}", style="red bold"))

    def selected(self):
        table = self.query_one("#table", FleetTable)
        order = self.fleet.order()
        if not order:
            return None
        index = min(max(table.cursor_row, 0), len(order) - 1)
        return order[index]

    def tick(self):
        self.fleet.prune()
        self.fleet.poll_targets()
        table = self.query_one("#table", FleetTable)
        table.sync(self.fleet)
        drone = self.selected()
        self.query_one("#detailbody", Detail).update(
            self.query_one("#detailbody", Detail).render_drone(drone))
        self.drain_log(drone)

    def drain_log(self, drone):
        entries = list(self.fleet.log)
        new = entries[self.seen_log:]
        self.seen_log = len(entries)
        for stamp, drone_id, source, text in new:
            owner = self.fleet.get(drone_id)
            if self.follow_only and drone and owner is not drone:
                continue
            line = Text()
            line.append(time.strftime("%H:%M:%S", time.localtime(stamp)) + " ",
                        style="dim")
            if owner is not None:
                line.append(f"{owner.marker} {owner.tag:<14}",
                            style=owner.color)
            else:
                line.append(f"{'-':<16}")
            line.append(f"{source:<8}", style="dim")
            if source == "target":
                line.append(text, style="bold green" if "TARGET" in text
                            else "yellow")
            else:
                line.append(text)
            self.log_widget.write(line)

    def action_clear_log(self):
        self.log_widget.clear()

    def action_follow(self):
        self.follow_only = not self.follow_only
        self.notify(f"log follow {'on' if self.follow_only else 'off'}")
