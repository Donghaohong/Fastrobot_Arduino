#!/usr/bin/env python3
"""Lab12 local planning runner for the real BLE robot.

Default mode is a desktop check: parse world.yaml, build the inflated 12x9
grid, run A*, and print the planned route from (5, -3) to (5, 3).

Use --run-robot only when the car is on the Lab map and the Lab12 firmware has
been flashed. The robot path executor performs one neighbor cell at a time,
then runs a 360 degree scan and applies the Lab11 localization update.
"""

from __future__ import annotations

import argparse
import ast
import heapq
import importlib.util
import json
import math
import os
import sys
import time
import types
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Callable, Iterable, Sequence

import numpy as np
import yaml

from cmd_types import CMD


FT_TO_M = 0.3048
DEFAULT_START = (5, -3)
DEFAULT_GOAL = (5, 3)

BLE_PYTHON_DIR = Path(__file__).resolve().parent
REPO_ROOT = BLE_PYTHON_DIR.parent
SIM_ROOT = REPO_ROOT / "FastRobots-sim-release-main"
WORLD_YAML = SIM_ROOT / "config" / "world.yaml"

Cell = tuple[int, int]
Segment = tuple[tuple[float, float], tuple[float, float]]


def wrap_deg(angle_deg: float) -> float:
    while angle_deg < -180.0:
        angle_deg += 360.0
    while angle_deg >= 180.0:
        angle_deg -= 360.0
    return angle_deg


def parse_world_yaml(path: Path = WORLD_YAML) -> dict:
    with path.open("r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def parse_world_segments(config: dict) -> list[Segment]:
    segments: list[Segment] = []
    for raw in config["world"]["lines"]:
        start, end = ast.literal_eval(raw)
        segments.append(((float(start[0]), float(start[1])),
                         (float(end[0]), float(end[1]))))
    return segments


def meters_to_feet_segment(segment: Segment) -> Segment:
    (x0, y0), (x1, y1) = segment
    return ((x0 / FT_TO_M, y0 / FT_TO_M),
            (x1 / FT_TO_M, y1 / FT_TO_M))


def point_in_polygon(point: tuple[float, float],
                     polygon: Sequence[tuple[float, float]]) -> bool:
    x, y = point
    inside = False
    j = len(polygon) - 1

    for i in range(len(polygon)):
        xi, yi = polygon[i]
        xj, yj = polygon[j]

        intersects = ((yi > y) != (yj > y))
        if intersects:
            x_at_y = (xj - xi) * (y - yi) / (yj - yi) + xi
            if x < x_at_y:
                inside = not inside
        j = i

    return inside


def point_segment_distance(point: tuple[float, float],
                           segment: Segment) -> float:
    px, py = point
    (x0, y0), (x1, y1) = segment
    dx = x1 - x0
    dy = y1 - y0
    denom = dx * dx + dy * dy

    if denom == 0.0:
        return math.hypot(px - x0, py - y0)

    t = ((px - x0) * dx + (py - y0) * dy) / denom
    t = max(0.0, min(1.0, t))
    closest_x = x0 + t * dx
    closest_y = y0 + t * dy
    return math.hypot(px - closest_x, py - closest_y)


class Lab12GridPlanner:
    """A 4-connected 1 ft planner over the Lab11 map grid."""

    def __init__(self,
                 world_yaml: Path = WORLD_YAML,
                 wall_inflation_ft: float = 0.5) -> None:
        self.world_yaml = world_yaml
        self.wall_inflation_ft = wall_inflation_ft
        self.config = parse_world_yaml(world_yaml)
        mapper_cfg = self.config["robot"]["mapper"]

        self.max_cells_x = int(mapper_cfg["max_cells_x"])
        self.max_cells_y = int(mapper_cfg["max_cells_y"])
        self.min_x_ft = float(mapper_cfg["min_x"]) / FT_TO_M
        self.min_y_ft = float(mapper_cfg["min_y"]) / FT_TO_M
        self.cell_size_ft = float(mapper_cfg["cell_size_x"]) / FT_TO_M

        if not math.isclose(self.cell_size_ft, 1.0, abs_tol=1e-6):
            raise ValueError(f"Expected a 1 ft grid, got {self.cell_size_ft:.3f} ft")

        self.x_cells = [
            int(round(self.min_x_ft + (i + 0.5) * self.cell_size_ft))
            for i in range(self.max_cells_x)
        ]
        self.y_cells = [
            int(round(self.min_y_ft + (i + 0.5) * self.cell_size_ft))
            for i in range(self.max_cells_y)
        ]

        self.segments_m = parse_world_segments(self.config)
        self.segments_ft = [meters_to_feet_segment(s) for s in self.segments_m]

        self.outer_polygon_ft = self._build_outer_polygon()
        self.internal_segments_ft = self.segments_ft[len(self.outer_polygon_ft):]
        self.blocked: set[Cell] = set()
        self._build_occupancy()

    def _build_outer_polygon(self) -> list[tuple[float, float]]:
        # The current course map lists the outer concave boundary first.
        outer_segment_count = 6
        polygon = [self.segments_ft[0][0]]
        polygon.extend(seg[1] for seg in self.segments_ft[:outer_segment_count])
        if polygon[-1] == polygon[0]:
            polygon.pop()
        return polygon

    def _build_occupancy(self) -> None:
        for cell in self.all_cells():
            point = (float(cell[0]), float(cell[1]))

            if not point_in_polygon(point, self.outer_polygon_ft):
                self.blocked.add(cell)
                continue

            for segment in self.internal_segments_ft:
                if point_segment_distance(point, segment) <= self.wall_inflation_ft + 1e-9:
                    self.blocked.add(cell)
                    break

    def all_cells(self) -> Iterable[Cell]:
        for x in self.x_cells:
            for y in self.y_cells:
                yield (x, y)

    def in_bounds(self, cell: Cell) -> bool:
        return cell[0] in self.x_cells and cell[1] in self.y_cells

    def is_free(self, cell: Cell) -> bool:
        return self.in_bounds(cell) and cell not in self.blocked

    def neighbors(self, cell: Cell) -> list[Cell]:
        x, y = cell
        candidates = [(x + 1, y), (x - 1, y), (x, y + 1), (x, y - 1)]
        return [c for c in candidates if self.is_free(c)]

    @staticmethod
    def heuristic(a: Cell, b: Cell) -> int:
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    @staticmethod
    def goal_reached(cell: Cell, goal: Cell) -> bool:
        return max(abs(cell[0] - goal[0]), abs(cell[1] - goal[1])) <= 1

    def astar(self, start: Cell, goal: Cell) -> list[Cell]:
        if not self.is_free(start):
            raise ValueError(f"Start cell {start} is blocked or outside the map")
        if not self.is_free(goal):
            raise ValueError(f"Goal cell {goal} is blocked or outside the map")

        frontier: list[tuple[int, int, Cell]] = []
        heapq.heappush(frontier, (0, 0, start))
        came_from: dict[Cell, Cell | None] = {start: None}
        cost_so_far: dict[Cell, int] = {start: 0}
        tie = 0

        while frontier:
            _, _, current = heapq.heappop(frontier)

            if current == goal:
                break

            for nxt in self.neighbors(current):
                new_cost = cost_so_far[current] + 1
                if nxt not in cost_so_far or new_cost < cost_so_far[nxt]:
                    cost_so_far[nxt] = new_cost
                    priority = new_cost + self.heuristic(nxt, goal)
                    tie += 1
                    heapq.heappush(frontier, (priority, tie, nxt))
                    came_from[nxt] = current

        if goal not in came_from:
            raise RuntimeError(f"No free-grid route from {start} to {goal}")

        route: list[Cell] = []
        cur: Cell | None = goal
        while cur is not None:
            route.append(cur)
            cur = came_from[cur]
        route.reverse()
        return route

    def render_ascii(self,
                     route: Sequence[Cell] | None = None,
                     start: Cell | None = None,
                     goal: Cell | None = None) -> str:
        route_set = set(route or [])
        rows: list[str] = []
        header = "     " + " ".join(f"{x:>2d}" for x in self.x_cells)
        rows.append(header)

        for y in reversed(self.y_cells):
            chars: list[str] = []
            for x in self.x_cells:
                cell = (x, y)
                if cell == start:
                    chars.append(" S")
                elif cell == goal:
                    chars.append(" G")
                elif cell in route_set:
                    chars.append(" *")
                elif cell in self.blocked:
                    chars.append(" #")
                else:
                    chars.append(" .")
            rows.append(f"y={y:>2d} " + " ".join(chars))

        return "\n".join(rows)

    def plot(self,
             route: Sequence[Cell],
             start: Cell,
             goal: Cell,
             output_path: Path) -> Path:
        try:
            import matplotlib.pyplot as plt
            from matplotlib.patches import Rectangle
        except ModuleNotFoundError:
            svg_path = output_path if output_path.suffix.lower() == ".svg" else output_path.with_suffix(".svg")
            self._write_svg_plot(route, start, goal, svg_path)
            return svg_path

        fig, ax = plt.subplots(figsize=(9, 6))

        for cell in self.all_cells():
            color = "#222222" if cell in self.blocked else "#f7f7f7"
            ax.add_patch(Rectangle((cell[0] - 0.5, cell[1] - 0.5),
                                   1.0,
                                   1.0,
                                   facecolor=color,
                                   edgecolor="#c8c8c8",
                                   linewidth=0.8))

        for segment in self.segments_ft:
            (x0, y0), (x1, y1) = segment
            ax.plot([x0, x1], [y0, y1], color="black", linewidth=2)

        if route:
            xs = [c[0] for c in route]
            ys = [c[1] for c in route]
            ax.plot(xs, ys, color="#0077bb", linewidth=2.5, marker="o")

        ax.scatter([start[0]], [start[1]], marker="s", s=110, color="#009e73", label="start")
        ax.scatter([goal[0]], [goal[1]], marker="*", s=170, color="#d55e00", label="goal")
        ax.set_xticks(self.x_cells)
        ax.set_yticks(self.y_cells)
        ax.set_aspect("equal", adjustable="box")
        ax.set_xlabel("x cell (ft)")
        ax.set_ylabel("y cell (ft)")
        ax.set_title(f"Lab12 inflated grid route, inflation={self.wall_inflation_ft:.2f} ft")
        ax.grid(False)
        ax.legend(loc="upper right")
        output_path.parent.mkdir(parents=True, exist_ok=True)
        fig.tight_layout()
        fig.savefig(output_path, dpi=160)
        plt.close(fig)
        return output_path

    def _write_svg_plot(self,
                        route: Sequence[Cell],
                        start: Cell,
                        goal: Cell,
                        output_path: Path) -> None:
        scale = 44
        margin = 54
        x_min = min(self.x_cells) - 1
        x_max = max(self.x_cells) + 1
        y_min = min(self.y_cells) - 1
        y_max = max(self.y_cells) + 1
        width = int((x_max - x_min) * scale + 2 * margin)
        height = int((y_max - y_min) * scale + 2 * margin)

        def sx(x: float) -> float:
            return margin + (x - x_min) * scale

        def sy(y: float) -> float:
            return margin + (y_max - y) * scale

        route_points = " ".join(f"{sx(x):.1f},{sy(y):.1f}" for x, y in route)
        lines: list[str] = [
            f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" '
            f'viewBox="0 0 {width} {height}">',
            '<rect width="100%" height="100%" fill="white"/>',
            f'<text x="{margin}" y="28" font-family="Arial" font-size="18">'
            f'Lab12 inflated grid route, inflation={self.wall_inflation_ft:.2f} ft</text>',
        ]

        for cell in self.all_cells():
            fill = "#222222" if cell in self.blocked else "#f7f7f7"
            x = sx(cell[0] - 0.5)
            y = sy(cell[1] + 0.5)
            lines.append(
                f'<rect x="{x:.1f}" y="{y:.1f}" width="{scale}" height="{scale}" '
                f'fill="{fill}" stroke="#c8c8c8" stroke-width="1"/>'
            )

        for segment in self.segments_ft:
            (x0, y0), (x1, y1) = segment
            lines.append(
                f'<line x1="{sx(x0):.1f}" y1="{sy(y0):.1f}" '
                f'x2="{sx(x1):.1f}" y2="{sy(y1):.1f}" '
                f'stroke="black" stroke-width="3" stroke-linecap="round"/>'
            )

        if route_points:
            lines.append(
                f'<polyline points="{route_points}" fill="none" stroke="#0077bb" '
                f'stroke-width="4" stroke-linejoin="round" stroke-linecap="round"/>'
            )
            for x, y in route:
                lines.append(
                    f'<circle cx="{sx(x):.1f}" cy="{sy(y):.1f}" r="5" fill="#0077bb"/>'
                )

        lines.append(
            f'<rect x="{sx(start[0]) - 8:.1f}" y="{sy(start[1]) - 8:.1f}" '
            f'width="16" height="16" fill="#009e73"/>'
        )
        lines.append(
            f'<text x="{sx(start[0]) + 10:.1f}" y="{sy(start[1]) - 10:.1f}" '
            f'font-family="Arial" font-size="14">start</text>'
        )
        lines.append(
            f'<polygon points="{sx(goal[0]):.1f},{sy(goal[1]) - 12:.1f} '
            f'{sx(goal[0]) + 11:.1f},{sy(goal[1]) + 9:.1f} '
            f'{sx(goal[0]) - 11:.1f},{sy(goal[1]) + 9:.1f}" fill="#d55e00"/>'
        )
        lines.append(
            f'<text x="{sx(goal[0]) + 12:.1f}" y="{sy(goal[1]) - 10:.1f}" '
            f'font-family="Arial" font-size="14">goal</text>'
        )
        lines.append("</svg>")

        output_path.parent.mkdir(parents=True, exist_ok=True)
        output_path.write_text("\n".join(lines) + "\n", encoding="utf-8")


@dataclass
class LocalizationState:
    cell: Cell
    heading_deg: float
    confidence: float
    map_index: tuple[int, int, int]
    pose_m: tuple[float, float, float]


class _DummyCommander:
    def __getattr__(self, _name: str) -> Callable[..., None]:
        def noop(*_args, **_kwargs) -> None:
            return None

        return noop


class Lab12Localizer:
    """Thin wrapper around the Lab11 Localization.update_step()."""

    def __init__(self,
                 planner: Lab12GridPlanner,
                 sensor_sigma_m: float = 0.18) -> None:
        self.planner = planner
        self._prepare_sim_imports()

        from localization import Mapper
        from localization_extras import Localization
        from utils import load_config_params

        class RobotStub:
            pass

        self.robot = RobotStub()
        self.robot.config_params = load_config_params(str(planner.world_yaml))
        self.robot.cmdr = _DummyCommander()
        self.mapper = Mapper(self.robot)
        self.loc = Localization(self.robot, self.mapper)
        self.loc.sensor_sigma = sensor_sigma_m
        self.free_mask_xy = self._build_free_mask()
        self.set_uniform_prior()

    @staticmethod
    def _prepare_sim_imports() -> None:
        sim_root = str(SIM_ROOT)
        if sim_root not in sys.path:
            sys.path.insert(0, sim_root)

        if importlib.util.find_spec("colorama") is None and "colorama" not in sys.modules:
            colorama_stub = types.ModuleType("colorama")

            class _AnsiStub:
                def __getattr__(self, _name: str) -> str:
                    return ""

            colorama_stub.Fore = _AnsiStub()
            colorama_stub.Back = _AnsiStub()
            colorama_stub.Style = _AnsiStub()
            sys.modules["colorama"] = colorama_stub

        loaded_utils = sys.modules.get("utils")
        if loaded_utils is not None:
            loaded_path = Path(getattr(loaded_utils, "__file__", "")).resolve()
            if SIM_ROOT not in loaded_path.parents:
                del sys.modules["utils"]

    def _build_free_mask(self) -> np.ndarray:
        mask = np.zeros((self.mapper.MAX_CELLS_X, self.mapper.MAX_CELLS_Y), dtype=float)
        for cx in range(self.mapper.MAX_CELLS_X):
            for cy in range(self.mapper.MAX_CELLS_Y):
                x_m, y_m, _ = self.mapper.from_map(cx, cy, 0)
                cell = (int(round(x_m / FT_TO_M)), int(round(y_m / FT_TO_M)))
                if self.planner.is_free(cell):
                    mask[cx, cy] = 1.0
        return mask

    def _normalize_prior(self, prior: np.ndarray) -> np.ndarray:
        prior = prior * self.free_mask_xy[:, :, None]
        total = float(np.sum(prior))
        if total <= 0.0 or not np.isfinite(total):
            raise RuntimeError("Belief prior collapsed to zero probability")
        return prior / total

    def set_uniform_prior(self) -> None:
        prior = np.ones_like(self.loc.bel_bar, dtype=float)
        prior = self._normalize_prior(prior)
        self.loc.bel_bar = prior
        self.loc.bel = prior.copy()

    def set_motion_prior(self,
                         expected_cell: Cell,
                         expected_heading_deg: float,
                         cell_sigma_ft: float = 1.0,
                         heading_sigma_deg: float = 20.0) -> None:
        expected_x_m = expected_cell[0] * FT_TO_M
        expected_y_m = expected_cell[1] * FT_TO_M

        x_grid = self.mapper.x_values[:, :, 0]
        y_grid = self.mapper.y_values[:, :, 0]
        pos_prob = np.exp(
            -0.5 * (
                ((x_grid - expected_x_m) / (cell_sigma_ft * FT_TO_M)) ** 2
                + ((y_grid - expected_y_m) / (cell_sigma_ft * FT_TO_M)) ** 2
            )
        )

        angle_grid = self.mapper.a_values[0, 0, :]
        angle_err = np.array([wrap_deg(a - expected_heading_deg) for a in angle_grid])
        heading_prob = np.exp(-0.5 * (angle_err / heading_sigma_deg) ** 2)

        prior = pos_prob[:, :, None] * heading_prob[None, None, :]
        self.loc.bel_bar = self._normalize_prior(prior)

    def update_from_scan(self, distances_m: Sequence[float]) -> LocalizationState:
        obs = np.asarray(distances_m, dtype=float)
        if obs.shape[0] != self.mapper.OBS_PER_CELL:
            raise ValueError(f"Expected {self.mapper.OBS_PER_CELL} scan samples, got {obs.shape[0]}")

        sensor_range = float(self.robot.config_params["sensor_range"])
        obs = np.where(np.isfinite(obs) & (obs > 0.0), obs, sensor_range)
        obs = np.clip(obs, 0.05, sensor_range)

        prior_backup = self.loc.bel_bar.copy()
        self.loc.obs_range_data = obs[:, None]
        self.loc.update_step()

        bel = self.loc.bel * self.free_mask_xy[:, :, None]
        total = float(np.sum(bel))
        if total <= 0.0 or not np.isfinite(total):
            self.loc.bel_bar = prior_backup
            self.loc.bel = prior_backup.copy()
            raise RuntimeError("Localization update produced an invalid belief")
        self.loc.bel = bel / total

        max_index = tuple(int(i) for i in np.unravel_index(np.argmax(self.loc.bel), self.loc.bel.shape))
        confidence = float(self.loc.bel[max_index])
        pose = tuple(float(v) for v in self.mapper.from_map(*max_index))
        cell = (int(round(pose[0] / FT_TO_M)), int(round(pose[1] / FT_TO_M)))
        heading = wrap_deg(pose[2])

        return LocalizationState(
            cell=cell,
            heading_deg=heading,
            confidence=confidence,
            map_index=max_index,
            pose_m=pose,
        )


@dataclass
class NavStepLog:
    step: int
    planned_cell: Cell
    expected_cell: Cell
    localized_cell: Cell | None = None
    localized_heading_deg: float | None = None
    confidence: float | None = None
    turn_status: str | None = None
    drive_status: str | None = None
    turn_motion_log: list[list[str]] = field(default_factory=list)
    drive_motion_log: list[list[str]] = field(default_factory=list)
    tof_safety_stop: bool = False
    elapsed_s: float = 0.0
    route: list[Cell] = field(default_factory=list)


class Lab12Navigator:
    def __init__(self) -> None:
        if str(BLE_PYTHON_DIR) not in sys.path:
            sys.path.insert(0, str(BLE_PYTHON_DIR))

        from ble import ArtemisBLEController

        self.ble = ArtemisBLEController(config=str(BLE_PYTHON_DIR / "connection.yaml"))
        self.messages: list[str] = []
        self.map_samples_mm: dict[int, float] = {}
        self.nav_rows: list[list[str]] = []

    def connect(self) -> None:
        self.ble.connect()

    def disconnect(self) -> None:
        self.ble.disconnect()

    def start_notify(self) -> None:
        self.ble.start_notify(self.ble.uuid["RX_STRING"], self._handle_notification)

    def stop_notify(self) -> None:
        self.ble.stop_notify(self.ble.uuid["RX_STRING"])

    def _handle_notification(self, _uuid, byte_array) -> None:
        msg = byte_array.decode(errors="replace").strip("\x00\r\n ")
        if not msg:
            return
        self.messages.append(msg)
        self._parse_message(msg)

    def _parse_message(self, msg: str) -> None:
        if msg.startswith("MAP,"):
            parts = msg.split(",")
            if len(parts) >= 5:
                self.map_samples_mm[int(parts[1])] = float(parts[4])
        elif msg.startswith("MAP_SAMPLE,"):
            parts = msg.split(",")
            if len(parts) >= 4:
                self.map_samples_mm[int(parts[1])] = float(parts[3])
        elif msg.startswith("NAV,"):
            self.nav_rows.append(msg.split(","))

    def _wait_for_prefix(self,
                         prefixes: Sequence[str],
                         timeout_s: float,
                         start_index: int = 0) -> str:
        deadline = time.time() + timeout_s
        checked = start_index

        while time.time() < deadline:
            while checked < len(self.messages):
                msg = self.messages[checked]
                checked += 1
                if any(msg.startswith(prefix) for prefix in prefixes):
                    return msg
            self.ble.sleep(0.05)

        raise TimeoutError(f"Timed out waiting for {prefixes}")

    def run_map_scan(self,
                     step_deg: float,
                     num_steps: int,
                     kp: float,
                     ki: float,
                     kd: float,
                     settle_ms: int,
                     timeout_s: float) -> np.ndarray:
        self.map_samples_mm.clear()
        start_idx = len(self.messages)
        payload = f"{step_deg}|{num_steps}|{kp}|{ki}|{kd}|{settle_ms}"
        self.ble.send_command(CMD.START_MAP_SCAN, payload)

        done = self._wait_for_prefix(
            ("MAP_SCAN_DONE", "MAP_SCAN_YAW_INVALID", "MAP_SCAN_BAD_ARGS"),
            timeout_s,
            start_idx,
        )
        if not done.startswith("MAP_SCAN_DONE"):
            raise RuntimeError(f"Map scan failed: {done}")

        self.map_samples_mm.clear()
        start_idx = len(self.messages)
        self.ble.send_command(CMD.SEND_MAP_SCAN, "")
        self._wait_for_prefix(("MAP_END",), 10.0, start_idx)

        if len(self.map_samples_mm) < num_steps:
            raise RuntimeError(f"Expected {num_steps} map samples, got {len(self.map_samples_mm)}")

        samples_cw_mm = [self.map_samples_mm[i] for i in range(num_steps)]
        # Arduino scans clockwise: index 1 is -20 deg relative to the start yaw.
        # Lab11 mapper expects counter-clockwise observations: 0, +20, +40, ...
        samples_ccw_mm = [samples_cw_mm[(num_steps - i) % num_steps] for i in range(num_steps)]
        return np.asarray(samples_ccw_mm, dtype=float) / 1000.0

    def turn(self,
             delta_deg: float,
             timeout_ms: int,
             kp: float,
             ki: float,
             kd: float) -> str:
        start_idx = len(self.messages)
        payload = f"{delta_deg}|{timeout_ms}|{kp}|{ki}|{kd}"
        self.ble.send_command(CMD.TURN_REL_DEG, payload)
        return self._wait_for_prefix(("TURN_DONE", "TURN_FAILED"),
                                     timeout_ms / 1000.0 + 1.5,
                                     start_idx)

    def drive_cell(self,
                   distance_mm: float,
                   base_pwm: int,
                   duration_ms: int,
                   heading_kp: float,
                   front_stop_mm: int) -> str:
        start_idx = len(self.messages)
        payload = f"{distance_mm}|{base_pwm}|{duration_ms}|{heading_kp}|{front_stop_mm}"
        self.ble.send_command(CMD.DRIVE_CELL_MM, payload)
        return self._wait_for_prefix(("DRIVE_DONE", "DRIVE_FAILED", "DRIVE_STOPPED_TOF"),
                                     duration_ms / 1000.0 + 2.0,
                                     start_idx)

    def request_nav_log(self) -> list[list[str]]:
        self.nav_rows.clear()
        start_idx = len(self.messages)
        self.ble.send_command(CMD.SEND_NAV_LOG, "")
        self._wait_for_prefix(("NAV_END",), 10.0, start_idx)
        return list(self.nav_rows)

    def stop_nav(self) -> None:
        self.ble.send_command(CMD.STOP_NAV, "")


def desired_heading_for_step(current: Cell, nxt: Cell) -> float:
    dx = nxt[0] - current[0]
    dy = nxt[1] - current[1]
    if (dx, dy) == (1, 0):
        return 0.0
    if (dx, dy) == (-1, 0):
        return 180.0
    if (dx, dy) == (0, 1):
        return 90.0
    if (dx, dy) == (0, -1):
        return -90.0
    raise ValueError(f"Cells {current} and {nxt} are not 4-neighbors")


def make_default_log_path() -> Path:
    stamp = time.strftime("%Y%m%d_%H%M%S")
    return BLE_PYTHON_DIR / "logs" / f"lab12_navigation_{stamp}.jsonl"


def append_jsonl(path: Path, record: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("a", encoding="utf-8") as f:
        f.write(json.dumps(record) + "\n")


def run_robot_navigation(args: argparse.Namespace,
                         planner: Lab12GridPlanner,
                         initial_route: list[Cell]) -> None:
    start = tuple(args.start)
    goal = tuple(args.goal)
    log_path = args.log_jsonl or make_default_log_path()

    localizer = Lab12Localizer(planner, sensor_sigma_m=args.sensor_sigma_m)
    nav = Lab12Navigator()

    print(f"Connecting to BLE robot; run log will be written to {log_path}")
    nav.connect()
    nav.start_notify()

    try:
        print("Initial 360 degree scan for Lab11 localization...")
        localizer.set_uniform_prior()
        scan = nav.run_map_scan(args.scan_step_deg,
                                args.scan_num_steps,
                                args.map_kp,
                                args.map_ki,
                                args.map_kd,
                                args.scan_settle_ms,
                                args.scan_timeout_s)
        state = localizer.update_from_scan(scan)
        current_cell = state.cell
        current_heading = state.heading_deg
        print(f"Initial belief: cell={state.cell}, heading={state.heading_deg:.1f}, "
              f"confidence={state.confidence:.4f}")

        append_jsonl(log_path, {
            "event": "initial_localization",
            "state": asdict(state),
            "initial_route": initial_route,
        })

        for step_idx in range(args.max_steps):
            if planner.goal_reached(current_cell, goal):
                print(f"Reached target neighborhood at {current_cell}; goal={goal}")
                append_jsonl(log_path, {
                    "event": "goal_reached",
                    "step": step_idx,
                    "cell": current_cell,
                    "goal": goal,
                })
                break

            route = planner.astar(current_cell, goal)
            if len(route) < 2:
                raise RuntimeError(f"Planner returned no next waypoint from {current_cell}")

            next_cell = route[1]
            desired_heading = desired_heading_for_step(current_cell, next_cell)
            world_delta = wrap_deg(desired_heading - current_heading)
            dmp_delta = args.dmp_delta_sign * world_delta

            step_start_s = time.time()
            step_log = NavStepLog(
                step=step_idx,
                planned_cell=next_cell,
                expected_cell=next_cell,
                route=route,
            )

            print(f"Step {step_idx}: {current_cell} -> {next_cell}, "
                  f"world_delta={world_delta:.1f}, cmd_delta={dmp_delta:.1f}")
            step_log.turn_status = nav.turn(dmp_delta,
                                            args.turn_timeout_ms,
                                            args.turn_kp,
                                            args.turn_ki,
                                            args.turn_kd)
            print(f"  turn: {step_log.turn_status}")
            step_log.turn_motion_log = nav.request_nav_log()
            if not step_log.turn_status.startswith("TURN_DONE"):
                step_log.elapsed_s = time.time() - step_start_s
                append_jsonl(log_path, {"event": "step_failed", "step": asdict(step_log)})
                break

            step_log.drive_status = nav.drive_cell(args.drive_mm,
                                                   args.base_pwm,
                                                   args.drive_duration_ms,
                                                   args.heading_kp,
                                                   args.front_stop_mm)
            print(f"  drive: {step_log.drive_status}")
            step_log.drive_motion_log = nav.request_nav_log()
            step_log.tof_safety_stop = step_log.drive_status.startswith("DRIVE_STOPPED_TOF")

            expected_cell = next_cell if step_log.drive_status.startswith("DRIVE_DONE") else current_cell
            step_log.expected_cell = expected_cell
            localizer.set_motion_prior(expected_cell, desired_heading)

            scan = nav.run_map_scan(args.scan_step_deg,
                                    args.scan_num_steps,
                                    args.map_kp,
                                    args.map_ki,
                                    args.map_kd,
                                    args.scan_settle_ms,
                                    args.scan_timeout_s)
            state = localizer.update_from_scan(scan)
            current_cell = state.cell
            current_heading = state.heading_deg

            step_log.localized_cell = state.cell
            step_log.localized_heading_deg = state.heading_deg
            step_log.confidence = state.confidence
            step_log.elapsed_s = time.time() - step_start_s
            append_jsonl(log_path, {"event": "step", "step": asdict(step_log)})
            print(f"  localized: cell={state.cell}, heading={state.heading_deg:.1f}, "
                  f"confidence={state.confidence:.4f}")

            if not step_log.drive_status.startswith("DRIVE_DONE"):
                print("Stopping integration run because drive did not finish cleanly.")
                break
        else:
            print(f"Max steps reached before target neighborhood: {args.max_steps}")

    finally:
        try:
            nav.stop_nav()
        except Exception:
            pass
        try:
            nav.stop_notify()
        except Exception:
            pass
        nav.disconnect()


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Lab12 local planning for the BLE robot")
    parser.add_argument("--start", nargs=2, type=int, default=DEFAULT_START, metavar=("X", "Y"))
    parser.add_argument("--goal", nargs=2, type=int, default=DEFAULT_GOAL, metavar=("X", "Y"))
    parser.add_argument("--inflation-ft", type=float, default=0.5)
    parser.add_argument("--plot-out", type=Path, default=None)

    parser.add_argument("--run-robot", action="store_true")
    parser.add_argument("--max-steps", type=int, default=16)
    parser.add_argument("--log-jsonl", type=Path, default=None)

    parser.add_argument("--scan-step-deg", type=float, default=20.0)
    parser.add_argument("--scan-num-steps", type=int, default=18)
    parser.add_argument("--scan-settle-ms", type=int, default=550)
    parser.add_argument("--scan-timeout-s", type=float, default=35.0)
    parser.add_argument("--map-kp", type=float, default=0.8)
    parser.add_argument("--map-ki", type=float, default=0.001)
    parser.add_argument("--map-kd", type=float, default=0.5)

    parser.add_argument("--turn-timeout-ms", type=int, default=3500)
    parser.add_argument("--turn-kp", type=float, default=0.8)
    parser.add_argument("--turn-ki", type=float, default=0.001)
    parser.add_argument("--turn-kd", type=float, default=0.2)
    parser.add_argument("--dmp-delta-sign", type=float, default=-1.0,
                        help="Use -1 when DMP yaw increases clockwise but map heading is CCW.")

    parser.add_argument("--drive-mm", type=float, default=304.8)
    parser.add_argument("--base-pwm", type=int, default=90)
    parser.add_argument("--drive-duration-ms", type=int, default=900)
    parser.add_argument("--heading-kp", type=float, default=1.2)
    parser.add_argument("--front-stop-mm", type=int, default=250)
    parser.add_argument("--sensor-sigma-m", type=float, default=0.18)
    return parser


def main() -> int:
    args = build_arg_parser().parse_args()
    start = tuple(args.start)
    goal = tuple(args.goal)

    planner = Lab12GridPlanner(WORLD_YAML, wall_inflation_ft=args.inflation_ft)
    route = planner.astar(start, goal)

    print(f"Loaded {WORLD_YAML}")
    print(f"Grid size: {planner.max_cells_x}x{planner.max_cells_y}, "
          f"wall inflation={args.inflation_ft:.2f} ft")
    print(planner.render_ascii(route=route, start=start, goal=goal))
    print(f"A* route ({len(route)} cells): {route}")

    if args.plot_out is not None:
        saved_path = planner.plot(route, start, goal, args.plot_out)
        print(f"Saved route plot to {saved_path}")

    if args.run_robot:
        run_robot_navigation(args, planner, route)
    else:
        print("Dry run only. Add --run-robot to connect to BLE and execute the route.")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
