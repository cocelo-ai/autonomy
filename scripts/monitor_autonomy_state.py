#!/usr/bin/env python3
"""Colour terminal dashboard for the LiDAR-only autonomy pipeline."""

import argparse
import math
import signal
import shutil
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy._rclpy_pybind11 import RCLError
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from autonomy_light.msg import HeightMap


def positive_rate(value: str) -> float:
    rate = float(value)
    if not math.isfinite(rate) or rate <= 0.0:
        raise argparse.ArgumentTypeError("rate must be a finite value greater than zero")
    return rate


class AutonomyStateMonitor(Node):
    """Keeps node logs separate while rendering one concise live dashboard."""

    _RESET = "\033[0m"
    _BOLD = "\033[1m"
    _DIM = "\033[2m"
    _RED = "\033[31m"
    _GREEN = "\033[32m"
    _YELLOW = "\033[33m"
    _BLUE = "\033[34m"
    _CYAN = "\033[36m"

    _NAV2_NODES = (
        "controller_server",
        "velocity_smoother",
        "planner_server",
        "behavior_server",
        "bt_navigator",
    )

    def __init__(self, rate_hz: float, color_mode: str, show_height_grid: bool) -> None:
        super().__init__("autonomy_state_monitor")
        self._color = color_mode == "always" or (
            color_mode == "auto" and sys.stdout.isatty())
        self._redraw = sys.stdout.isatty()
        self._table_width = max(94, shutil.get_terminal_size(fallback=(100, 24)).columns - 2)
        self._show_height_grid = show_height_grid
        self._odom: Odometry | None = None
        self._odom_received_at: float | None = None
        self._height_map: HeightMap | None = None
        self._height_map_received_at: float | None = None
        self._nav_command: Twist | None = None
        self._nav_command_received_at: float | None = None
        self._autopilot_command: Twist | None = None
        self._autopilot_command_received_at: float | None = None
        self._map: OccupancyGrid | None = None
        self._map_received_at: float | None = None
        # nav2_lifecycle_manager is not itself a LifecycleNode. Query its five
        # managed LifecycleNodes directly, otherwise a non-existent manager
        # /get_state service leaves the dashboard permanently in "waiting".
        self._nav2_state_clients = {
            name: self.create_client(GetState, f"/{name}/get_state")
            for name in self._NAV2_NODES
        }
        self._nav2_states: dict[str, tuple[str, float]] = {}
        self._nav2_state_pending: set[str] = set()

        self.create_subscription(Odometry, "/lio/odom", self._on_odom, 10)
        self.create_subscription(HeightMap, "/autonomy_light/elevation_map",
                                 self._on_height_map, 1)
        self.create_subscription(Twist, "/nav2/cmd_vel", self._on_nav_command, 10)
        self.create_subscription(Twist, "/autonomy_light/autopilot_command",
                                 self._on_autopilot_command, 10)
        self.create_subscription(OccupancyGrid, "/map", self._on_map, 1)
        self.create_timer(1.0, self._refresh_nav2_state)
        self.create_timer(1.0 / rate_hz, self._print_state)

    def _on_odom(self, message: Odometry) -> None:
        self._odom = message
        self._odom_received_at = time.monotonic()

    def _on_height_map(self, message: HeightMap) -> None:
        self._height_map = message
        self._height_map_received_at = time.monotonic()

    def _on_nav_command(self, message: Twist) -> None:
        self._nav_command = message
        self._nav_command_received_at = time.monotonic()

    def _on_autopilot_command(self, message: Twist) -> None:
        self._autopilot_command = message
        self._autopilot_command_received_at = time.monotonic()

    def _on_map(self, message: OccupancyGrid) -> None:
        self._map = message
        self._map_received_at = time.monotonic()

    def _refresh_nav2_state(self) -> None:
        for name, client in self._nav2_state_clients.items():
            if name in self._nav2_state_pending or not client.service_is_ready():
                continue
            self._nav2_state_pending.add(name)
            future = client.call_async(GetState.Request())
            future.add_done_callback(
                lambda completed, node_name=name: self._on_nav2_state(node_name, completed))

    def _on_nav2_state(self, name: str, future: object) -> None:
        self._nav2_state_pending.discard(name)
        try:
            response = future.result()
            state = response.current_state.label or f"state {response.current_state.id}"
        except Exception:
            state = "unavailable"
        self._nav2_states[name] = (state, time.monotonic())

    @staticmethod
    def _age(received_at: float | None) -> float | None:
        return None if received_at is None else max(0.0, time.monotonic() - received_at)

    def _age_text(self, received_at: float | None) -> str:
        age = self._age(received_at)
        return "n/a" if age is None else f"{age:.2f}s"

    def _fresh_color(self, received_at: float | None, ready: bool = True) -> str:
        age = self._age(received_at)
        if not ready or age is None or age > 2.0:
            return self._RED
        if age > 0.5:
            return self._YELLOW
        return self._GREEN

    @staticmethod
    def _roll_pitch_yaw_deg(odom: Odometry) -> tuple[float, float, float]:
        orientation = odom.pose.pose.orientation
        roll = math.atan2(
            2.0 * (orientation.w * orientation.x + orientation.y * orientation.z),
            1.0 - 2.0 * (orientation.x ** 2 + orientation.y ** 2))
        pitch = math.asin(max(-1.0, min(
            1.0, 2.0 * (orientation.w * orientation.y - orientation.z * orientation.x))))
        yaw = math.atan2(
            2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
            1.0 - 2.0 * (orientation.y ** 2 + orientation.z ** 2))
        return tuple(math.degrees(value) for value in (roll, pitch, yaw))

    def _paint(self, text: str, color: str = "", bold: bool = False) -> str:
        if not self._color:
            return text
        prefix = (self._BOLD if bold else "") + color
        return f"{prefix}{text}{self._RESET}" if prefix else text

    def _fit(self, text: str, width: int) -> str:
        return text.ljust(width) if len(text) <= width else f"{text[:width - 1]}…"

    def _border(self, left: str, right: str, title: str) -> str:
        title = f" {title} "
        left_width = max(0, (self._table_width - len(title)) // 2)
        right_width = max(0, self._table_width - len(title) - left_width)
        return (left + "═" * left_width + self._paint(title, self._CYAN, bold=True) +
                "═" * right_width + right)

    def _row(self, label: str, value: str, color: str = "", bold: bool = False) -> str:
        value_width = max(1, self._table_width - 13)
        return ("║ " + self._paint(f"{label:<11}", self._CYAN, bold=True) + " " +
                self._paint(self._fit(value, value_width), color, bold=bold) + "║")

    def _lio_rows(self) -> list[tuple[str, str, str, bool]]:
        if self._odom is None:
            return [
                ("SLAM", "WAITING FOR /lio/odom", self._RED, True),
                ("POSITION", "waiting for Super-LIO initialization", self._DIM, False),
                ("ATTITUDE", "", "", False),
                ("VELOCITY", "", "", False),
            ]
        position = self._odom.pose.pose.position
        velocity = self._odom.twist.twist.linear
        roll, pitch, yaw = self._roll_pitch_yaw_deg(self._odom)
        speed = math.sqrt(velocity.x ** 2 + velocity.y ** 2 + velocity.z ** 2)
        frame = self._odom.header.frame_id or "unknown"
        child_frame = self._odom.child_frame_id or "unknown"
        return [
            ("SLAM", f"READY | /lio/odom age {self._age_text(self._odom_received_at)} | "
             f"{frame} -> {child_frame}", self._fresh_color(self._odom_received_at), True),
            ("POSITION", f"map ({position.x:+.3f}, {position.y:+.3f}, {position.z:+.3f}) m",
             "", False),
            ("ATTITUDE", f"roll {roll:+.2f}° | pitch {pitch:+.2f}° | yaw {yaw:+.2f}°",
             "", False),
            ("VELOCITY", f"({velocity.x:+.3f}, {velocity.y:+.3f}, {velocity.z:+.3f}) m/s | "
             f"speed {speed:.3f} m/s", "", False),
        ]

    def _elevation_rows(self) -> list[tuple[str, str, str, bool]]:
        if self._height_map is None:
            return [
                ("HEIGHT MAP", "WAITING FOR /autonomy_light/elevation_map", self._RED, True),
                ("GRID", "waiting for the first local Super-LIO crop", self._DIM, False),
            ]
        resolution = float(self._height_map.resolution)
        width = round(float(self._height_map.x_length) / resolution) if resolution > 0.0 else 0
        height = round(float(self._height_map.y_length) / resolution) if resolution > 0.0 else 0
        values = self._height_map.data
        finite = sum(math.isfinite(value) for value in values)
        return [
            ("HEIGHT MAP", f"READY | message age {self._age_text(self._height_map_received_at)}",
             self._fresh_color(self._height_map_received_at), True),
            ("GRID", f"{width}x{height} @ {resolution:.3f} m | "
             f"target-link distance | finite {finite}/{len(values)}",
             self._fresh_color(self._height_map_received_at, finite > 0), False),
        ]

    def _height_grid_rows(self) -> list[str]:
        if not self._show_height_grid or self._height_map is None:
            return []
        resolution = float(self._height_map.resolution)
        width = round(float(self._height_map.x_length) / resolution) if resolution > 0.0 else 0
        height = round(float(self._height_map.y_length) / resolution) if resolution > 0.0 else 0
        values = self._height_map.data
        if width <= 0 or height <= 0 or width * height != len(values):
            return [self._row("VALUES", "invalid HeightMap geometry", self._RED, True)]
        finite_values = [value for value in values if math.isfinite(value)]
        if not finite_values:
            return [self._row("VALUES", "all HeightMap cells are non-finite", self._RED, True)]
        lower = min(finite_values)
        upper = max(finite_values)
        span = upper - lower
        lines = [self._row("VALUES", f"blue = {lower:.2f} m, red = {upper:.2f} m | "
                           "row 0 is grid y_min", self._CYAN)]
        for row in range(height):
            cells: list[str] = []
            for value in values[row * width:(row + 1) * width]:
                if not math.isfinite(value):
                    cells.append(self._paint(" nan ", self._DIM))
                    continue
                normalized = 0.5 if span <= 1.0e-6 else (value - lower) / span
                if normalized < 0.25:
                    color = self._BLUE
                elif normalized < 0.50:
                    color = self._CYAN
                elif normalized < 0.75:
                    color = self._YELLOW
                else:
                    color = self._RED
                cells.append(self._paint(f"{value:4.2f} ", color, bold=True))
            label = f"Y{row:02d}"
            plain_width = 4 + 1 + 5 * width
            padding = " " * max(0, self._table_width - 1 - plain_width)
            lines.append("║ " + self._paint(f"{label:<4}", self._CYAN, bold=True) + " " +
                         "".join(cells) + padding + "║")
        return lines

    def _navigation_rows(self) -> list[tuple[str, str, str, bool]]:
        labels = []
        all_active = True
        newest_state_at: float | None = None
        for name in self._NAV2_NODES:
            state, received_at = self._nav2_states.get(name, ("waiting", None))
            fresh = self._age(received_at) is not None and self._age(received_at) <= 3.0
            all_active = all_active and fresh and state.lower() == "active"
            labels.append(f"{name.removesuffix('_server')}: {state}")
            if received_at is not None:
                newest_state_at = max(newest_state_at or received_at, received_at)
        lifecycle_text = ("ALL MANAGED NODES ACTIVE" if all_active else " | ".join(labels))
        lifecycle_text += f" | update {self._age_text(newest_state_at)}"
        if self._nav_command is None:
            nav_command_text = "WAITING FOR /nav2/cmd_vel (normal until a navigation command is issued)"
            nav_command_color = self._DIM
        else:
            command = self._nav_command
            nav_command_text = (f"vx {command.linear.x:+.3f} | vy {command.linear.y:+.3f} | "
                                f"wz {command.angular.z:+.3f} | "
                                f"age {self._age_text(self._nav_command_received_at)}")
            nav_command_color = self._fresh_color(self._nav_command_received_at)
        if self._autopilot_command is None:
            autopilot_command_text = "WAITING FOR /autonomy_light/autopilot_command"
            autopilot_command_color = self._YELLOW
        else:
            command = self._autopilot_command
            autopilot_command_text = (f"vx {command.linear.x:+.3f} | "
                                      f"vy {command.linear.y:+.3f} | "
                                      f"wz {command.angular.z:+.3f} | "
                                      f"age {self._age_text(self._autopilot_command_received_at)}")
            autopilot_command_color = self._fresh_color(self._autopilot_command_received_at)
        if self._map is None:
            map_text = "WAITING FOR /map"
            map_color = self._YELLOW
        else:
            map_text = (f"{self._map.info.width}x{self._map.info.height} @ "
                        f"{self._map.info.resolution:.2f} m | "
                        f"age {self._age_text(self._map_received_at)}")
            map_color = self._fresh_color(self._map_received_at)
        return [
            ("LIFECYCLE", lifecycle_text,
             self._fresh_color(newest_state_at, all_active), True),
            ("OCCUPANCY", map_text, map_color, False),
            ("NAV2 CMD", nav_command_text, nav_command_color, False),
            ("AUTOPILOT CMD", autopilot_command_text, autopilot_command_color, False),
        ]

    def _print_state(self) -> None:
        lines = [
            self._border("╔", "╗", "AUTONOMY / LIO STATUS"),
            *(self._row(label, value, color, bold)
              for label, value, color, bold in self._lio_rows()),
            self._border("╠", "╣", "ELEVATION MAP"),
            *(self._row(label, value, color, bold)
              for label, value, color, bold in self._elevation_rows()),
            *self._height_grid_rows(),
            self._border("╠", "╣", "NAV2 / COMMAND"),
            *(self._row(label, value, color, bold)
              for label, value, color, bold in self._navigation_rows()),
            "╚" + "═" * self._table_width + "╝",
        ]
        if self._redraw:
            print("\033[2J\033[H", end="")
        print("\n".join(lines), flush=True)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--rate", type=positive_rate, default=2.0,
                        help="terminal refresh frequency in Hz (default: 2.0)")
    parser.add_argument("--color", choices=("auto", "always", "never"), default="auto",
                        help="ANSI colour mode (default: auto)")
    parser.add_argument("--show-height-grid", action="store_true",
                        help="render the numeric HeightMap grid with a blue-to-red scale")
    arguments = parser.parse_args()
    shutdown_requested = False

    def request_shutdown(_signum: int, _frame: object) -> None:
        nonlocal shutdown_requested
        shutdown_requested = True

    rclpy.init()
    signal.signal(signal.SIGINT, request_shutdown)
    signal.signal(signal.SIGTERM, request_shutdown)
    node = AutonomyStateMonitor(arguments.rate, arguments.color, arguments.show_height_grid)
    try:
        while rclpy.ok() and not shutdown_requested:
            rclpy.spin_once(node, timeout_sec=0.2)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RCLError as error:
        if "context is not valid" not in str(error):
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
