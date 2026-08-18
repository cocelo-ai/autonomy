#!/usr/bin/env python3
"""Compact terminal dashboard for Super-LIO and elevation-map health."""

import argparse
import math
import os
import shutil
import sys
import time

import rclpy
from grid_map_msgs.msg import GridMap
from geometry_msgs.msg import Twist
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float32, String


def positive_rate(value: str) -> float:
    rate = float(value)
    if not math.isfinite(rate) or rate <= 0.0:
        raise argparse.ArgumentTypeError("rate must be a finite value greater than zero")
    return rate


class AutonomyStateMonitor(Node):
    """Keeps raw ROS logs in files while presenting only current state on TTY."""

    _RESET = "\033[0m"
    _BOLD = "\033[1m"
    _DIM = "\033[2m"
    _RED = "\033[31m"
    _GREEN = "\033[32m"
    _YELLOW = "\033[33m"
    _CYAN = "\033[36m"

    def __init__(self, rate_hz: float, log_dir: str, color_mode: str) -> None:
        super().__init__("autonomy_state_monitor")
        self._color = color_mode == "always" or (
            color_mode == "auto" and sys.stdout.isatty())
        self._redraw = sys.stdout.isatty()
        self._log_dir = log_dir
        # The border itself consumes two terminal columns.  Use the actual
        # terminal width so a normal 80/100-column console never wraps the
        # right border; non-interactive output uses a practical 100-column
        # fallback.
        terminal_columns = shutil.get_terminal_size(fallback=(100, 24)).columns
        self._table_width = max(40, terminal_columns - 2)
        self._odom: Odometry | None = None
        self._odom_received_at: float | None = None
        self._elevation: GridMap | None = None
        self._elevation_received_at: float | None = None
        self._elevation_heartbeat = "waiting_for_observation"
        self._elevation_heartbeat_received_at: float | None = None
        self._merge_heartbeat = "waiting_for_synchronized_observation"
        self._merge_heartbeat_received_at: float | None = None
        self._elevation_processing_ms: float | None = None
        self._merge_processing_ms: float | None = None
        self._sync_skew_ms: float | None = None
        self._nav2_nodes = (
            "controller_server",
            "velocity_smoother",
            "planner_server",
            "behavior_server",
            "bt_navigator",
        )
        self._nav2_clients = {
            name: self.create_client(GetState, f"/{name}/get_state")
            for name in self._nav2_nodes
        }
        self._nav2_states: dict[str, tuple[str, float]] = {}
        self._nav2_pending: set[str] = set()
        self._global_costmap: OccupancyGrid | None = None
        self._global_costmap_received_at: float | None = None
        self._local_costmap: OccupancyGrid | None = None
        self._local_costmap_received_at: float | None = None
        self._plan: Path | None = None
        self._plan_received_at: float | None = None
        self._command: Twist | None = None
        self._command_received_at: float | None = None

        self.create_subscription(Odometry, "/lio/odom", self._on_odom, 10)
        self.create_subscription(GridMap, "/autonomy_light/elevation_map",
                                 self._on_elevation, 1)
        self.create_subscription(String,
                                 "/autonomy_light/heartbeat/precise_elevation_mapping",
                                 self._on_elevation_heartbeat, 10)
        self.create_subscription(String, "/autonomy_light/heartbeat/observation_merge",
                                 self._on_merge_heartbeat, 10)
        self.create_subscription(Float32, "/autonomy_light/elevation_processing_ms",
                                 self._on_elevation_processing, qos_profile_sensor_data)
        self.create_subscription(Float32, "/autonomy_light/observation_processing_ms",
                                 self._on_merge_processing, qos_profile_sensor_data)
        self.create_subscription(Float32, "/autonomy_light/observation_sync_skew_ms",
                                 self._on_sync_skew, qos_profile_sensor_data)
        self.create_subscription(OccupancyGrid, "/global_costmap/costmap",
                                 self._on_global_costmap, 1)
        self.create_subscription(OccupancyGrid, "/local_costmap/costmap",
                                 self._on_local_costmap, 1)
        self.create_subscription(Path, "/plan", self._on_plan, 1)
        self.create_subscription(Twist, "/nav2/cmd_vel", self._on_command, 10)
        self.create_timer(1.0, self._refresh_nav2_states)
        self.create_timer(1.0 / rate_hz, self._print_state)

    def _on_odom(self, message: Odometry) -> None:
        self._odom = message
        self._odom_received_at = time.monotonic()

    def _on_elevation(self, message: GridMap) -> None:
        self._elevation = message
        self._elevation_received_at = time.monotonic()

    def _on_elevation_heartbeat(self, message: String) -> None:
        self._elevation_heartbeat = message.data or "unknown"
        self._elevation_heartbeat_received_at = time.monotonic()

    def _on_merge_heartbeat(self, message: String) -> None:
        self._merge_heartbeat = message.data or "unknown"
        self._merge_heartbeat_received_at = time.monotonic()

    def _on_elevation_processing(self, message: Float32) -> None:
        self._elevation_processing_ms = message.data

    def _on_merge_processing(self, message: Float32) -> None:
        self._merge_processing_ms = message.data

    def _on_sync_skew(self, message: Float32) -> None:
        self._sync_skew_ms = message.data

    def _on_global_costmap(self, message: OccupancyGrid) -> None:
        self._global_costmap = message
        self._global_costmap_received_at = time.monotonic()

    def _on_local_costmap(self, message: OccupancyGrid) -> None:
        self._local_costmap = message
        self._local_costmap_received_at = time.monotonic()

    def _on_plan(self, message: Path) -> None:
        self._plan = message
        self._plan_received_at = time.monotonic()

    def _on_command(self, message: Twist) -> None:
        self._command = message
        self._command_received_at = time.monotonic()

    def _refresh_nav2_states(self) -> None:
        for name, client in self._nav2_clients.items():
            if name in self._nav2_pending or not client.service_is_ready():
                continue
            self._nav2_pending.add(name)
            future = client.call_async(GetState.Request())
            future.add_done_callback(lambda completed, node_name=name:
                                     self._on_nav2_state(node_name, completed))

    def _on_nav2_state(self, name: str, future: object) -> None:
        self._nav2_pending.discard(name)
        try:
            response = future.result()
            state = response.current_state
            self._nav2_states[name] = (state.label or f"state {state.id}", time.monotonic())
        except Exception:
            self._nav2_states[name] = ("unavailable", time.monotonic())

    @staticmethod
    def _age(received_at: float | None) -> float | None:
        return None if received_at is None else max(0.0, time.monotonic() - received_at)

    @staticmethod
    def _rpy_degrees(odom: Odometry) -> tuple[float, float, float]:
        quaternion = odom.pose.pose.orientation
        sin_roll = 2.0 * (quaternion.w * quaternion.x + quaternion.y * quaternion.z)
        cos_roll = 1.0 - 2.0 * (quaternion.x ** 2 + quaternion.y ** 2)
        roll = math.atan2(sin_roll, cos_roll)
        sin_pitch = 2.0 * (quaternion.w * quaternion.y - quaternion.z * quaternion.x)
        pitch = math.asin(max(-1.0, min(1.0, sin_pitch)))
        sin_yaw = 2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y)
        cos_yaw = 1.0 - 2.0 * (quaternion.y ** 2 + quaternion.z ** 2)
        yaw = math.atan2(sin_yaw, cos_yaw)
        return tuple(math.degrees(value) for value in (roll, pitch, yaw))

    def _paint(self, text: str, color: str = "", bold: bool = False) -> str:
        if not self._color:
            return text
        prefix = (self._BOLD if bold else "") + color
        return f"{prefix}{text}{self._RESET}" if prefix else text

    def _state_color(self, received_at: float | None, ready: bool = True) -> str:
        age = self._age(received_at)
        if not ready or age is None or age > 2.0:
            return self._RED
        if age > 0.5:
            return self._YELLOW
        return self._GREEN

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

    def _odom_rows(self) -> list[tuple[str, str, str, bool]]:
        if self._odom is None:
            return [("SLAM", "WAITING FOR /lio/odom", self._RED, True),
                    ("POSITION", "waiting for Super-LIO initialization", self._DIM, False),
                    ("ATTITUDE", "", "", False),
                    ("VELOCITY", "", "", False)]
        position = self._odom.pose.pose.position
        velocity = self._odom.twist.twist.linear
        speed = math.sqrt(velocity.x ** 2 + velocity.y ** 2 + velocity.z ** 2)
        roll, pitch, yaw = self._rpy_degrees(self._odom)
        frame = self._odom.header.frame_id or "unknown"
        child_frame = self._odom.child_frame_id or "unknown"
        age = self._age(self._odom_received_at)
        age_text = "n/a" if age is None else f"{age:.2f}s"
        return [
            ("SLAM", f"READY | /lio/odom age {age_text} | {frame} -> {child_frame}",
             self._state_color(self._odom_received_at), True),
            ("POSITION", f"map ({position.x:+.3f}, {position.y:+.3f}, {position.z:+.3f}) m",
             "", False),
            ("ATTITUDE", f"roll {roll:+.2f}° | pitch {pitch:+.2f}° | yaw {yaw:+.2f}°",
             "", False),
            ("VELOCITY", f"({velocity.x:+.3f}, {velocity.y:+.3f}, {velocity.z:+.3f}) m/s | "
             f"speed {speed:.3f} m/s", "", False),
        ]

    def _elevation_rows(self) -> list[tuple[str, str, str, bool]]:
        map_ready = self._elevation is not None
        map_age = self._age(self._elevation_received_at)
        map_age_text = "n/a" if map_age is None else f"{map_age:.2f}s"
        ready = map_ready and self._elevation_heartbeat == "ready"
        rows = [
            ("ELEVATION", f"{self._elevation_heartbeat} | map age {map_age_text}",
             self._state_color(self._elevation_received_at, ready), True),
        ]
        if self._elevation is None:
            rows.append(("GRID", "waiting for /autonomy_light/elevation_map", self._DIM, False))
        else:
            info = self._elevation.info
            width = max(0, round(info.length_x / info.resolution)) if info.resolution > 0.0 else 0
            height = max(0, round(info.length_y / info.resolution)) if info.resolution > 0.0 else 0
            layer_count = len(self._elevation.layers)
            if layer_count == 0:
                layer_text = "no layers"
            elif layer_count == 1:
                layer_text = self._elevation.layers[0]
            elif "elevation" in self._elevation.layers:
                layer_text = f"{layer_count} layers (elevation + {layer_count - 1})"
            else:
                layer_text = f"{layer_count} layers"
            valid = total = 0
            try:
                elevation_index = list(self._elevation.layers).index("elevation")
                values = self._elevation.data[elevation_index].data
                total = len(values)
                valid = sum(math.isfinite(value) for value in values)
            except (ValueError, IndexError):
                pass
            valid_text = "n/a" if total == 0 else f"{valid}/{total} ({100.0 * valid / total:.1f}%)"
            rows.extend([
                ("GRID", f"{self._elevation.header.frame_id or 'unknown'} | {width}x{height} @ "
                 f"{info.resolution:.3f} m | {layer_text}", "", False),
                ("VALID CELLS", valid_text, self._state_color(self._elevation_received_at, valid > 0), False),
            ])
        merge_age = self._age(self._merge_heartbeat_received_at)
        merge_age_text = "n/a" if merge_age is None else f"{merge_age:.2f}s"
        rows.append(("MERGE", f"{self._merge_heartbeat} | heartbeat age {merge_age_text}",
                     self._state_color(self._merge_heartbeat_received_at,
                                       self._merge_heartbeat.startswith("ready")), False))
        timing = (
            f"merge {'n/a' if self._merge_processing_ms is None else f'{self._merge_processing_ms:.2f} ms'}"
            f" | elevation {'n/a' if self._elevation_processing_ms is None else f'{self._elevation_processing_ms:.2f} ms'}"
            f" | sync skew {'n/a' if self._sync_skew_ms is None else f'{self._sync_skew_ms:.2f} ms'}")
        rows.append(("TIMING", timing, "", False))
        return rows

    def _nav2_rows(self) -> list[tuple[str, str, str, bool]]:
        states = []
        active = True
        for name in self._nav2_nodes:
            label, received_at = self._nav2_states.get(name, ("waiting", None))
            fresh = self._age(received_at) is not None and self._age(received_at) <= 3.0
            active = active and fresh and label.lower() == "active"
            states.append(f"{name.replace('_server', '')}: {label}")
        lifecycle_color = self._GREEN if active else self._YELLOW
        if not self._nav2_states:
            lifecycle_color = self._RED

        def costmap_text(name: str, message: OccupancyGrid | None,
                         received_at: float | None) -> tuple[str, str]:
            age = self._age(received_at)
            age_text = "n/a" if age is None else f"{age:.2f}s"
            if message is None:
                return f"{name}: waiting", self._RED
            info = message.info
            return (f"{name}: {info.width}x{info.height} @ {info.resolution:.2f} m | "
                    f"{message.header.frame_id or 'unknown'} | age {age_text}",
                    self._state_color(received_at))

        global_text, global_color = costmap_text(
            "global", self._global_costmap, self._global_costmap_received_at)
        local_text, local_color = costmap_text(
            "local", self._local_costmap, self._local_costmap_received_at)
        plan_age = self._age(self._plan_received_at)
        plan_age_text = "n/a" if plan_age is None else f"{plan_age:.2f}s"
        plan_text = ("waiting for /plan" if self._plan is None else
                     f"{len(self._plan.poses)} poses | age {plan_age_text}")
        if self._command is None:
            command_text = "waiting for /nav2/cmd_vel"
        else:
            command_text = (f"vx {self._command.linear.x:+.3f} | vy {self._command.linear.y:+.3f} | "
                            f"wz {self._command.angular.z:+.3f} | age "
                            f"{self._age(self._command_received_at):.2f}s")
        return [
            ("LIFECYCLE", " | ".join(states), lifecycle_color, True),
            ("COSTMAP", global_text, global_color, False),
            ("COSTMAP", local_text, local_color, False),
            ("GLOBAL PLAN", plan_text, self._state_color(self._plan_received_at), False),
            ("NAV2 CMD", command_text, self._state_color(self._command_received_at), False),
        ]

    def _print_state(self) -> None:
        lines = [
            self._border("╔", "╗", "AUTONOMY / LIO STATUS"),
            *(self._row(label, value, color, bold)
              for label, value, color, bold in self._odom_rows()),
            self._border("╠", "╣", "ELEVATION MAP"),
            *(self._row(label, value, color, bold)
              for label, value, color, bold in self._elevation_rows()),
            self._border("╠", "╣", "NAV2"),
            *(self._row(label, value, color, bold)
              for label, value, color, bold in self._nav2_rows()),
            self._border("╠", "╣", "RAW NODE LOGS"),
            self._row("DIRECTORY", os.path.join("log", os.path.basename(self._log_dir)), self._DIM),
            "╚" + "═" * self._table_width + "╝",
        ]
        if self._redraw:
            print("\033[2J\033[H", end="")
        print("\n".join(lines), flush=True)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--rate", type=positive_rate, default=2.0,
                        help="terminal refresh frequency in Hz (default: 2.0)")
    parser.add_argument("--log-dir", required=True,
                        help="directory containing raw node stdout/stderr logs")
    parser.add_argument("--color", choices=("auto", "always", "never"), default="auto")
    arguments = parser.parse_args()

    rclpy.init()
    node = AutonomyStateMonitor(arguments.rate, arguments.log_dir, arguments.color)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
