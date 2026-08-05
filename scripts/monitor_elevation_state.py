#!/usr/bin/env python3
"""Terminal elevation-layer view and Super-LIO health display."""

import argparse
import math
import os
import signal
import shutil
import sys
import time
from collections import deque

import rclpy
from grid_map_msgs.msg import GridMap
from autonomy_light.msg import HeightMap
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2


def positive_rate(value: str) -> float:
    rate = float(value)
    if not math.isfinite(rate) or rate <= 0.0:
        raise argparse.ArgumentTypeError("rate must be a finite value greater than zero")
    return rate


class ElevationStateMonitor(Node):
    """Render the latest native ``elevation`` GridMap layer as a top view."""

    DEFAULT_TABLE_WIDTH = 96
    RESET = "\033[0m"
    BOLD = "\033[1m"
    RED = "\033[31m"
    GREEN = "\033[32m"
    YELLOW = "\033[33m"
    CYAN = "\033[36m"

    def __init__(self, cloud_topic: str, camera_topic: str, map_topic: str, height_map_topic: str,
                 odom_topic: str, slam_mode: str, rate_hz: float, color_mode: str) -> None:
        super().__init__("elevation_map_state_monitor")
        self.color = color_mode == "always" or (color_mode == "auto" and sys.stdout.isatty())
        self.redraw = sys.stdout.isatty()
        self.requested_rate_hz = rate_hz
        # A 50 Hz full-screen redraw is useful in the launch terminal, but it
        # would make redirected logs unusable.  The live GridMap rate remains
        # visible in both modes.
        self.render_rate_hz = rate_hz if self.redraw else min(rate_hz, 2.0)
        self.cloud_topic = cloud_topic
        self.camera_topic = camera_topic
        self.map_topic = map_topic
        self.height_map_topic = height_map_topic
        self.odom_topic = odom_topic
        self.slam_mode = slam_mode
        self.cloud_at: float | None = None
        self.camera_at: float | None = None
        self.map_at: float | None = None
        self.odom_at: float | None = None
        self.height_map_at: float | None = None
        self.cloud_times: deque[float] = deque()
        self.camera_times: deque[float] = deque()
        self.map_times: deque[float] = deque()
        self.odom_times: deque[float] = deque()
        self.height_map_times: deque[float] = deque()
        self.cloud_points = 0
        self.camera_points = 0
        self.map_frame = "waiting"
        self.map_resolution = math.nan
        self.map_length_x = math.nan
        self.map_length_y = math.nan
        self.valid_cells = 0
        self.total_cells = 0
        self.height_low = math.nan
        self.height_high = math.nan
        self.uncertainty_p95 = math.nan
        self.odom: Odometry | None = None
        self.height_map: HeightMap | None = None

        # GridMap uses an Eigen column-major array on the wire. Keep only the
        # elevation layer for native-map diagnostics; the controller height
        # map below is rendered separately as exact values.
        self.elevation_data: list[float] = []
        self.grid_rows = 0
        self.grid_columns = 0
        self.outer_start_index = 0
        self.inner_start_index = 0
        self.column_major = True
        self.table_width = self.DEFAULT_TABLE_WIDTH
        self._first_draw = True
        self._cursor_hidden = False
        self._refresh_layout()

        # The monitor must never add backpressure to the 50 Hz fused-map
        # publisher.  Receiving only the latest map is exactly what a terminal
        # dashboard needs.
        latest_only_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(PointCloud2, cloud_topic, self.on_cloud, latest_only_qos)
        if camera_topic:
            self.create_subscription(PointCloud2, camera_topic, self.on_camera, latest_only_qos)
        self.create_subscription(GridMap, map_topic, self.on_map, latest_only_qos)
        self.create_subscription(HeightMap, height_map_topic, self.on_height_map, latest_only_qos)
        self.create_subscription(Odometry, odom_topic, self.on_odom, latest_only_qos)
        self.create_timer(1.0 / self.render_rate_hz, self.render)
        mode = "TTY redraw" if self.redraw else "non-interactive output"
        self.get_logger().info(
            f"Monitoring cloud={cloud_topic}, elevation={map_topic}, height={height_map_topic}, odom={odom_topic}; "
            f"{mode}={self.render_rate_hz:g} Hz (requested {rate_hz:g} Hz)")

    def _refresh_layout(self) -> bool:
        """Fit the box to the active terminal, avoiding line wrapping."""
        if self.redraw:
            terminal = shutil.get_terminal_size(fallback=(self.DEFAULT_TABLE_WIDTH + 2, 32))
            target_width = max(54, min(116, terminal.columns - 2))
            target_height = max(6, min(18, terminal.lines - 16))
        else:
            target_width = self.DEFAULT_TABLE_WIDTH
            target_height = 10
        changed = target_width != self.table_width
        self.table_width = target_width
        return changed

    def on_cloud(self, message: PointCloud2) -> None:
        now = time.monotonic()
        self.cloud_at = now
        self.cloud_times.append(now)
        self.cloud_points = message.width * message.height

    def on_camera(self, message: PointCloud2) -> None:
        now = time.monotonic()
        self.camera_at = now
        self.camera_times.append(now)
        self.camera_points = message.width * message.height

    def on_odom(self, message: Odometry) -> None:
        now = time.monotonic()
        self.odom_at = now
        self.odom_times.append(now)
        self.odom = message

    def on_height_map(self, message: HeightMap) -> None:
        now = time.monotonic()
        self.height_map_at = now
        self.height_map_times.append(now)
        self.height_map = message

    @staticmethod
    def layer_message(message: GridMap, name: str):
        try:
            index = list(message.layers).index(name)
        except ValueError:
            return None
        return message.data[index] if index < len(message.data) else None

    @staticmethod
    def finite_values(values) -> list[float]:
        return [value for value in values if math.isfinite(value)]

    def on_map(self, message: GridMap) -> None:
        elevation_layer = self.layer_message(message, "elevation")
        if elevation_layer is None:
            return
        dimensions = elevation_layer.layout.dim
        if len(dimensions) != 2 or not elevation_layer.data:
            return

        # grid_map_ros serializes Eigen::MatrixXf as (column_index,
        # row_index), where data[column * rows + row] is the cell value.
        first_label = dimensions[0].label.lower()
        second_label = dimensions[1].label.lower()
        self.column_major = "column" in first_label and "row" in second_label
        if self.column_major:
            self.grid_columns = int(dimensions[0].size)
            self.grid_rows = int(dimensions[1].size)
        else:
            self.grid_rows = int(dimensions[0].size)
            self.grid_columns = int(dimensions[1].size)
        if self.grid_rows <= 0 or self.grid_columns <= 0:
            return
        if len(elevation_layer.data) < self.grid_rows * self.grid_columns:
            return

        now = time.monotonic()
        self.map_at = now
        self.map_times.append(now)
        self.elevation_data = list(elevation_layer.data)
        self.outer_start_index = int(message.outer_start_index) % self.grid_rows
        self.inner_start_index = int(message.inner_start_index) % self.grid_columns
        elevation = self.finite_values(self.elevation_data)
        uncertainty_layer = self.layer_message(message, "uncertainty_range")
        # GridMap can retain a finite uncertainty value in cells that have no
        # elevation observation.  Restrict the diagnostic to observed cells;
        # otherwise the p95 is dominated by those unknown-space defaults.
        uncertainty = ([uncertainty_value for elevation_value, uncertainty_value
                        in zip(self.elevation_data, uncertainty_layer.data)
                        if math.isfinite(elevation_value) and math.isfinite(uncertainty_value)]
                       if uncertainty_layer is not None else [])
        self.valid_cells = len(elevation)
        self.total_cells = self.grid_rows * self.grid_columns
        self.height_low = min(elevation) if elevation else math.nan
        self.height_high = max(elevation) if elevation else math.nan
        if uncertainty:
            uncertainty.sort()
            self.uncertainty_p95 = uncertainty[min(
                len(uncertainty) - 1, int(math.ceil(len(uncertainty) * 0.95)) - 1)]
        else:
            self.uncertainty_p95 = math.nan
        self.map_frame = message.header.frame_id or "unknown"
        self.map_resolution = message.info.resolution
        self.map_length_x = message.info.length_x
        self.map_length_y = message.info.length_y
        self._refresh_layout()

    def elevation_at(self, logical_row: int, logical_column: int) -> float:
        """Read a logical x/y cell while accounting for GridMap's ring buffer."""
        physical_row = (logical_row + self.outer_start_index) % self.grid_rows
        physical_column = (logical_column + self.inner_start_index) % self.grid_columns
        if self.column_major:
            index = physical_column * self.grid_rows + physical_row
        else:
            index = physical_row * self.grid_columns + physical_column
        return self.elevation_data[index]

    def height_map_rows(self) -> list[str]:
        """Render every controller height-map cell as its actual numeric value."""
        message = self.height_map
        if message is None:
            return ["waiting for /autonomy_light/height_map_data"]
        if message.resolution <= 0.0 or message.x_length <= 0.0 or message.y_length <= 0.0:
            return ["invalid height_map_data geometry"]
        width = max(1, round(message.x_length / message.resolution))
        height = max(1, round(message.y_length / message.resolution))
        values = list(message.data)
        if len(values) != width * height:
            return [f"invalid sample count: {len(values)} (expected {width * height})"]

        # Four characters per cell (e.g. 0.48) plus one separator fit the
        # normal 100+ column launch terminal. On smaller terminals split a
        # physical row, never truncate numeric height-map data.
        inner_width = self.table_width - 2
        cells_per_line = max(1, (inner_width - 12) // 5)
        rows: list[str] = []
        for row in range(height - 1, -1, -1):
            for first_column in range(0, width, cells_per_line):
                last_column = min(width, first_column + cells_per_line)
                cells = []
                for column in range(first_column, last_column):
                    value = values[row * width + column]
                    cells.append(" nan" if not math.isfinite(value) else f"{value:4.2f}")
                rows.append(f"y={row:02d} c={first_column:02d} " + " ".join(cells))
        return rows

    @staticmethod
    def age(value: float | None) -> str:
        return "waiting" if value is None else f"{time.monotonic() - value:.2f}s"

    @staticmethod
    def rate(samples: deque[float]) -> float:
        now = time.monotonic()
        while samples and now - samples[0] > 1.0:
            samples.popleft()
        return float(len(samples))

    @staticmethod
    def roll_pitch_yaw_deg(message: Odometry) -> tuple[float, float, float]:
        quaternion = message.pose.pose.orientation
        sin_roll = 2.0 * (quaternion.w * quaternion.x + quaternion.y * quaternion.z)
        cos_roll = 1.0 - 2.0 * (quaternion.x ** 2 + quaternion.y ** 2)
        roll = math.atan2(sin_roll, cos_roll)
        sin_pitch = 2.0 * (quaternion.w * quaternion.y - quaternion.z * quaternion.x)
        pitch = math.asin(max(-1.0, min(1.0, sin_pitch)))
        sin_yaw = 2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y)
        cos_yaw = 1.0 - 2.0 * (quaternion.y ** 2 + quaternion.z ** 2)
        yaw = math.atan2(sin_yaw, cos_yaw)
        return tuple(math.degrees(value) for value in (roll, pitch, yaw))

    def paint(self, text: str, color: str = "", bold: bool = False) -> str:
        if not self.color or not (color or bold):
            return text
        return f"{self.BOLD if bold else ''}{color}{text}{self.RESET}"

    @staticmethod
    def fit(text: str, width: int) -> str:
        if width <= 0:
            return ""
        return text.ljust(width) if len(text) <= width else f"{text[:width - 1]}…"

    def border(self, left: str, right: str, title: str) -> str:
        title = f" {title} "
        left_width = (self.table_width - len(title)) // 2
        right_width = self.table_width - len(title) - left_width
        return (left + "═" * left_width + self.paint(title, self.CYAN, True) +
                "═" * right_width + right)

    def row(self, name: str, text: str, color: str = "", bold: bool = False) -> str:
        return ("║ " + self.paint(f"{name:<10}", self.CYAN, True) + " " +
                self.paint(self.fit(text, self.table_width - 12), color, bold) + "║")

    def map_row(self, text: str) -> str:
        return "║ " + self.fit(text.center(self.table_width - 2), self.table_width - 2) + " ║"

    def data_row(self, text: str) -> str:
        return "║ " + self.fit(text, self.table_width - 2) + " ║"

    def render(self) -> None:
        self._refresh_layout()
        cloud_ok = self.cloud_at is not None and time.monotonic() - self.cloud_at < 0.5
        camera_ok = self.camera_at is not None and time.monotonic() - self.camera_at < 0.5
        map_ok = self.map_at is not None and time.monotonic() - self.map_at < 0.5 and self.valid_cells > 0
        height_map_ok = (self.height_map_at is not None and
                         time.monotonic() - self.height_map_at < 0.5)
        odom_ok = self.odom_at is not None and time.monotonic() - self.odom_at < 0.5
        valid_ratio = 0.0 if self.total_cells == 0 else 100.0 * self.valid_cells / self.total_cells
        cloud_text = (f"{self.rate(self.cloud_times):.1f} Hz | {self.cloud_points} points | "
                      f"age {self.age(self.cloud_at)}")
        camera_text = (f"{self.rate(self.camera_times):.1f} Hz | {self.camera_points} points | "
                       f"age {self.age(self.camera_at)}") if self.camera_topic else "disabled"
        map_text = (f"{self.rate(self.map_times):.1f} Hz | target 50 Hz | frame {self.map_frame} | "
                    f"age {self.age(self.map_at)}")
        geometry = (f"{self.map_length_x:.2f} x {self.map_length_y:.2f} m | "
                    f"resolution {self.map_resolution:.3f} m") if math.isfinite(self.map_resolution) else "waiting for GridMap"
        coverage = f"{self.valid_cells}/{self.total_cells} valid cells ({valid_ratio:.1f}%)"
        height = (f"{self.height_low:+.3f} .. {self.height_high:+.3f} m"
                  if math.isfinite(self.height_low) else "waiting for finite elevation cells")
        uncertainty = (f"p95 range {self.uncertainty_p95:.3f} m" if math.isfinite(self.uncertainty_p95)
                       else "uncertainty layer not available yet")
        if self.height_map is None:
            height_map_text = "waiting for first /autonomy_light/height_map_data sample"
        else:
            expected = (round(self.height_map.x_length / self.height_map.resolution)
                        * round(self.height_map.y_length / self.height_map.resolution)
                        if self.height_map.resolution > 0.0 else 0)
            height_map_text = (f"{self.rate(self.height_map_times):.1f} Hz | "
                               f"{len(self.height_map.data)}/{expected} cells | "
                               f"age {self.age(self.height_map_at)}")
        if self.odom is None:
            slam = f"WAITING FOR {self.odom_topic} | mode {self.slam_mode}"
            position = attitude = velocity = "waiting for odometry"
        else:
            pose = self.odom.pose.pose.position
            linear = self.odom.twist.twist.linear
            angular = self.odom.twist.twist.angular
            speed = math.sqrt(linear.x ** 2 + linear.y ** 2 + linear.z ** 2)
            frame = self.odom.header.frame_id or "unknown"
            child_frame = self.odom.child_frame_id or "unknown"
            roll, pitch, yaw = self.roll_pitch_yaw_deg(self.odom)
            slam = (f"RUNNING | {self.slam_mode} | {self.rate(self.odom_times):.1f} Hz | "
                    f"age {self.age(self.odom_at)}")
            position = f"pos[{frame}] ({pose.x:+.3f}, {pose.y:+.3f}, {pose.z:+.3f}) m"
            attitude = f"rpy[{frame}->{child_frame}] ({roll:+.2f}, {pitch:+.2f}, {yaw:+.2f}) deg"
            velocity = (f"vel[{frame}] ({linear.x:+.3f}, {linear.y:+.3f}, {linear.z:+.3f}) m/s | "
                        f"speed {speed:.3f} m/s | yaw rate {angular.z:+.3f} rad/s")
        lines = [
            self.border("╔", "╗", "ELEVATION MAP / SLAM STATUS"),
            self.row("LIDAR", cloud_text, self.GREEN if cloud_ok else self.YELLOW, cloud_ok),
            self.row("D435", camera_text, self.GREEN if camera_ok else self.YELLOW, camera_ok),
            self.row("GRIDMAP", map_text, self.GREEN if map_ok else self.RED, map_ok),
            self.row("GEOMETRY", geometry),
            self.row("COVERAGE", coverage, self.GREEN if valid_ratio >= 25.0 else self.YELLOW),
            self.row("ELEVATION", height, self.GREEN if map_ok else self.YELLOW),
            self.row("UNCERTAINTY", uncertainty, self.GREEN if map_ok else self.YELLOW),
            self.border("╠", "╣", f"HEIGHT MAP DATA / {self.requested_rate_hz:g} HZ REDRAW"),
            self.row("HEIGHTMAP", height_map_text, self.GREEN if height_map_ok else self.YELLOW,
                     height_map_ok),
            self.map_row("meters; actual sampled values, positive downward from base_link | y rows: high to low"),
            *(self.data_row(row) for row in self.height_map_rows()),
            self.border("╠", "╣", "SUPER-LIO STATE"),
            self.row("SLAM", slam, self.GREEN if odom_ok else self.RED, odom_ok),
            self.row("POSITION", position),
            self.row("ATTITUDE", attitude),
            self.row("VELOCITY", velocity),
            "╚" + "═" * self.table_width + "╝",
        ]
        output = "\n".join(lines)
        if self.redraw:
            prefix = "\033[2J\033[H\033[?25l" if self._first_draw else "\033[H"
            self._first_draw = False
            self._cursor_hidden = True
            sys.stdout.write(prefix + output + "\033[J")
            sys.stdout.flush()
        else:
            print(output, flush=True)

    def close_display(self) -> None:
        if self._cursor_hidden:
            sys.stdout.write("\033[?25h\n")
            sys.stdout.flush()
            self._cursor_hidden = False


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--rate", type=positive_rate, default=50.0,
                        help="terminal redraw rate in Hz (default: 50.0)")
    parser.add_argument("--cloud-topic", default="/lio/cloud_world")
    parser.add_argument("--camera-topic", default="")
    parser.add_argument("--map-topic", default="/autonomy_light/elevation_map")
    parser.add_argument("--height-map-topic", default="/autonomy_light/height_map_data")
    parser.add_argument("--odom-topic", default="/lio/odom")
    parser.add_argument("--slam-mode", default="Super-LIO odometry")
    parser.add_argument("--color", choices=("auto", "always", "never"), default="auto")
    args = parser.parse_args()
    rclpy.init()
    node = ElevationStateMonitor(
        args.cloud_topic, args.camera_topic, args.map_topic, args.height_map_topic,
        args.odom_topic, args.slam_mode, args.rate, args.color)
    # launch.sh stops background children with SIGINT. Python can inherit an
    # ignored SIGINT from a non-interactive parent.  In Humble an executor can
    # remain blocked after rclpy.shutdown(), so restore the terminal then exit
    # directly instead of leaving a zombie monitor behind.
    def stop_signal(_signum, _frame) -> None:
        node.close_display()
        os._exit(0)

    signal.signal(signal.SIGINT, stop_signal)
    signal.signal(signal.SIGTERM, stop_signal)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception:
        # rclpy Humble can raise from a callback already queued when SIGTERM
        # shuts its context down.  This is a normal launcher cleanup path.
        if rclpy.ok():
            raise
    finally:
        node.close_display()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
