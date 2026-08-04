#!/usr/bin/env python3
"""ANSI terminal dashboard for pose, velocity, and sampled height-map data."""

import argparse
import math
import sys
import time
from collections import deque

import rclpy
from autonomy_light.msg import HeightMap
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy


class TelemetryVis(Node):
    def __init__(self, odom_topic: str, height_map_topic: str) -> None:
        super().__init__("autonomy_light_telemetry_vis")
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.odom = None
        self.height_map = None
        self.odom_received = None
        self.height_received = None
        self.odom_times = deque()
        self.height_times = deque()
        self.create_subscription(Odometry, odom_topic, self.on_odom, qos)
        self.create_subscription(HeightMap, height_map_topic, self.on_height_map, qos)
        self.odom_topic = odom_topic
        self.height_map_topic = height_map_topic

    def on_odom(self, message: Odometry) -> None:
        now = time.monotonic()
        self.odom = message
        self.odom_received = now
        self.odom_times.append(now)

    def on_height_map(self, message: HeightMap) -> None:
        now = time.monotonic()
        self.height_map = message
        self.height_received = now
        self.height_times.append(now)

    @staticmethod
    def rate(samples: deque, now: float) -> float:
        while samples and now - samples[0] > 1.0:
            samples.popleft()
        return float(len(samples))


def yaw_from_quaternion(quaternion) -> float:
    sin_yaw = 2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y)
    cos_yaw = 1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z)
    return math.atan2(sin_yaw, cos_yaw)


def age_text(received: float | None, now: float) -> str:
    return "waiting" if received is None else f"{now - received:0.3f}s"


def color_cell(text: str, value: float, low: float, high: float, enabled: bool) -> str:
    if not enabled or not math.isfinite(value):
        return text
    normalized = 0.5 if high <= low else min(1.0, max(0.0, (value - low) / (high - low)))
    # 21=blue through 196=red in the xterm 256-color palette.
    background = int(round(21 + normalized * (196 - 21)))
    foreground = 15 if normalized < 0.62 else 16
    return f"\033[38;5;{foreground};48;5;{background}m{text}\033[0m"


def render_height_map(message: HeightMap | None, color: bool) -> list[str]:
    if message is None:
        return ["height_map_data: waiting for first sample"]
    if message.resolution <= 0.0 or message.x_length <= 0.0 or message.y_length <= 0.0:
        return ["height_map_data: invalid geometry"]

    width = max(1, round(message.x_length / message.resolution))
    height = max(1, round(message.y_length / message.resolution))
    values = list(message.data)
    expected = width * height
    finite = [value for value in values if math.isfinite(value)]
    low = min(finite) if finite else 0.0
    high = max(finite) if finite else 0.0
    lines = [
        "height_map_data  "
        f"shape={width}x{height}  res={message.resolution:.3f}m  "
        f"roi={message.x_length:.2f}x{message.y_length:.2f}m  "
        f"samples={len(values)}/{expected}  range={low:.3f}..{high:.3f}m",
        "values: positive is downward distance from base_link; PointCloud2 z = -value",
    ]
    if len(values) != expected:
        lines.append("warning: sample count does not match declared grid geometry")
    for row in range(height):
        cells = []
        for column in range(width):
            index = row * width + column
            value = values[index] if index < len(values) else math.nan
            text = "  nan" if not math.isfinite(value) else f" {value:0.2f}"
            cells.append(color_cell(text, value, low, high, color))
        lines.append(f"y={row:02d} " + "".join(cells))
    return lines


def render_dashboard(node: TelemetryVis, color: bool) -> str:
    now = time.monotonic()
    odom_hz = node.rate(node.odom_times, now)
    height_hz = node.rate(node.height_times, now)
    lines = [
        "\033[2J\033[H"
        "Autonomy-light telemetry  (Ctrl-C to quit)",
        f"odom: {node.odom_topic}  {odom_hz:5.1f} Hz  age={age_text(node.odom_received, now)}",
        f"height: {node.height_map_topic}  {height_hz:5.1f} Hz  age={age_text(node.height_received, now)}",
    ]
    if node.odom is None:
        lines.append("position / velocity: waiting for /lio/odom")
    else:
        pose = node.odom.pose.pose
        linear = node.odom.twist.twist.linear
        angular = node.odom.twist.twist.angular
        speed = math.sqrt(linear.x * linear.x + linear.y * linear.y + linear.z * linear.z)
        lines.extend([
            "",
            f"position [m]    x={pose.position.x:+.3f}  y={pose.position.y:+.3f}  z={pose.position.z:+.3f}  "
            f"yaw={math.degrees(yaw_from_quaternion(pose.orientation)):+.1f} deg",
            f"velocity [m/s]  x={linear.x:+.3f}  y={linear.y:+.3f}  z={linear.z:+.3f}  |v|={speed:.3f}  "
            f"yaw_rate={angular.z:+.3f} rad/s",
            "",
        ])
    lines.extend(render_height_map(node.height_map, color))
    return "\n".join(lines) + "\n"


def parse_args():
    parser = argparse.ArgumentParser(description="Live ROS 2 terminal telemetry dashboard")
    parser.add_argument("--odom-topic", default="/lio/odom")
    parser.add_argument("--height-map-topic", default="/autonomy_light/height_map_data")
    parser.add_argument("--fps", type=float, default=50.0)
    parser.add_argument("--no-color", action="store_true")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    if args.fps <= 0.0:
        raise SystemExit("--fps must be positive")
    rclpy.init()
    node = TelemetryVis(args.odom_topic, args.height_map_topic)
    period = 1.0 / args.fps
    try:
        while rclpy.ok():
            started = time.monotonic()
            # spin_once handles one callback. Drain a bounded batch so the
            # 50 Hz odometry and 50 Hz height-map streams are both observed
            # rather than alternating at roughly half rate.
            for _ in range(16):
                rclpy.spin_once(node, timeout_sec=0.0)
            sys.stdout.write(render_dashboard(node, not args.no_color))
            sys.stdout.flush()
            time.sleep(max(0.0, period - (time.monotonic() - started)))
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
