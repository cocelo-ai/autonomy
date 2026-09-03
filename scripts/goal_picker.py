#!/usr/bin/env python3
"""Click a free cell in the live occupancy map to publish a Nav2 goal."""

import argparse
import math
import signal
import time

try:
    import cv2
    import numpy as np
except ImportError as error:
    raise SystemExit(
        "error: --goal-picker requires python3-opencv and python3-numpy") from error

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy._rclpy_pybind11 import RCLError
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


def positive_rate(value: str) -> float:
    rate = float(value)
    if not math.isfinite(rate) or rate <= 0.0:
        raise argparse.ArgumentTypeError("rate must be a positive finite number")
    return rate


class GoalPicker(Node):
    """A lightweight replacement for RViz's 2D Nav Goal tool."""

    _WINDOW = "Autonomy Light: click free space to set Nav2 goal"
    _CANVAS_WIDTH = 1100
    _CANVAS_HEIGHT = 840
    _MAP_TOP = 62
    _MAP_BOTTOM = 24
    _MAP_MARGIN = 18

    def __init__(self, rate_hz: float) -> None:
        super().__init__("goal_picker")
        self._map: OccupancyGrid | None = None
        self._odom: Odometry | None = None
        self._plan: Path | None = None
        self._goal: PoseStamped | None = None
        self._message = "Waiting for /map. Click a light free-space cell to send /goal_pose."
        self._message_color = (220, 220, 220)
        self._period = 1.0 / rate_hz
        self._last_render = 0.0
        self._open = True
        self._display_geometry: tuple[int, int, float, int, int] | None = None

        map_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, "/map", self._on_map, map_qos)
        self.create_subscription(Odometry, "/lio/odom", self._on_odom, 10)
        self.create_subscription(Path, "/plan", self._on_plan, 1)
        self._goal_publisher = self.create_publisher(PoseStamped, "/goal_pose", 10)

        cv2.namedWindow(self._WINDOW, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self._WINDOW, self._CANVAS_WIDTH, self._CANVAS_HEIGHT)
        cv2.setMouseCallback(self._WINDOW, self._on_mouse)

    def _on_map(self, message: OccupancyGrid) -> None:
        self._map = message

    def _on_odom(self, message: Odometry) -> None:
        self._odom = message

    def _on_plan(self, message: Path) -> None:
        self._plan = message

    @staticmethod
    def _yaw_from_quaternion(orientation: object) -> float:
        return math.atan2(
            2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
            1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z))

    def _world_to_pixel(self, x: float, y: float) -> tuple[int, int] | None:
        if self._map is None or self._display_geometry is None:
            return None
        offset_x, offset_y, scale, width, height = self._display_geometry
        resolution = self._map.info.resolution
        if resolution <= 0.0:
            return None
        column = (x - self._map.info.origin.position.x) / resolution
        row = (y - self._map.info.origin.position.y) / resolution
        if column < 0.0 or row < 0.0 or column >= width or row >= height:
            return None
        return (round(offset_x + (column + 0.5) * scale),
                round(offset_y + (height - row - 0.5) * scale))

    def _on_mouse(self, event: int, x: int, y: int, _flags: int, _userdata: object) -> None:
        if event != cv2.EVENT_LBUTTONDOWN or self._map is None or self._display_geometry is None:
            return
        offset_x, offset_y, scale, width, height = self._display_geometry
        column = int((x - offset_x) / scale)
        display_row = int((y - offset_y) / scale)
        row = height - 1 - display_row
        if column < 0 or row < 0 or column >= width or row >= height:
            self._message = "Click inside the occupancy map."
            self._message_color = (0, 210, 255)
            return
        occupancy = self._map.data[row * width + column]
        if occupancy != 0:
            kind = "unknown" if occupancy < 0 else "occupied"
            self._message = f"Rejected {kind} cell ({column}, {row}); choose a light free cell."
            self._message_color = (40, 80, 255)
            return
        goal_x = self._map.info.origin.position.x + (column + 0.5) * self._map.info.resolution
        goal_y = self._map.info.origin.position.y + (row + 0.5) * self._map.info.resolution
        yaw = 0.0
        if self._odom is not None and self._odom.header.frame_id == self._map.header.frame_id:
            robot = self._odom.pose.pose.position
            yaw = math.atan2(goal_y - robot.y, goal_x - robot.x)
        goal = PoseStamped()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = self._map.header.frame_id or "map"
        goal.pose.position.x = goal_x
        goal.pose.position.y = goal_y
        goal.pose.orientation.z = math.sin(yaw * 0.5)
        goal.pose.orientation.w = math.cos(yaw * 0.5)
        self._goal_publisher.publish(goal)
        self._goal = goal
        self._message = f"Published /goal_pose: x={goal_x:+.2f}, y={goal_y:+.2f}, yaw={math.degrees(yaw):+.1f} deg"
        self._message_color = (80, 235, 80)

    def _draw_robot(self, canvas: np.ndarray) -> None:
        if self._odom is None or self._map is None or self._odom.header.frame_id != self._map.header.frame_id:
            return
        position = self._odom.pose.pose.position
        pixel = self._world_to_pixel(position.x, position.y)
        if pixel is None:
            return
        yaw = self._yaw_from_quaternion(self._odom.pose.pose.orientation)
        tip = (round(pixel[0] + 16.0 * math.cos(yaw)),
               round(pixel[1] - 16.0 * math.sin(yaw)))
        cv2.circle(canvas, pixel, 7, (255, 220, 0), -1, cv2.LINE_AA)
        cv2.arrowedLine(canvas, pixel, tip, (255, 220, 0), 2, cv2.LINE_AA, tipLength=0.35)

    def _draw_plan(self, canvas: np.ndarray) -> None:
        if self._plan is None or self._map is None or self._plan.header.frame_id != self._map.header.frame_id:
            return
        pixels = [self._world_to_pixel(pose.pose.position.x, pose.pose.position.y)
                  for pose in self._plan.poses]
        pixels = [pixel for pixel in pixels if pixel is not None]
        if len(pixels) >= 2:
            cv2.polylines(canvas, [np.asarray(pixels, dtype=np.int32)], False,
                          (255, 100, 255), 2, cv2.LINE_AA)

    def _draw_goal(self, canvas: np.ndarray) -> None:
        if self._goal is None:
            return
        pixel = self._world_to_pixel(self._goal.pose.position.x, self._goal.pose.position.y)
        if pixel is None:
            return
        cv2.drawMarker(canvas, pixel, (70, 255, 70), cv2.MARKER_CROSS, 16, 2, cv2.LINE_AA)

    def _render(self) -> np.ndarray:
        canvas = np.full((self._CANVAS_HEIGHT, self._CANVAS_WIDTH, 3), (27, 27, 27), dtype=np.uint8)
        cv2.putText(canvas, "LIVE OCCUPANCY MAP  |  left-click a free cell: publish /goal_pose",
                    (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.63, (245, 245, 245), 2, cv2.LINE_AA)
        cv2.putText(canvas, "free: light  |  occupied: red  |  unknown: dark  |  cyan: robot  |  magenta: plan",
                    (20, 53), cv2.FONT_HERSHEY_SIMPLEX, 0.43, (190, 190, 190), 1, cv2.LINE_AA)
        if self._map is None or self._map.info.width == 0 or self._map.info.height == 0:
            cv2.putText(canvas, "Waiting for /map", (430, 430), cv2.FONT_HERSHEY_SIMPLEX,
                        0.8, (220, 220, 220), 2, cv2.LINE_AA)
            self._display_geometry = None
            return canvas

        width = int(self._map.info.width)
        height = int(self._map.info.height)
        data = np.asarray(self._map.data, dtype=np.int16)
        if data.size != width * height:
            cv2.putText(canvas, "Invalid OccupancyGrid geometry", (360, 430),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (40, 80, 255), 2, cv2.LINE_AA)
            self._display_geometry = None
            return canvas
        grid = data.reshape((height, width))
        image = np.full((height, width, 3), (72, 72, 72), dtype=np.uint8)
        image[grid == 0] = (232, 232, 232)
        image[(grid > 0) & (grid < 50)] = (180, 180, 180)
        image[grid >= 50] = (40, 55, 220)
        image = np.flipud(image)
        available_width = self._CANVAS_WIDTH - 2 * self._MAP_MARGIN
        available_height = self._CANVAS_HEIGHT - self._MAP_TOP - self._MAP_BOTTOM
        scale = min(available_width / width, available_height / height)
        rendered_width = max(1, round(width * scale))
        rendered_height = max(1, round(height * scale))
        offset_x = (self._CANVAS_WIDTH - rendered_width) // 2
        offset_y = self._MAP_TOP + (available_height - rendered_height) // 2
        rendered = cv2.resize(image, (rendered_width, rendered_height), interpolation=cv2.INTER_NEAREST)
        canvas[offset_y:offset_y + rendered_height, offset_x:offset_x + rendered_width] = rendered
        cv2.rectangle(canvas, (offset_x, offset_y),
                      (offset_x + rendered_width - 1, offset_y + rendered_height - 1),
                      (165, 165, 165), 1, cv2.LINE_AA)
        self._display_geometry = (offset_x, offset_y, scale, width, height)
        self._draw_plan(canvas)
        self._draw_goal(canvas)
        self._draw_robot(canvas)
        frame = self._map.header.frame_id or "map"
        cv2.putText(canvas, f"frame={frame}  resolution={self._map.info.resolution:.2f}m  "
                    f"size={width}x{height}  |  q: close", (20, self._CANVAS_HEIGHT - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, self._message_color, 1, cv2.LINE_AA)
        cv2.putText(canvas, self._message, (20, self._CANVAS_HEIGHT - 31),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.46, self._message_color, 1, cv2.LINE_AA)
        return canvas

    def render_if_due(self) -> None:
        now = time.monotonic()
        if now - self._last_render < self._period:
            return
        self._last_render = now
        cv2.imshow(self._WINDOW, self._render())

    def process_events(self) -> None:
        key = cv2.waitKey(1)
        if key in (27, ord("q"), ord("Q")):
            self._open = False

    @property
    def open(self) -> bool:
        return self._open


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--rate", type=positive_rate, default=10.0,
                        help="map rendering frequency in Hz (default: 10.0)")
    arguments = parser.parse_args()
    shutdown_requested = False

    def request_shutdown(_signum: int, _frame: object) -> None:
        nonlocal shutdown_requested
        shutdown_requested = True

    rclpy.init()
    signal.signal(signal.SIGINT, request_shutdown)
    signal.signal(signal.SIGTERM, request_shutdown)
    picker = GoalPicker(arguments.rate)
    try:
        while rclpy.ok() and picker.open and not shutdown_requested:
            rclpy.spin_once(picker, timeout_sec=min(0.01, 1.0 / arguments.rate))
            picker.render_if_due()
            picker.process_events()
    except KeyboardInterrupt:
        pass
    except RCLError as error:
        if "context is not valid" not in str(error):
            raise
    finally:
        picker.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
