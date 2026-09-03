#!/usr/bin/env python3
"""Built-in third-person renderer for the final ROS HeightMap."""

import argparse
import math
import signal
import time

try:
    import cv2
    import numpy as np
except ImportError as error:
    raise SystemExit(
        "error: --vis-height-3d requires python3-opencv and python3-numpy") from error

import rclpy
from rclpy._rclpy_pybind11 import RCLError
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from autonomy_light.msg import HeightMap


def positive_rate(value: str) -> float:
    try:
        rate = float(value)
    except ValueError as error:
        raise argparse.ArgumentTypeError("rate must be a positive number") from error
    if not math.isfinite(rate) or rate <= 0.0:
        raise argparse.ArgumentTypeError("rate must be a positive finite number")
    return rate


class ThirdPersonHeightMap(Node):
    """Renders HeightMap cells as a perspective mesh without RViz."""

    _WINDOW = "Elevation map 3D"
    _WIDTH = 1000
    _HEIGHT = 760

    def __init__(self, topic: str, rate: float) -> None:
        super().__init__("height_map_3d_visualizer")
        self._message: HeightMap | None = None
        self._received_at: float | None = None
        self._period = 1.0 / rate
        self._last_render = 0.0
        self._yaw = -0.85
        self._pitch = 0.58
        self._distance = 2.8
        self._open = True
        self.create_subscription(
            HeightMap, topic, self._on_height_map,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE))
        cv2.namedWindow(self._WINDOW, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self._WINDOW, self._WIDTH, self._HEIGHT)

    def _on_height_map(self, message: HeightMap) -> None:
        self._message = message
        self._received_at = time.monotonic()

    @staticmethod
    def _grid(message: HeightMap) -> tuple[np.ndarray, float] | None:
        resolution = float(message.resolution)
        if not math.isfinite(resolution) or resolution <= 0.0:
            return None
        x_cells = round(float(message.x_length) / resolution)
        y_cells = round(float(message.y_length) / resolution)
        if x_cells <= 0 or y_cells <= 0 or len(message.data) != x_cells * y_cells:
            return None
        # HeightMap is y-major. Its value means vertical distance below base,
        # so negate it for a conventional upward-positive 3D surface.
        return -np.asarray(message.data, dtype=np.float32).reshape((y_cells, x_cells)).T, resolution

    def _camera(self, target: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        direction = np.array([
            math.cos(self._pitch) * math.cos(self._yaw),
            math.cos(self._pitch) * math.sin(self._yaw),
            math.sin(self._pitch)], dtype=np.float32)
        eye = target + self._distance * direction
        forward = target - eye
        forward /= np.linalg.norm(forward)
        right = np.cross(forward, np.array([0.0, 0.0, 1.0], dtype=np.float32))
        right /= max(1.0e-6, np.linalg.norm(right))
        up = np.cross(right, forward)
        return eye, right, up, forward

    def _project(self, point: np.ndarray, camera: tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]) -> tuple[int, int, float] | None:
        eye, right, up, forward = camera
        relative = point - eye
        depth = float(np.dot(relative, forward))
        if depth <= 0.03:
            return None
        focal = 820.0
        x = int(self._WIDTH * 0.5 + focal * float(np.dot(relative, right)) / depth)
        y = int(self._HEIGHT * 0.53 - focal * float(np.dot(relative, up)) / depth)
        return x, y, depth

    @staticmethod
    def _color(value: float, lower: float, upper: float) -> tuple[int, int, int]:
        scaled = int(round(255.0 * np.clip((value - lower) / max(upper - lower, 1.0e-6), 0.0, 1.0)))
        return tuple(int(channel) for channel in cv2.applyColorMap(
            np.array([[scaled]], dtype=np.uint8), cv2.COLORMAP_TURBO)[0, 0])

    def _render(self, grid: np.ndarray, resolution: float, age: float) -> np.ndarray:
        canvas = np.full((self._HEIGHT, self._WIDTH, 3), (26, 26, 26), dtype=np.uint8)
        x_cells, y_cells = grid.shape
        x_extent = x_cells * resolution
        y_extent = y_cells * resolution
        valid = np.isfinite(grid)
        if not np.any(valid):
            cv2.putText(canvas, "Waiting for valid elevation cells", (280, 380),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.75, (230, 230, 230), 2, cv2.LINE_AA)
            return canvas
        lower = float(np.nanmin(grid))
        upper = float(np.nanmax(grid))
        target = np.array([0.0, 0.0, float(np.nanmean(grid))], dtype=np.float32)
        camera = self._camera(target)
        cells: list[tuple[float, np.ndarray, tuple[int, int, int]]] = []
        for x in range(x_cells - 1):
            for y in range(y_cells - 1):
                heights = grid[x:x + 2, y:y + 2]
                if not np.all(np.isfinite(heights)):
                    continue
                corners = np.array([
                    [-x_extent / 2 + x * resolution, -y_extent / 2 + y * resolution, heights[0, 0]],
                    [-x_extent / 2 + (x + 1) * resolution, -y_extent / 2 + y * resolution, heights[1, 0]],
                    [-x_extent / 2 + (x + 1) * resolution, -y_extent / 2 + (y + 1) * resolution, heights[1, 1]],
                    [-x_extent / 2 + x * resolution, -y_extent / 2 + (y + 1) * resolution, heights[0, 1]],
                ], dtype=np.float32)
                projected = [self._project(corner, camera) for corner in corners]
                if any(item is None for item in projected):
                    continue
                cells.append((sum(item[2] for item in projected) / 4.0,
                              np.asarray([(item[0], item[1]) for item in projected], dtype=np.int32),
                              self._color(float(np.mean(heights)), lower, upper)))
        for _, polygon, color in sorted(cells, reverse=True, key=lambda item: item[0]):
            cv2.fillConvexPoly(canvas, polygon, color, cv2.LINE_AA)
            cv2.polylines(canvas, [polygon], True, (45, 45, 45), 1, cv2.LINE_AA)
        cv2.putText(canvas, "FINAL ELEVATION MAP  |  third-person view", (24, 34),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.72, (245, 245, 245), 2, cv2.LINE_AA)
        cv2.putText(canvas, f"age {age:.3f}s  range {lower:.2f}..{upper:.2f}m  "
                    "arrow keys: orbit  +/-: zoom  q: close", (24, 66),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.48, (210, 210, 210), 1, cv2.LINE_AA)
        return canvas

    def render_if_due(self) -> None:
        now = time.monotonic()
        if now - self._last_render < self._period:
            return
        self._last_render = now
        if self._message is None:
            blank = np.full((self._HEIGHT, self._WIDTH, 3), (26, 26, 26), dtype=np.uint8)
            cv2.putText(blank, "Waiting for final elevation map", (310, 380),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.75, (230, 230, 230), 2, cv2.LINE_AA)
            cv2.imshow(self._WINDOW, blank)
            return
        decoded = self._grid(self._message)
        if decoded is None:
            return
        grid, resolution = decoded
        cv2.imshow(self._WINDOW, self._render(
            grid, resolution, max(0.0, now - (self._received_at or now))))

    def process_events(self) -> None:
        key = cv2.waitKey(1)
        if key in (27, ord("q"), ord("Q")):
            self._open = False
        elif key in (81, 2424832):
            self._yaw -= 0.10
        elif key in (83, 2555904):
            self._yaw += 0.10
        elif key in (82, 2490368):
            self._pitch = min(1.35, self._pitch + 0.08)
        elif key in (84, 2621440):
            self._pitch = max(0.08, self._pitch - 0.08)
        elif key in (ord("+"), ord("=")):
            self._distance = max(0.4, self._distance - 0.2)
        elif key in (ord("-"), ord("_")):
            self._distance = min(12.0, self._distance + 0.2)

    @property
    def open(self) -> bool:
        return self._open


def main() -> int:
    parser = argparse.ArgumentParser(description="Render the final HeightMap in a built-in 3D view.")
    parser.add_argument("--topic", default="/autonomy_light/elevation_map")
    parser.add_argument("--rate", type=positive_rate, default=50.0)
    args = parser.parse_args()
    shutdown_requested = False

    def request_shutdown(_signum: int, _frame: object) -> None:
        nonlocal shutdown_requested
        shutdown_requested = True

    rclpy.init()
    signal.signal(signal.SIGINT, request_shutdown)
    signal.signal(signal.SIGTERM, request_shutdown)
    viewer = ThirdPersonHeightMap(args.topic, args.rate)
    try:
        while rclpy.ok() and viewer.open and not shutdown_requested:
            rclpy.spin_once(viewer, timeout_sec=min(0.001, 1.0 / args.rate))
            viewer.render_if_due()
            viewer.process_events()
    except KeyboardInterrupt:
        pass
    except RCLError as error:
        if "context is not valid" not in str(error):
            raise
    finally:
        viewer.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
