#!/usr/bin/env python3
"""Fast live 2D view of the exact 144-cell height-map payload sent to DDS."""

import argparse
import math
import signal
import time

try:
    import cv2
    import numpy as np
except ImportError as error:
    raise SystemExit(
        "error: --vis-height requires python3-opencv and python3-numpy; "
        "run ./build.sh or install those packages") from error

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


class HeightMapVisualizer(Node):
    """Shows the latest DDS-bound sample; no old frames are queued for display."""

    _WINDOW_NAME = "DDS height map (2D)"
    _CELL_PIXELS = 60
    _HEADER_PIXELS = 75
    _FOOTER_PIXELS = 45
    _COLOR_BAR_WIDTH = 32
    _SIDE_PIXELS = 145

    def __init__(self, topic: str, rate_hz: float, minimum: float, maximum: float) -> None:
        super().__init__("height_map_visualizer")
        self._message: HeightMap | None = None
        self._received_at: float | None = None
        self._minimum = minimum
        self._maximum = maximum
        self._last_render = 0.0
        self._period = 1.0 / rate_hz
        self._received_count = 0
        self._rendered_count = 0
        self._started_at = time.monotonic()
        self._window_open = True
        self.create_subscription(
            HeightMap, topic, self._on_height_map,
            # Keep only the latest bridge output: rendering never trails the
            # 50 Hz calculation by replaying a backlog of old samples.
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE))
        cv2.namedWindow(self._WINDOW_NAME, cv2.WINDOW_NORMAL)

    def _on_height_map(self, message: HeightMap) -> None:
        self._message = message
        self._received_at = time.monotonic()
        self._received_count += 1

    @staticmethod
    def _put_centered(image: np.ndarray, text: str, center: tuple[int, int],
                      scale: float, color: tuple[int, int, int], thickness: int = 1) -> None:
        size, baseline = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, scale, thickness)
        origin = (center[0] - size[0] // 2, center[1] + (size[1] - baseline) // 2)
        cv2.putText(image, text, origin, cv2.FONT_HERSHEY_SIMPLEX, scale, color,
                    thickness, cv2.LINE_AA)

    @classmethod
    def _render_map(cls, grid: np.ndarray, minimum: float, maximum: float,
                    age: float, receive_hz: float, draw_hz: float) -> np.ndarray:
        """Create one inexpensive OpenCV frame, including all cell values."""
        x_cells, y_cells = grid.shape
        cell = cls._CELL_PIXELS
        map_height = x_cells * cell
        map_width = y_cells * cell
        canvas_height = cls._HEADER_PIXELS + map_height + cls._FOOTER_PIXELS
        canvas_width = map_width + cls._SIDE_PIXELS
        canvas = np.full((canvas_height, canvas_width, 3), 32, dtype=np.uint8)

        # Input data is x-major after DDS payload normalization. OpenCV's
        # first row is top, so reverse x: +x is up and +y is right.
        displayed = grid[::-1, :]
        normalized = np.clip((displayed - minimum) / (maximum - minimum), 0.0, 1.0)
        color_cells = cv2.applyColorMap(
            np.rint(normalized * 255.0).astype(np.uint8), cv2.COLORMAP_TURBO)
        color_cells[~np.isfinite(displayed)] = (75, 75, 75)
        map_image = cv2.resize(color_cells, (map_width, map_height),
                               interpolation=cv2.INTER_NEAREST)
        top = cls._HEADER_PIXELS
        canvas[top:top + map_height, :map_width] = map_image

        for row in range(x_cells + 1):
            y = top + row * cell
            cv2.line(canvas, (0, y), (map_width, y), (230, 230, 230), 1, cv2.LINE_AA)
        for column in range(y_cells + 1):
            x = column * cell
            cv2.line(canvas, (x, top), (x, top + map_height), (230, 230, 230), 1,
                     cv2.LINE_AA)

        for display_row in range(x_cells):
            source_x = x_cells - 1 - display_row
            for y_index in range(y_cells):
                value = float(grid[source_x, y_index])
                center = (y_index * cell + cell // 2, top + display_row * cell + cell // 2)
                if not math.isfinite(value):
                    cls._put_centered(canvas, "--", center, 0.55, (255, 255, 255), 1)
                    continue
                background = map_image[display_row * cell + cell // 2,
                                       y_index * cell + cell // 2]
                luminance = (0.114 * float(background[0]) + 0.587 * float(background[1]) +
                             0.299 * float(background[2]))
                cls._put_centered(canvas, f"{value:.2f}", center, 0.48,
                                  (0, 0, 0) if luminance > 135.0 else (255, 255, 255), 1)

        bar_x = map_width + 26
        gradient = np.linspace(maximum, minimum, map_height, dtype=np.float32).reshape(-1, 1)
        gradient_normalized = np.rint(
            np.clip((gradient - minimum) / (maximum - minimum), 0.0, 1.0) * 255.0).astype(np.uint8)
        color_bar = cv2.applyColorMap(gradient_normalized, cv2.COLORMAP_TURBO)
        color_bar = cv2.resize(color_bar, (cls._COLOR_BAR_WIDTH, map_height),
                               interpolation=cv2.INTER_NEAREST)
        canvas[top:top + map_height, bar_x:bar_x + cls._COLOR_BAR_WIDTH] = color_bar
        cv2.rectangle(canvas, (bar_x, top), (bar_x + cls._COLOR_BAR_WIDTH, top + map_height),
                      (230, 230, 230), 1)
        for label, y in ((f"{maximum:.2f}", top + 12),
                         (f"{(minimum + maximum) / 2.0:.2f}", top + map_height // 2),
                         (f"{minimum:.2f}", top + map_height - 4)):
            cv2.putText(canvas, label, (bar_x + cls._COLOR_BAR_WIDTH + 8, y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.42, (240, 240, 240), 1, cv2.LINE_AA)

        cv2.putText(canvas, "DDS OUTPUT HEIGHT MAP   (+x UP, +y RIGHT)", (12, 24),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.60, (255, 255, 255), 1, cv2.LINE_AA)
        cv2.putText(canvas, f"age {age:.3f} s | rx {receive_hz:.1f} Hz | draw {draw_hz:.1f} Hz",
                    (12, 52), cv2.FONT_HERSHEY_SIMPLEX, 0.52, (210, 210, 210), 1, cv2.LINE_AA)
        cv2.putText(canvas, "+x", (6, top + 22), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                    (255, 255, 255), 1, cv2.LINE_AA)
        cv2.putText(canvas, "-x", (6, top + map_height - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                    (255, 255, 255), 1, cv2.LINE_AA)
        cv2.putText(canvas, "-y", (6, canvas_height - 13), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                    (255, 255, 255), 1, cv2.LINE_AA)
        cv2.putText(canvas, "+y", (map_width - 34, canvas_height - 13),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1, cv2.LINE_AA)
        return canvas

    def render_if_due(self) -> None:
        now = time.monotonic()
        if now - self._last_render < self._period:
            return
        self._last_render = now
        message = self._message
        if message is None:
            waiting = np.full((160, 520, 3), 32, dtype=np.uint8)
            cv2.putText(waiting, "Waiting for calculated DDS height map", (25, 85),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 1, cv2.LINE_AA)
            cv2.imshow(self._WINDOW_NAME, waiting)
            return

        resolution = float(message.resolution)
        x_cells = max(1, round(float(message.x_length) / resolution)) if resolution > 0.0 else 0
        y_cells = max(1, round(float(message.y_length) / resolution)) if resolution > 0.0 else 0
        if not math.isfinite(resolution) or x_cells <= 0 or y_cells <= 0 or \
                len(message.data) != x_cells * y_cells:
            self.get_logger().warning("Ignoring DDS height map with invalid geometry")
            return

        # The DDS array is row(y)-major with x advancing inside each row.
        grid = np.asarray(message.data, dtype=np.float32).reshape((y_cells, x_cells)).T
        self._rendered_count += 1
        elapsed = max(1.0e-6, now - self._started_at)
        frame = self._render_map(
            grid, self._minimum, self._maximum,
            max(0.0, now - (self._received_at or now)),
            self._received_count / elapsed, self._rendered_count / elapsed)
        cv2.imshow(self._WINDOW_NAME, frame)

    def process_window_events(self) -> None:
        key = cv2.waitKey(1)
        if key in (27, ord("q"), ord("Q")):
            self._window_open = False

    def window_open(self) -> bool:
        return self._window_open


def main() -> int:
    shutdown_requested = False

    def request_shutdown(_signum: int, _frame: object) -> None:
        nonlocal shutdown_requested
        shutdown_requested = True

    parser = argparse.ArgumentParser(
        description="Display the calculated 2D height-map payload sent to DDS.")
    parser.add_argument("--topic", default="/autonomy_light/height_map_visualization")
    parser.add_argument("--rate", type=positive_rate, default=50.0,
                        help="render rate in Hz (normally elevation_mapping.publish_rate_hz)")
    parser.add_argument("--min", dest="minimum", type=float, default=0.0)
    parser.add_argument("--max", dest="maximum", type=float, default=0.75)
    args = parser.parse_args()
    if not (math.isfinite(args.minimum) and math.isfinite(args.maximum) and
            args.minimum < args.maximum):
        parser.error("--min and --max must be finite and --min must be below --max")

    rclpy.init()
    # Handle launcher shutdown ourselves.  rclpy's default signal handler can
    # invalidate its context while spin_once() is allocating a wait-set,
    # producing a spurious traceback during an otherwise normal exit.
    signal.signal(signal.SIGINT, request_shutdown)
    signal.signal(signal.SIGTERM, request_shutdown)
    viewer = HeightMapVisualizer(args.topic, args.rate, args.minimum, args.maximum)
    try:
        while rclpy.ok() and viewer.window_open() and not shutdown_requested:
            rclpy.spin_once(viewer, timeout_sec=min(0.001, 1.0 / args.rate))
            viewer.render_if_due()
            viewer.process_window_events()
    except KeyboardInterrupt:
        pass
    except RCLError as error:
        # rclpy's SIGINT/SIGTERM handler can invalidate the context while the
        # executor is waiting. This is a normal shutdown, not a visualizer
        # failure. Other RCLError instances still indicate an actual problem.
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
