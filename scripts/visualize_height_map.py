#!/usr/bin/env python3
"""Live side-by-side view of the DDS-bound height map and camera depth image."""

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
from sensor_msgs.msg import Image


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

    _WINDOW_NAME = "Height map and depth image"
    _MAX_MAP_WIDTH = 960
    _MAX_MAP_HEIGHT = 720
    _MAX_CELL_PIXELS = 60
    _HEADER_PIXELS = 75
    _FOOTER_PIXELS = 45
    _COLOR_BAR_WIDTH = 32
    _SIDE_PIXELS = 145
    _PANEL_GAP_PIXELS = 8
    _MAX_DEPTH_WIDTH = 960

    def __init__(self, topic: str, depth_topic: str, rate_hz: float,
                 minimum: float, maximum: float) -> None:
        super().__init__("height_map_visualizer")
        self._message: HeightMap | None = None
        self._received_at: float | None = None
        self._depth_message: Image | None = None
        self._depth_received_at: float | None = None
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
        self.create_subscription(
            Image, depth_topic, self._on_depth_image,
            # The image is a live visual reference.  Showing an old queue of
            # frames would make it misleading next to the current height map.
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        cv2.namedWindow(self._WINDOW_NAME, cv2.WINDOW_NORMAL)

    def _on_height_map(self, message: HeightMap) -> None:
        self._message = message
        self._received_at = time.monotonic()
        self._received_count += 1

    def _on_depth_image(self, message: Image) -> None:
        self._depth_message = message
        self._depth_received_at = time.monotonic()

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
        # Keep the entire physical map visible for any valid cell resolution.
        # The bridge message is authoritative for geometry; resolution only
        # changes the cell count, never whether the viewer can render it.
        cell = max(1, min(cls._MAX_CELL_PIXELS,
                          cls._MAX_MAP_WIDTH // y_cells,
                          cls._MAX_MAP_HEIGHT // x_cells))
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

        if cell >= 12:
            for row in range(x_cells + 1):
                y = top + row * cell
                cv2.line(canvas, (0, y), (map_width, y), (230, 230, 230), 1, cv2.LINE_AA)
            for column in range(y_cells + 1):
                x = column * cell
                cv2.line(canvas, (x, top), (x, top + map_height), (230, 230, 230), 1,
                         cv2.LINE_AA)

        if cell >= 38:
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
        cv2.putText(canvas, f"age {age:.3f} s | rx {receive_hz:.1f} Hz | draw {draw_hz:.1f} Hz | {x_cells}x{y_cells}",
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

    @staticmethod
    def _depth_meters(message: Image) -> np.ndarray | None:
        """Decode the uncompressed RealSense depth encodings into metres."""
        encoding = message.encoding.lower()
        if encoding in ("16uc1", "mono16"):
            dtype = np.dtype(">u2" if message.is_bigendian else "<u2")
            scale = 0.001  # RealSense 16UC1 depth is millimetres.
        elif encoding == "32fc1":
            dtype = np.dtype(">f4" if message.is_bigendian else "<f4")
            scale = 1.0
        else:
            return None
        item_size = dtype.itemsize
        row_bytes = message.width * item_size
        if message.height == 0 or message.width == 0 or message.step < row_bytes or \
                len(message.data) < message.step * message.height:
            return None
        # Image rows can have padding, so extract the usable bytes before
        # viewing them as scalar depth values.
        rows = np.frombuffer(message.data, dtype=np.uint8,
                             count=message.step * message.height).reshape(
                                 message.height, message.step)
        return rows[:, :row_bytes].copy().view(dtype).reshape(
            message.height, message.width).astype(np.float32) * scale

    @classmethod
    def _render_depth(cls, message: Image | None, frame_height: int,
                      content_height: int, age: float) -> np.ndarray:
        """Render a depth panel matched vertically to the height-map panel."""
        top = cls._HEADER_PIXELS
        canvas = np.full((frame_height, 640, 3), 32, dtype=np.uint8)
        if message is None:
            cv2.putText(canvas, "Waiting for depth image", (25, top + content_height // 2),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 1, cv2.LINE_AA)
            cv2.putText(canvas, "DEPTH IMAGE", (12, 24), cv2.FONT_HERSHEY_SIMPLEX,
                        0.60, (255, 255, 255), 1, cv2.LINE_AA)
            return canvas

        depth = cls._depth_meters(message)
        if depth is None:
            cv2.putText(canvas, "Unsupported depth image", (25, top + content_height // 2 - 14),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.60, (255, 255, 255), 1, cv2.LINE_AA)
            cv2.putText(canvas, message.encoding, (25, top + content_height // 2 + 18),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.54, (190, 190, 190), 1, cv2.LINE_AA)
            return canvas

        width = max(1, min(cls._MAX_DEPTH_WIDTH,
                           round(message.width * content_height / message.height)))
        canvas = np.full((frame_height, width, 3), 32, dtype=np.uint8)
        valid = np.isfinite(depth) & (depth > 0.0)
        if np.any(valid):
            # A robust per-frame upper bound keeps both near obstacles and
            # farther terrain legible without being dominated by outliers.
            sampled_depth = depth[::4, ::4]
            sampled_valid = np.isfinite(sampled_depth) & (sampled_depth > 0.0)
            depth_max = max(0.5, float(np.percentile(sampled_depth[sampled_valid], 99.0)))
            normalized = np.zeros_like(depth)
            np.divide(depth, depth_max, out=normalized, where=valid)
            np.clip(normalized, 0.0, 1.0, out=normalized)
            color_depth = cv2.applyColorMap(
                np.rint(normalized * 255.0).astype(np.uint8), cv2.COLORMAP_TURBO)
            color_depth[~valid] = (75, 75, 75)
            depth_image = cv2.resize(color_depth, (width, content_height),
                                     interpolation=cv2.INTER_NEAREST)
            range_label = f"0.00 - {depth_max:.2f} m"
        else:
            depth_image = np.full((content_height, width, 3), 75, dtype=np.uint8)
            range_label = "no valid depth"
        canvas[top:top + content_height, :] = depth_image
        cv2.putText(canvas, "DEPTH IMAGE", (12, 24), cv2.FONT_HERSHEY_SIMPLEX,
                    0.60, (255, 255, 255), 1, cv2.LINE_AA)
        cv2.putText(canvas,
                    f"age {age:.3f} s | {message.width}x{message.height} | {message.encoding}",
                    (12, 52), cv2.FONT_HERSHEY_SIMPLEX, 0.52, (210, 210, 210), 1, cv2.LINE_AA)
        cv2.putText(canvas, range_label, (12, frame_height - 13),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.48, (220, 220, 220), 1, cv2.LINE_AA)
        return canvas

    @classmethod
    def _side_by_side(cls, height_frame: np.ndarray, depth_frame: np.ndarray) -> np.ndarray:
        gap = np.full((height_frame.shape[0], cls._PANEL_GAP_PIXELS, 3), 18, dtype=np.uint8)
        return np.hstack((height_frame, gap, depth_frame))

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
        depth_frame = self._render_depth(
            self._depth_message, frame.shape[0],
            frame.shape[0] - self._HEADER_PIXELS - self._FOOTER_PIXELS,
            max(0.0, now - (self._depth_received_at or now)))
        cv2.imshow(self._WINDOW_NAME, self._side_by_side(frame, depth_frame))

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
        description="Display the calculated DDS height map beside the live camera depth image.")
    parser.add_argument("--topic", default="/autonomy_light/height_map_visualization")
    parser.add_argument("--depth-topic", default="/front_d435/front_d435/depth/image_rect_raw",
                        help="sensor_msgs/Image depth topic shown in the right panel")
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
    viewer = HeightMapVisualizer(args.topic, args.depth_topic, args.rate,
                                 args.minimum, args.maximum)
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
