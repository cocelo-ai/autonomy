#!/usr/bin/env python3
import argparse
import math
import threading
import time

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time

from autonomy_light.msg import HeightMap

try:
    import cv2
except ImportError as exc:
    raise SystemExit(
        "OpenCV Python bindings are required for --vis. "
        "Install them with: sudo apt install python3-opencv"
    ) from exc


class HeightMapVis(Node):
    def __init__(self, topic):
        super().__init__("autonomy_light_height_map_vis")
        # This is a live monitor, not a recorder. Keeping one best-effort sample
        # prevents a slow GUI from displaying stale terrain after the robot moves.
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._lock = threading.Lock()
        self._latest = None
        self._count = 0
        self.create_subscription(HeightMap, topic, self._on_height_map, qos)

    def _on_height_map(self, msg):
        with self._lock:
            self._latest = msg
            self._count += 1

    def latest(self):
        with self._lock:
            return self._latest, self._count


def parse_args():
    parser = argparse.ArgumentParser(description="Live 2D HeightMap.data visualizer.")
    parser.add_argument("--topic", default="/autonomy_light/height_map_data")
    parser.add_argument("--fps", type=float, default=50.0)
    parser.add_argument(
        "--scale",
        type=int,
        default=48,
        help="Pixel size of each grid cell. Default: 48.",
    )
    parser.add_argument("--window", default="autonomy_light height_map")
    parser.add_argument(
        "--inverse-colors",
        action="store_true",
        help="Invert colors only; numeric labels always show raw HeightMap.data.",
    )
    return parser.parse_args()


def grid_axis_cells(length, resolution):
    cells = length / resolution
    nearest = round(cells)
    return int(nearest) if abs(cells - nearest) < 1.0e-3 else int(math.ceil(cells))


def message_to_grid(msg, inverse_colors):
    if msg.resolution <= 0.0:
        return None, None, "invalid resolution"
    width = grid_axis_cells(msg.x_length, msg.resolution)
    height = grid_axis_cells(msg.y_length, msg.resolution)
    expected = width * height
    if width <= 0 or height <= 0 or len(msg.data) < expected:
        if len(msg.data) == 144:
            width = height = 12
            expected = 144
        else:
            return None, None, f"invalid grid {width}x{height} len={len(msg.data)}"

    raw = np.asarray(msg.data[:expected], dtype=np.float32).reshape((height, width))
    finite = np.isfinite(raw)
    if not finite.any():
        return None, None, f"{width}x{height} all invalid"

    low = float(np.nanmin(raw[finite]))
    high = float(np.nanmax(raw[finite]))
    span = max(1.0e-6, high - low)
    values = np.where(finite, raw, low)
    gray = np.clip((values - low) * (255.0 / span), 0.0, 255.0).astype(np.uint8)
    if inverse_colors:
        gray = 255 - gray

    # Input is y_min -> y_max by x_min -> x_max. Display is +x forward at top
    # and +y left, matching the robot-centric height-map convention.
    raw_oriented = np.flipud(np.fliplr(raw.T))
    gray_oriented = np.flipud(np.fliplr(gray.T))
    stats = f"{width}x{height} data=[{low:.3f},{high:.3f}]"
    return raw_oriented, gray_oriented, stats


def render_grid(raw, gray, cell_size):
    cell_size = max(28, int(cell_size))
    rows, cols = raw.shape
    colors = cv2.applyColorMap(gray, cv2.COLORMAP_TURBO)
    image = cv2.resize(colors, (cols * cell_size, rows * cell_size),
                       interpolation=cv2.INTER_NEAREST)

    font_scale = max(0.34, min(0.72, cell_size / 70.0))
    thickness = 1 if cell_size < 64 else 2
    for row in range(rows):
        for col in range(cols):
            x0 = col * cell_size
            y0 = row * cell_size
            value = raw[row, col]
            label = f"{value:.2f}" if np.isfinite(value) else "nan"
            (text_width, text_height), _ = cv2.getTextSize(
                label, cv2.FONT_HERSHEY_SIMPLEX, font_scale, thickness
            )
            x = x0 + max(2, (cell_size - text_width) // 2)
            y = y0 + (cell_size + text_height) // 2
            b, g, r = (int(channel) for channel in colors[row, col])
            luminance = 0.114 * b + 0.587 * g + 0.299 * r
            text_color = (0, 0, 0) if luminance > 145 else (255, 255, 255)
            cv2.putText(image, label, (x, y), cv2.FONT_HERSHEY_SIMPLEX,
                        font_scale, text_color, thickness, cv2.LINE_AA)

    for row in range(rows + 1):
        y = min(row * cell_size, image.shape[0] - 1)
        cv2.line(image, (0, y), (image.shape[1] - 1, y), (40, 40, 40), 1)
    for col in range(cols + 1):
        x = min(col * cell_size, image.shape[1] - 1)
        cv2.line(image, (x, 0), (x, image.shape[0] - 1), (40, 40, 40), 1)
    return image


def draw_overlay(grid_image, text):
    image = cv2.copyMakeBorder(grid_image, 52, 0, 0, 0,
                               cv2.BORDER_CONSTANT, value=(0, 0, 0))
    cv2.putText(image, text, (8, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                (255, 255, 255), 1, cv2.LINE_AA)
    cv2.putText(image, "+x forward: UP   +y: LEFT   labels: HeightMap.data [m]   q/ESC: quit",
                (8, 38), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                (255, 255, 255), 1, cv2.LINE_AA)
    return image


def message_age_ms(node, msg):
    stamp = Time.from_msg(msg.header.stamp)
    if stamp.nanoseconds == 0:
        return None
    return (node.get_clock().now().nanoseconds - stamp.nanoseconds) / 1.0e6


def spin_node(node):
    try:
        rclpy.spin(node)
    except ExternalShutdownException:
        pass


def main():
    args = parse_args()
    rclpy.init()
    node = HeightMapVis(args.topic)
    spin_thread = threading.Thread(target=spin_node, args=(node,), daemon=True)
    spin_thread.start()
    cv2.namedWindow(args.window, cv2.WINDOW_NORMAL)

    period = 1.0 / max(1.0, args.fps)
    last_count = display_count = 0
    last_rate_time = time.monotonic()
    source_hz = display_hz = render_ms = 0.0

    try:
        while rclpy.ok():
            start = time.monotonic()
            msg, count = node.latest()
            if msg is None:
                image = draw_overlay(np.zeros((320, 640, 3), dtype=np.uint8),
                                     f"waiting for {args.topic}")
            else:
                raw, gray, stats = message_to_grid(msg, args.inverse_colors)
                grid_image = (np.zeros((320, 640, 3), dtype=np.uint8) if raw is None
                              else render_grid(raw, gray, args.scale))
                elapsed = time.monotonic() - last_rate_time
                display_count += 1
                if elapsed >= 0.5:
                    source_hz = (count - last_count) / elapsed
                    display_hz = display_count / elapsed
                    last_count, display_count, last_rate_time = count, 0, time.monotonic()
                age_ms = message_age_ms(node, msg)
                age_text = "age=n/a" if age_ms is None else f"age={age_ms:5.1f}ms"
                render_ms = 0.85 * render_ms + 0.15 * ((time.monotonic() - start) * 1000.0)
                image = draw_overlay(
                    grid_image,
                    f"{args.topic} source={source_hz:4.1f}Hz display={display_hz:4.1f}Hz "
                    f"{age_text} render={render_ms:4.1f}ms {stats}",
                )

            cv2.imshow(args.window, image)
            if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
                break
            sleep_s = period - (time.monotonic() - start)
            if sleep_s > 0.0:
                time.sleep(sleep_s)
    finally:
        rclpy.shutdown()
        spin_thread.join(timeout=1.0)
        node.destroy_node()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
