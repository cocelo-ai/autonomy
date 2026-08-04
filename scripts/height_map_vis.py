#!/usr/bin/env python3
"""Minimal live viewer for grid_map_msgs/GridMap elevation data."""

import argparse
import time

import cv2
import numpy as np
import rclpy
from grid_map_msgs.msg import GridMap
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy


class GridMapVis(Node):
    def __init__(self, topic):
        super().__init__("elevation_map_vis")
        self.latest = None
        self.received = 0
        self.shown = 0
        self.last_report = time.monotonic()
        self.create_subscription(
            GridMap, topic, self.on_map,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT),
        )

    def on_map(self, message):
        self.latest = message
        self.received += 1

    @staticmethod
    def elevation(message):
        if "elevation" not in message.layers:
            return None
        data = message.data[message.layers.index("elevation")]
        if len(data.layout.dim) != 2:
            return None
        shape = tuple(dim.size for dim in data.layout.dim)
        if not shape[0] or not shape[1] or len(data.data) != shape[0] * shape[1]:
            return None
        # grid_map stores Eigen matrices column-major and uses a circular
        # buffer.  Undo both conventions so adjacent pixels are adjacent cells.
        values = np.asarray(data.data, dtype=np.float32).reshape(shape, order="F")
        return np.roll(values, (-message.outer_start_index, -message.inner_start_index), axis=(0, 1))

    def render(self, values, scale, message):
        valid = np.isfinite(values)
        image = np.zeros((*values.shape, 3), dtype=np.uint8)
        if np.any(valid):
            low, high = np.percentile(values[valid], (2.0, 98.0))
            if high - low < 0.02:
                low, high = low - 0.01, high + 0.01
            normalized = np.zeros(values.shape, dtype=np.uint8)
            normalized[valid] = np.clip(255.0 * (values[valid] - low) / (high - low), 0, 255)
            image[valid] = cv2.applyColorMap(normalized, cv2.COLORMAP_TURBO)[valid]
        image = cv2.resize(image, (values.shape[1] * scale, values.shape[0] * scale), interpolation=cv2.INTER_NEAREST)
        for row in range(values.shape[0] + 1):
            cv2.line(image, (0, row * scale), (image.shape[1], row * scale), (55, 55, 55), 1)
        for column in range(values.shape[1] + 1):
            cv2.line(image, (column * scale, 0), (column * scale, image.shape[0]), (55, 55, 55), 1)
        if scale >= 42:
            for row, column in np.ndindex(values.shape):
                text = "--" if not valid[row, column] else f"{values[row, column]:+.2f}"
                color = (220, 220, 220) if valid[row, column] else (120, 120, 120)
                cv2.putText(image, text, (column * scale + 3, row * scale + scale // 2 + 5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.30, color, 1, cv2.LINE_AA)
        label = (f"elevation [m] frame={message.header.frame_id}  grid={values.shape[1]}x{values.shape[0]}  "
                 f"res={message.info.resolution:.3f} m  valid={np.count_nonzero(valid)}/{values.size}")
        image = cv2.copyMakeBorder(image, 30, 0, 0, 0, cv2.BORDER_CONSTANT, value=(25, 25, 25))
        cv2.putText(image, label, (8, 21), cv2.FONT_HERSHEY_SIMPLEX, 0.52, (230, 230, 230), 1, cv2.LINE_AA)
        return image


def main():
    parser = argparse.ArgumentParser(description="Live native GridMap elevation viewer.")
    parser.add_argument("--topic", default="/autonomy_light/elevation_map")
    parser.add_argument("--fps", type=float, default=50.0)
    parser.add_argument("--scale", type=int, default=48)
    args = parser.parse_args()

    rclpy.init()
    node = GridMapVis(args.topic)
    cv2.namedWindow("Elevation map", cv2.WINDOW_NORMAL)
    period = 1.0 / max(args.fps, 1.0)
    try:
        while rclpy.ok():
            started = time.monotonic()
            rclpy.spin_once(node, timeout_sec=0.0)
            if node.latest is not None:
                elevation = node.elevation(node.latest)
                if elevation is not None:
                    cv2.imshow("Elevation map", node.render(elevation, max(args.scale, 8), node.latest))
                    node.shown += 1
            key = cv2.waitKey(1) & 0xFF
            if key in (27, ord("q")):
                break
            now = time.monotonic()
            if now - node.last_report >= 2.0:
                elapsed = now - node.last_report
                print(f"GridMap viewer: source={node.received / elapsed:.1f} Hz display={node.shown / elapsed:.1f} Hz")
                node.received = node.shown = 0
                node.last_report = now
            time.sleep(max(0.0, period - (time.monotonic() - started)))
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
