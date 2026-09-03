#!/usr/bin/env python3
"""Restore the 18x8 elevation grid with the trained ONNX network."""

from pathlib import Path
from time import perf_counter

import numpy as np

import rclpy
from ament_index_python.packages import get_package_share_directory
from autonomy_light.msg import HeightMap
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Float32


MODEL_HEIGHT_INPUT = "noisy_elevation"
MODEL_OUTPUT = "elevation"
MODEL_CELL_COUNT = 144


class ElevationRestorer(Node):
    def __init__(self) -> None:
        super().__init__("elevation_restorer")

        self.enabled = bool(self.declare_parameter("enabled", True).value)
        self.output_mode = str(
            self.declare_parameter("output_mode", "restored").value
        ).strip().lower()
        self.model_path = str(
            self.declare_parameter("model_path", "elevation_restorer.onnx").value
        )
        self.input_topic = str(
            self.declare_parameter(
                "input_topic", "/autonomy_light/elevation_map/raw_input"
            ).value
        )
        self.output_topic = str(
            self.declare_parameter(
                "output_topic", "/autonomy_light/elevation_map"
            ).value
        )
        self.clip_enabled = bool(
            self.declare_parameter("output.clip_enabled", False).value
        )
        self.clip_min = float(self.declare_parameter("output.clip_min", 0.0).value)
        self.clip_max = float(self.declare_parameter("output.clip_max", 0.75).value)
        self.debug_enabled = bool(
            self.declare_parameter("debug.enabled", True).value
        )
        self.debug_topic = str(
            self.declare_parameter(
                "debug.pointcloud_topic",
                "/autonomy_light/elevation_map/debug_points",
            ).value
        )
        self.debug_frame = str(
            self.declare_parameter("debug.frame_id", "base_link_gravity").value
        )
        self.intra_op_threads = int(
            self.declare_parameter("inference.intra_op_threads", 1).value
        )

        if not self.enabled:
            self.get_logger().info("HeightMap output selector is disabled")
            return
        if self.output_mode not in ("raw", "restored"):
            raise ValueError("output_mode must be either 'raw' or 'restored'")
        if not self.input_topic or not self.output_topic:
            raise ValueError("HeightMap selector topics must not be empty")
        if self.clip_enabled and self.clip_min > self.clip_max:
            raise ValueError("output.clip_min must not exceed output.clip_max")
        if self.debug_enabled and (not self.debug_topic or not self.debug_frame):
            raise ValueError("debug PointCloud2 topic and frame must not be empty")
        if self.intra_op_threads < 1:
            raise ValueError("inference.intra_op_threads must be positive")

        self.session = None
        self.warning_times: dict[str, int] = {}

        resolved_model = None
        if self.output_mode == "restored":
            try:
                import onnxruntime as ort
            except ImportError as error:
                raise RuntimeError(
                    "restored output_mode requires the Python 'onnxruntime' package"
                ) from error
            resolved_model = self._resolve_model_path(self.model_path)
            options = ort.SessionOptions()
            options.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
            options.intra_op_num_threads = self.intra_op_threads
            options.inter_op_num_threads = 1
            self.session = ort.InferenceSession(
                str(resolved_model),
                sess_options=options,
                providers=["CPUExecutionProvider"],
            )
            self._validate_model_contract()

        reliable_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.output_publisher = self.create_publisher(
            HeightMap, self.output_topic, reliable_qos
        )
        self.inference_time_publisher = None
        if self.output_mode == "restored":
            self.inference_time_publisher = self.create_publisher(
                Float32,
                "/autonomy_light/elevation_restorer/inference_ms",
                reliable_qos,
            )
        self.debug_publisher = None
        if self.debug_enabled:
            self.debug_publisher = self.create_publisher(
                PointCloud2, self.debug_topic, reliable_qos
            )
        self.map_subscription = self.create_subscription(
            HeightMap, self.input_topic, self._on_height_map, reliable_qos
        )

        detail = f"ONNX model {resolved_model}" if resolved_model else "raw passthrough"
        self.get_logger().info(
            f"HeightMap output mode '{self.output_mode}': {self.input_topic} -> "
            f"{self.output_topic} ({detail})"
        )
        if self.debug_publisher is not None:
            self.get_logger().info(
                f"selected HeightMap debug PointCloud2: {self.debug_topic} "
                f"in {self.debug_frame}"
            )

    def _resolve_model_path(self, configured_path: str) -> Path:
        path = Path(configured_path).expanduser()
        candidates = [path] if path.is_absolute() else [Path.cwd() / path]
        if not path.is_absolute():
            share = Path(get_package_share_directory("autonomy_light"))
            candidates.extend(
                (share / "parameter" / "models" / path, share / path)
            )
        for candidate in candidates:
            if candidate.is_file():
                return candidate.resolve()
        checked = ", ".join(str(candidate) for candidate in candidates)
        raise FileNotFoundError(f"ONNX model not found; checked: {checked}")

    def _validate_model_contract(self) -> None:
        inputs = {item.name: item for item in self.session.get_inputs()}
        outputs = {item.name: item for item in self.session.get_outputs()}
        expected_inputs = {
            MODEL_HEIGHT_INPUT: [1, MODEL_CELL_COUNT],
        }
        if set(inputs) != set(expected_inputs):
            raise RuntimeError(
                f"unexpected ONNX inputs: expected {list(expected_inputs)}, "
                f"got {list(inputs)}"
            )
        for name, shape in expected_inputs.items():
            item = inputs.get(name)
            if item is None or item.type != "tensor(float)" or item.shape != shape:
                actual = None if item is None else (item.type, item.shape)
                raise RuntimeError(
                    f"unexpected ONNX input {name}: expected tensor(float) {shape}, got {actual}"
                )
        output = outputs.get(MODEL_OUTPUT)
        if (
            output is None
            or output.type != "tensor(float)"
            or output.shape != [1, MODEL_CELL_COUNT]
        ):
            actual = None if output is None else (output.type, output.shape)
            raise RuntimeError(
                f"unexpected ONNX output {MODEL_OUTPUT}: "
                f"expected tensor(float) [1, {MODEL_CELL_COUNT}], got {actual}"
            )

    def _warn_throttled(self, key: str, text: str, period_sec: float = 2.0) -> None:
        now_ns = self.get_clock().now().nanoseconds
        last_ns = self.warning_times.get(key)
        if last_ns is None or now_ns - last_ns >= int(period_sec * 1.0e9):
            self.get_logger().warning(text)
            self.warning_times[key] = now_ns

    def _on_height_map(self, message: HeightMap) -> None:
        if len(message.data) != MODEL_CELL_COUNT:
            self._warn_throttled(
                "shape",
                f"height map has {len(message.data)} cells; output requires "
                f"{MODEL_CELL_COUNT}",
            )
            return
        elevation = np.asarray(message.data, dtype=np.float32).reshape(-1)
        if not np.all(np.isfinite(elevation)):
            self._warn_throttled("map", "height map contains non-finite values")
            return

        if self.output_mode == "raw":
            self._publish_selected(elevation, message)
            return

        start = perf_counter()
        restored = self.session.run(
            [MODEL_OUTPUT],
            {MODEL_HEIGHT_INPUT: elevation.reshape(1, -1)},
        )[0].reshape(-1)
        elapsed_ms = (perf_counter() - start) * 1000.0
        if restored.size != MODEL_CELL_COUNT or not np.all(np.isfinite(restored)):
            self._warn_throttled("output", "ONNX restorer returned invalid output")
            return
        if self.clip_enabled:
            restored = np.clip(restored, self.clip_min, self.clip_max)

        self._publish_selected(restored, message)

        if self.inference_time_publisher is None:
            return
        timing_message = Float32()
        timing_message.data = float(elapsed_ms)
        self.inference_time_publisher.publish(timing_message)

    def _publish_selected(
        self, selected: np.ndarray, geometry: HeightMap
    ) -> None:
        output = HeightMap()
        output.data = selected.astype(np.float32).tolist()
        output.resolution = geometry.resolution
        output.x_length = geometry.x_length
        output.y_length = geometry.y_length
        self.output_publisher.publish(output)
        self._publish_debug_cloud(selected, geometry)

    def _publish_debug_cloud(
        self, selected: np.ndarray, geometry: HeightMap
    ) -> None:
        if self.debug_publisher is None:
            return
        resolution = float(geometry.resolution)
        x_length = float(geometry.x_length)
        y_length = float(geometry.y_length)
        if resolution <= 0.0 or x_length <= 0.0 or y_length <= 0.0:
            self._warn_throttled(
                "debug_geometry", "cannot create selected debug cloud: invalid grid geometry"
            )
            return
        width = int(round(x_length / resolution))
        height = int(round(y_length / resolution))
        if width <= 0 or height <= 0 or width * height != selected.size:
            self._warn_throttled(
                "debug_shape",
                f"cannot create selected debug cloud: geometry is {width}x{height}, "
                f"but output contains {selected.size} cells",
            )
            return

        x = -0.5 * x_length + (np.arange(width, dtype=np.float32) + 0.5) * resolution
        y = -0.5 * y_length + (np.arange(height, dtype=np.float32) + 0.5) * resolution
        points = np.empty((selected.size, 4), dtype="<f4")
        points[:, 0] = np.tile(x, height)
        points[:, 1] = np.repeat(y, width)
        # HeightMap values are positive target-to-surface distances. The debug
        # surface is therefore below the target-frame origin at z=-distance.
        points[:, 2] = -selected
        points[:, 3] = selected

        cloud = PointCloud2()
        cloud.header.stamp = self.get_clock().now().to_msg()
        cloud.header.frame_id = self.debug_frame
        cloud.height = 1
        cloud.width = selected.size
        cloud.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(
                name="intensity", offset=12, datatype=PointField.FLOAT32, count=1
            ),
        ]
        cloud.is_bigendian = False
        cloud.point_step = 16
        cloud.row_step = cloud.point_step * cloud.width
        cloud.data = points.tobytes()
        cloud.is_dense = True
        self.debug_publisher.publish(cloud)


def main(args=None) -> int:
    rclpy.init(args=args)
    node = None
    try:
        node = ElevationRestorer()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
