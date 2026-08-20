#!/usr/bin/env python3
"""Generate a static horizontal-FOV mask shared by height-map outputs.

The mask deliberately makes no terrain-height or ground-plane assumption.  It
is a 2-D fan in the base-frame XY plane, centred at each camera's projected
position and pointing along its projected optical axis.
"""

import argparse
from collections import deque
from datetime import datetime
import math
from pathlib import Path
import re
import xml.etree.ElementTree as element_tree


# Keep the mask bounds with the camera model so callers cannot accidentally
# combine a camera frame with another model's FOV or usable depth range.
CAMERA_PROFILES = {
    "d435": {"horizontal_fov_deg": 87.0, "min_depth": 0.10, "max_depth": 2.50},
    "d435i": {"horizontal_fov_deg": 87.0, "min_depth": 0.10, "max_depth": 2.50},
    "d455": {"horizontal_fov_deg": 86.0, "min_depth": 0.20, "max_depth": 6.00},
}

# Intel RealSense D400 series minimum-Z table. In this intentionally 2-D
# approximation these values become conservative inner radial cutoffs; the
# launcher passes the selected driver profile, so YAML needs only camera_type.
MIN_DEPTH_BY_MODEL_AND_RESOLUTION = {
    "d435": {(1280, 720): 0.280, (848, 480): 0.195, (640, 480): 0.175,
             (640, 360): 0.150, (480, 270): 0.120, (424, 240): 0.105},
    "d435i": {(1280, 720): 0.280, (848, 480): 0.195, (640, 480): 0.175,
              (640, 360): 0.150, (480, 270): 0.120, (424, 240): 0.105},
    "d455": {(1280, 720): 0.520, (848, 480): 0.350, (640, 480): 0.320,
             (640, 360): 0.260, (480, 270): 0.200, (424, 240): 0.180},
}


def timestamped_output_path():
    """Allocate a chronologically sortable C++ runtime-mask filename."""
    output_directory = Path(__file__).resolve().parents[1] / "log" / "height_fov_masks"
    output_directory.mkdir(parents=True, exist_ok=True)
    stem = "height_fov_mask_" + datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    output = output_directory / f"{stem}.txt"
    suffix = 1
    while output.exists():
        output = output_directory / f"{stem}_{suffix:02d}.txt"
        suffix += 1
    return output


def print_python_mask(values):
    """Print a flat Python list matching a flattened row-major ray grid."""
    columns_per_line = 20
    print("MASK = [")
    for start in range(0, len(values), columns_per_line):
        print("    " + ", ".join(f"{float(value):.1f}" for value in values[start:start + columns_per_line]) + ",")
    print("]")


def normalize_camera_type(value):
    return str(value).strip().lower().replace("-", "").replace("_", "")


def resolution_from_depth_profile(value):
    match = re.fullmatch(r"\s*(\d+)x(\d+)(?:x\d+)?\s*", str(value))
    if not match:
        raise ValueError(f"invalid RealSense depth profile '{value}' (expected WIDTHxHEIGHT[xFPS])")
    return int(match.group(1)), int(match.group(2))


def profile_for_camera(camera_type, depth_profile):
    model = normalize_camera_type(camera_type)
    if model not in CAMERA_PROFILES:
        supported = ", ".join(CAMERA_PROFILES)
        raise ValueError(f"unsupported camera type '{camera_type}' (supported: {supported})")
    profile = dict(CAMERA_PROFILES[model])
    width, height = resolution_from_depth_profile(depth_profile)
    min_depths = MIN_DEPTH_BY_MODEL_AND_RESOLUTION[model]
    profile["min_depth"] = min_depths.get((width, height), profile["min_depth"])
    profile["model"] = model
    profile["depth_width"] = width
    profile["depth_height"] = height
    return profile


def matrix_multiply(left, right):
    return tuple(tuple(sum(left[row][index] * right[index][column] for index in range(3))
                       for column in range(3)) for row in range(3))


def matrix_vector(matrix, vector):
    return tuple(sum(matrix[row][index] * vector[index] for index in range(3))
                 for row in range(3))


def compose(left, right):
    left_rotation, left_translation = left
    right_rotation, right_translation = right
    rotated = matrix_vector(left_rotation, right_translation)
    return matrix_multiply(left_rotation, right_rotation), tuple(
        left_translation[index] + rotated[index] for index in range(3))


def inverse(transform):
    rotation, translation = transform
    inverse_rotation = tuple(tuple(rotation[column][row] for column in range(3))
                             for row in range(3))
    return inverse_rotation, tuple(-value for value in matrix_vector(inverse_rotation, translation))


def planar_camera_geometry(camera_from_base, camera_frame):
    """Return camera origin and optical-forward direction projected into base XY."""
    rotation, translation = camera_from_base
    base_from_camera_rotation = tuple(
        tuple(rotation[column][row] for column in range(3)) for row in range(3))
    origin = matrix_vector(base_from_camera_rotation, tuple(-value for value in translation))
    forward = matrix_vector(base_from_camera_rotation, (0.0, 0.0, 1.0))
    forward_norm = math.hypot(forward[0], forward[1])
    if forward_norm <= 1.0e-9:
        raise ValueError(
            f"camera frame '{camera_frame}' optical axis has no horizontal component")
    return origin[0], origin[1], forward[0] / forward_norm, forward[1] / forward_norm


def origin_transform(origin, label):
    try:
        xyz = tuple(float(value) for value in origin.attrib.get("xyz", "0 0 0").split())
        roll, pitch, yaw = (float(value) for value in origin.attrib.get("rpy", "0 0 0").split())
    except ValueError as error:
        raise ValueError(f"invalid origin in fixed joint '{label}'") from error
    if len(xyz) != 3 or not all(math.isfinite(value) for value in (*xyz, roll, pitch, yaw)):
        raise ValueError(f"invalid origin in fixed joint '{label}'")
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rotation = (
        (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
        (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
        (-sp, cp * sr, cp * cr),
    )
    return rotation, xyz


def base_to_camera_transforms(urdf_path, base_frame, camera_frames):
    root = element_tree.parse(urdf_path).getroot()
    links = {link.attrib["name"] for link in root.findall("link") if link.attrib.get("name")}
    if base_frame not in links:
        raise ValueError(f"base frame '{base_frame}' is not in {urdf_path}")
    adjacency = {link: [] for link in links}
    for joint in root.findall("joint"):
        if joint.attrib.get("type") != "fixed":
            continue
        parent = joint.find("parent")
        child = joint.find("child")
        if parent is None or child is None:
            continue
        parent_name = parent.attrib.get("link", "")
        child_name = child.attrib.get("link", "")
        if parent_name not in links or child_name not in links:
            raise ValueError("a fixed URDF joint references an unknown link")
        origin = joint.find("origin")
        if origin is None:
            origin = element_tree.Element("origin")
        transform = origin_transform(origin, joint.attrib.get("name", ""))
        adjacency[parent_name].append((child_name, transform))
        adjacency[child_name].append((parent_name, inverse(transform)))

    transforms = []
    identity = ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)), (0.0, 0.0, 0.0)
    for camera_frame in camera_frames:
        if camera_frame not in links:
            raise ValueError(f"camera frame '{camera_frame}' is not in {urdf_path}")
        queue = deque([(base_frame, identity)])
        visited = {base_frame}
        transform = None
        while queue:
            frame, base_from_frame = queue.popleft()
            if frame == camera_frame:
                transform = base_from_frame
                break
            for next_frame, frame_from_next in adjacency[frame]:
                if next_frame not in visited:
                    visited.add(next_frame)
                    queue.append((next_frame, compose(base_from_frame, frame_from_next)))
        if transform is None:
            raise ValueError(f"no fixed-joint path from '{base_frame}' to '{camera_frame}'")
        transforms.append(inverse(transform))  # camera_from_base
    return transforms


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--urdf", required=True, type=Path)
    parser.add_argument("--base-frame", required=True)
    parser.add_argument("--camera-frames", required=True,
                        help="comma-separated optical camera frame names")
    parser.add_argument("--resolution", required=True, type=float)
    parser.add_argument("--x-length", required=True, type=float)
    parser.add_argument("--y-length", required=True, type=float)
    parser.add_argument("--camera-type", required=True,
                        help="RealSense model: d435, d435i, or d455")
    parser.add_argument("--depth-profile", default="640x480",
                        help="selected driver profile WIDTHxHEIGHT[xFPS] (default: 640x480)")
    parser.add_argument("--runtime-mask", action="store_true",
                        help="internal: write the timestamped C++ FOV-mask file for launch.sh")
    args = parser.parse_args()
    try:
        camera_profile = profile_for_camera(args.camera_type, args.depth_profile)
    except ValueError as error:
        raise SystemExit(str(error)) from error
    horizontal_fov_deg = camera_profile["horizontal_fov_deg"]
    min_depth = camera_profile["min_depth"]
    max_depth = camera_profile["max_depth"]
    numeric = (args.resolution, args.x_length, args.y_length,
               horizontal_fov_deg, min_depth, max_depth)
    if not all(math.isfinite(value) for value in numeric) or args.resolution <= 0.0 or \
            args.x_length <= 0.0 or args.y_length <= 0.0 or min_depth < 0.0 or \
            max_depth <= min_depth or not 0.0 < horizontal_fov_deg < 180.0 or \
            not math.isfinite(min_depth) or not math.isfinite(max_depth):
        raise SystemExit("invalid FOV mask geometry or camera bounds")
    width = round(args.x_length / args.resolution)
    height = round(args.y_length / args.resolution)
    if width <= 0 or height <= 0 or not math.isclose(width * args.resolution, args.x_length,
                                                       abs_tol=1.0e-6) or \
            not math.isclose(height * args.resolution, args.y_length, abs_tol=1.0e-6):
        raise SystemExit("height-map dimensions must be integral multiples of resolution")
    camera_frames = [frame.strip() for frame in args.camera_frames.split(",") if frame.strip()]
    if not camera_frames:
        raise SystemExit("at least one camera frame is required")
    try:
        transforms = base_to_camera_transforms(args.urdf, args.base_frame, camera_frames)
        camera_geometries = [
            planar_camera_geometry(transform, frame)
            for transform, frame in zip(transforms, camera_frames)
        ]
    except ValueError as error:
        raise SystemExit(str(error)) from error
    h_limit = math.radians(horizontal_fov_deg) / 2.0
    values = []
    for row in range(height):
        y = -args.y_length / 2.0 + (row + 0.5) * args.resolution
        for column in range(width):
            x = -args.x_length / 2.0 + (column + 0.5) * args.resolution
            inside = False
            for camera_x, camera_y, forward_x, forward_y in camera_geometries:
                delta_x = x - camera_x
                delta_y = y - camera_y
                planar_range = math.hypot(delta_x, delta_y)
                signed_angle = math.atan2(
                    forward_x * delta_y - forward_y * delta_x,
                    forward_x * delta_x + forward_y * delta_y)
                if min_depth < planar_range < max_depth and abs(signed_angle) < h_limit:
                    inside = True
                    break
            values.append(1 if inside else 0)
    if not args.runtime_mask:
        print_python_mask(values)
        return
    output = timestamped_output_path()
    output.write_text(
        "AUTONOMY_LIGHT_FOV_MASK_V1\n"
        f"{width} {height} {args.resolution:.12g} {args.x_length:.12g} {args.y_length:.12g}\n" +
        " ".join(str(value) for value in values) + "\n", encoding="utf-8")
    print(f"FOV mask: {output} ({sum(values)}/{len(values)} in view; "
          f"camera_type={camera_profile['model']} depth={camera_profile['depth_width']}x"
          f"{camera_profile['depth_height']} planar_range={min_depth:.3f}..{max_depth:.3f}m; "
          f"frames={','.join(camera_frames)})")
    print(f"FOV_MASK_FILE={output}")


if __name__ == "__main__":
    main()
