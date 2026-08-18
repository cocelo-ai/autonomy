#!/usr/bin/env python3
"""Visualize every connected RealSense RGB stream with its USB port ID."""

import argparse
import re
import sys
import time


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Open one labeled RGB window for every connected RealSense camera."
    )
    parser.add_argument("--width", type=int, default=640, help="RGB stream width (default: 640).")
    parser.add_argument("--height", type=int, default=480, help="RGB stream height (default: 480).")
    parser.add_argument("--fps", type=int, default=30, help="RGB stream rate (default: 30).")
    return parser.parse_args()


def import_runtime_modules():
    try:
        import cv2
        import numpy as np
        import pyrealsense2 as rs
    except ModuleNotFoundError as error:
        print(
            f"Missing Python module: {error.name}\n"
            "Install OpenCV and pyrealsense2 before running this tool.\n"
            "Example: sudo apt install python3-opencv",
            file=sys.stderr,
        )
        raise SystemExit(2) from error
    return cv2, np, rs


def safe_device_info(device, info) -> str:
    try:
        return device.get_info(info)
    except RuntimeError:
        return ""


def usb_port_id(physical_port: str) -> str:
    matches = re.findall(r"(?<![\w.])\d+-\d+(?:\.\d+)*(?=[:/]|$)", physical_port or "")
    if matches:
        return matches[-1]
    parts = [part for part in (physical_port or "").split("/") if part]
    return parts[-1].split(":")[0] if parts else ""


def connected_devices(rs):
    devices = []
    for device in rs.context().query_devices():
        physical_port = safe_device_info(device, rs.camera_info.physical_port)
        devices.append(
            {
                "serial": safe_device_info(device, rs.camera_info.serial_number),
                "name": safe_device_info(device, rs.camera_info.name),
                "physical_port": physical_port,
                "usb_port_id": usb_port_id(physical_port),
            }
        )
    return devices


def start_pipeline(rs, serial: str, width: int, height: int, fps: int):
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_device(serial)
    config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)
    pipeline.start(config)
    return pipeline


def draw_label(cv2, image, device):
    output = image.copy()
    cv2.rectangle(output, (8, 8), (632, 84), (0, 0, 0), thickness=-1)
    cv2.putText(
        output,
        f"USB: {device['usb_port_id'] or 'unknown'}  {device['name']}",
        (20, 40),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.70,
        (0, 255, 255),
        2,
        cv2.LINE_AA,
    )
    cv2.putText(
        output,
        f"Serial: {device['serial'] or 'unknown'}",
        (20, 70),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.60,
        (255, 255, 255),
        1,
        cv2.LINE_AA,
    )
    return output


def main() -> int:
    args = parse_args()
    if args.width <= 0 or args.height <= 0 or args.fps <= 0:
        print("--width, --height, and --fps must be positive.", file=sys.stderr)
        return 2
    cv2, np, rs = import_runtime_modules()
    devices = connected_devices(rs)
    if not devices:
        print("No RealSense devices found.", file=sys.stderr)
        return 1

    print("Connected RealSense devices:")
    pipelines = []
    for device in devices:
        print(
            f"  usb_port_id={device['usb_port_id']} "
            f"serial={device['serial']} model={device['name']}"
        )
        if device["physical_port"]:
            print(f"    physical_port={device['physical_port']}")
        try:
            pipelines.append(
                (device, start_pipeline(rs, device["serial"], args.width, args.height, args.fps))
            )
        except RuntimeError as error:
            print(f"  Skipping {device['serial']}: cannot start RGB stream: {error}", file=sys.stderr)

    if not pipelines:
        print("No RealSense RGB stream could be started.", file=sys.stderr)
        return 1

    print("Press q or ESC in any image window to quit.")
    try:
        while True:
            for device, pipeline in pipelines:
                try:
                    frames = pipeline.wait_for_frames(timeout_ms=1000)
                except RuntimeError as error:
                    print(f"Frame timeout for {device['serial']}: {error}", file=sys.stderr)
                    continue
                color_frame = frames.get_color_frame()
                if color_frame:
                    image = np.asanyarray(color_frame.get_data())
                    cv2.imshow(
                        f"RealSense USB {device['usb_port_id'] or device['serial']}",
                        draw_label(cv2, image, device),
                    )
            if cv2.waitKey(1) & 0xFF in (27, ord("q")):
                break
            time.sleep(0.001)
    finally:
        for _, pipeline in pipelines:
            pipeline.stop()
        cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
