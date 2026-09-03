#!/usr/bin/env python3
"""Find one observed Livox Ethernet address without changing host networking.

Livox MID360 devices advertise with the Livox OUI (E4:7A:2C).  The script
consults the kernel neighbour table and can optionally issue one ICMP probe to
the configured hint to refresh its ARP entry.  It intentionally does not scan
or reconfigure an entire subnet.
"""

from __future__ import annotations

import argparse
import ipaddress
import json
import subprocess
import sys
from typing import Any


LIVOX_OUIS = ("e4:7a:2c",)
USABLE_STATES = {"REACHABLE", "STALE", "DELAY", "PROBE", "PERMANENT"}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--interface", required=True, help="LiDAR Ethernet interface")
    parser.add_argument("--hint", help="configured LiDAR IPv4 address")
    parser.add_argument(
        "--probe-hint",
        action="store_true",
        help="send one short ICMP probe to the hint before reading neighbours",
    )
    return parser.parse_args()


def probe_hint(interface: str, hint: str | None) -> None:
    if hint is None:
        return
    try:
        ipaddress.IPv4Address(hint)
    except ipaddress.AddressValueError as error:
        raise ValueError(f"invalid IPv4 hint: {hint}") from error
    subprocess.run(
        ["ping", "-n", "-c", "1", "-W", "1", "-I", interface, hint],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        check=False,
        timeout=2,
    )


def livox_neighbours(interface: str) -> list[str]:
    result = subprocess.run(
        ["ip", "-j", "neigh", "show", "dev", interface],
        capture_output=True,
        text=True,
        check=True,
    )
    records: list[dict[str, Any]] = json.loads(result.stdout)
    candidates: list[str] = []
    for record in records:
        address = record.get("dst")
        mac = str(record.get("lladdr", "")).lower()
        # iproute2 JSON calls this `state`; older releases used `nud`.
        raw_states = record.get("state", record.get("nud", []))
        if isinstance(raw_states, str):
            raw_states = [raw_states]
        states = {str(state).upper() for state in raw_states}
        if not isinstance(address, str) or not mac.startswith(LIVOX_OUIS):
            continue
        try:
            ipaddress.IPv4Address(address)
        except ipaddress.AddressValueError:
            continue
        if states & USABLE_STATES:
            candidates.append(address)
    return sorted(set(candidates), key=ipaddress.IPv4Address)


def main() -> int:
    args = parse_args()
    if args.probe_hint:
        probe_hint(args.interface, args.hint)
    candidates = livox_neighbours(args.interface)
    if args.hint in candidates:
        print(args.hint)
        return 0
    if len(candidates) == 1:
        print(candidates[0])
        return 0
    if not candidates:
        return 1
    print(
        "multiple Livox neighbours found; select one with --lidar-ip: "
        + ", ".join(candidates),
        file=sys.stderr,
    )
    return 2


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (OSError, ValueError, json.JSONDecodeError) as error:
        print(f"find_livox_ip: {error}", file=sys.stderr)
        raise SystemExit(2)
