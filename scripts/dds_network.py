#!/usr/bin/env python3
"""Validate autonomy_light DDS networking and write a Cyclone DDS URI file."""

import argparse
import ipaddress
import shlex
from pathlib import Path
from typing import Dict, Optional
from xml.sax.saxutils import escape

import yaml


def parameters(path: Path) -> dict:
    with path.open(encoding="utf-8") as stream:
        document = yaml.safe_load(stream) or {}
    return document.get("autonomy_light", {}).get("ros__parameters", {})


def boolean(value: object, name: str) -> bool:
    if isinstance(value, bool):
        return value
    if str(value).strip().lower() in ("true", "1", "yes", "on"):
        return True
    if str(value).strip().lower() in ("false", "0", "no", "off"):
        return False
    raise ValueError(f"{name} must be a boolean")


def network_config(params: dict) -> Optional[Dict[str, str]]:
    output = params.get("height_map_output") or {}
    transport = str(output.get("transport", "both")).strip().lower()
    if transport not in ("ros2", "cyclone_dds", "both"):
        raise ValueError("height_map_output.transport must be ros2, cyclone_dds, or both")
    if transport == "ros2":
        return None

    network = params.get("dds_network") or {}
    livox = params.get("livox_network") or {}
    if not isinstance(network, dict) or not isinstance(livox, dict):
        raise ValueError("dds_network and livox_network must be mappings")
    mode = str(network.get("mode", "wireless")).strip().lower()
    interface = str(network.get("interface", "")).strip()
    if mode not in ("wired", "wireless") or not interface:
        raise ValueError("dds_network requires mode wired|wireless and interface")
    local = ipaddress.ip_interface(str(network.get("local_ip", "")).strip())
    peer = ipaddress.ip_address(str(network.get("peer_ip", "")).strip())
    livox_subnet = ipaddress.ip_network(str(livox.get("subnet", "192.168.1.0/24")).strip())
    livox_interface = str(livox.get("interface", "")).strip()
    if local.version != 4 or peer.version != 4 or peer not in local.network:
        raise ValueError("dds_network.local_ip and peer_ip must be IPv4 addresses on one subnet")
    if local.network.overlaps(livox_subnet):
        raise ValueError("DDS and Livox network subnets must not overlap")
    if livox_interface and livox_interface == interface:
        raise ValueError("DDS and Livox must use different network interfaces")
    return {
        "DDS_OUTPUT_ENABLED": "true",
        "DDS_MODE": mode,
        "DDS_INTERFACE": interface,
        "DDS_LOCAL_CIDR": str(local),
        "DDS_LOCAL_IP": str(local.ip),
        "DDS_PEER_IP": str(peer),
        "DDS_ALLOW_MULTICAST": str(boolean(network.get("allow_multicast", True), "dds_network.allow_multicast")).lower(),
    }


def cyclonedds_xml(network: dict) -> str:
    multicast = network["DDS_ALLOW_MULTICAST"]
    receive = "preferred" if multicast == "true" else "none"
    return f'''<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS xmlns="https://cdds.io/config">
  <Domain Id="any">
    <General>
      <Interfaces>
        <NetworkInterface address="{escape(network["DDS_LOCAL_IP"])}" priority="default" multicast="{multicast}" />
      </Interfaces>
      <AllowMulticast>{multicast}</AllowMulticast>
      <MulticastRecvNetworkInterfaceAddresses>{receive}</MulticastRecvNetworkInterfaceAddresses>
    </General>
    <Discovery>
      <ParticipantIndex>auto</ParticipantIndex>
      <Peers><Peer address="{escape(network["DDS_PEER_IP"])}" /></Peers>
    </Discovery>
  </Domain>
</CycloneDDS>
'''


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", required=True, type=Path)
    parser.add_argument("--cyclonedds-config", required=True, type=Path)
    parser.add_argument("--runtime-env", required=True, type=Path)
    args = parser.parse_args()
    try:
        network = network_config(parameters(args.config))
    except ValueError as error:
        raise SystemExit(f"error: {error}") from error
    values = network or {"DDS_OUTPUT_ENABLED": "false"}
    if network:
        args.cyclonedds_config.write_text(cyclonedds_xml(network), encoding="utf-8")
    args.runtime_env.write_text(
        "".join(f"{name}={shlex.quote(value)}\n" for name, value in values.items()), encoding="utf-8")


if __name__ == "__main__":
    main()
