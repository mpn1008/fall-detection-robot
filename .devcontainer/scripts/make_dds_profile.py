#!/usr/bin/env python3
"""Generate a Fast DDS UDP-only profile on Windows or Linux (stdlib only)."""

import argparse
import ipaddress
from pathlib import Path
import socket
import xml.etree.ElementTree as ET


def ipv4(value):
    value = value.strip()
    try:
        address = socket.gethostbyname(value)
        return str(ipaddress.IPv4Address(address))
    except (OSError, ValueError) as error:
        raise argparse.ArgumentTypeError(f"Cannot resolve IPv4 address {value!r}") from error


def locator(parent, address):
    entry = ET.SubElement(parent, "locator")
    udp = ET.SubElement(entry, "udpv4")
    ET.SubElement(udp, "address").text = address


def make_profile(peers, address):
    profiles = ET.Element("profiles", xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles")
    transports = ET.SubElement(profiles, "transport_descriptors")
    transport = ET.SubElement(transports, "transport_descriptor")
    ET.SubElement(transport, "transport_id").text = "udp_transport"
    ET.SubElement(transport, "type").text = "UDPv4"

    participant = ET.SubElement(
        profiles, "participant", profile_name="ros_udp", is_default_profile="true"
    )
    rtps = ET.SubElement(participant, "rtps")
    builtin = ET.SubElement(rtps, "builtin")
    # Unicast-only discovery. Probe participant IDs 0..119 on each peer.
    ET.SubElement(builtin, "metatrafficMulticastLocatorList")
    discovery = ET.SubElement(builtin, "initialPeersList")
    # Local nodes also need to discover one another without multicast.
    for peer in dict.fromkeys([address, *peers]):
        locator(discovery, peer)
    local_discovery = ET.SubElement(builtin, "metatrafficUnicastLocatorList")
    locator(local_discovery, address)
    local_data = ET.SubElement(rtps, "defaultUnicastLocatorList")
    locator(local_data, address)
    user_transports = ET.SubElement(rtps, "userTransports")
    ET.SubElement(user_transports, "transport_id").text = "udp_transport"
    ET.SubElement(rtps, "useBuiltinTransports").text = "false"
    # This is a transport setting, not an RTPS participant setting.
    ET.SubElement(transport, "maxInitialPeersRange").text = "120"
    return profiles


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--peers", required=True, help="Comma-separated reachable peer IPv4s/hostnames")
    parser.add_argument("--address", required=True, type=ipv4, help="Reachable local IPv4 advertised to peers")
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    try:
        peers = [ipv4(peer) for peer in args.peers.split(",") if peer.strip()]
    except argparse.ArgumentTypeError as error:
        parser.error(str(error))
    if not peers:
        parser.error("At least one peer is required")
    tree = ET.ElementTree(make_profile(peers, args.address))
    ET.indent(tree, space="  ")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    tree.write(args.output, encoding="utf-8", xml_declaration=True)
    print(f"DDS profile: {args.output}; local={args.address}; peers={','.join(peers)}")


if __name__ == "__main__":
    main()
