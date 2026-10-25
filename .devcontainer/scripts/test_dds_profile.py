"""Run with python3 -B .devcontainer/scripts/test_dds_profile.py."""

import tempfile
import unittest
from pathlib import Path
import subprocess
import sys
import xml.etree.ElementTree as ET

from make_dds_profile import ipv4, make_profile


class ProfileTests(unittest.TestCase):
    def test_udp_only_and_local_discovery(self):
        root = make_profile(["192.168.1.10", "192.168.1.10"], "192.168.1.20")
        self.assertEqual(root.find(".//useBuiltinTransports").text, "false")
        self.assertEqual(root.find(".//transport_descriptor/type").text, "UDPv4")
        self.assertEqual(root.find(".//maxInitialPeersRange").text, "120")
        peers = root.findall(".//initialPeersList/locator/udpv4/address")
        self.assertEqual([peer.text for peer in peers], ["192.168.1.20", "192.168.1.10"])
        self.assertEqual(root.find(".//defaultUnicastLocatorList/locator/udpv4/address").text, "192.168.1.20")

    def test_resolves_localhost(self):
        self.assertEqual(ipv4("localhost"), "127.0.0.1")

    def test_cli_writes_namespaced_xml(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "nested" / "fastdds.xml"
            subprocess.run(
                [sys.executable, "-B", str(Path(__file__).with_name("make_dds_profile.py")),
                 "--peers", "127.0.0.1", "--address", "127.0.0.1", "--output", str(output)],
                check=True, capture_output=True, text=True,
            )
            root = ET.parse(output).getroot()
            self.assertEqual(root.tag, "{http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles}profiles")

    def test_empty_peers_rejected(self):
        result = subprocess.run(
            [sys.executable, "-B", str(Path(__file__).with_name("make_dds_profile.py")),
             "--peers", ",", "--address", "127.0.0.1", "--output", "unused.xml"],
            capture_output=True, text=True,
        )
        self.assertEqual(result.returncode, 2)


if __name__ == "__main__":
    unittest.main()
