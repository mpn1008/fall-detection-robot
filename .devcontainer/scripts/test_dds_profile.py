"""Run with python3 -B .devcontainer/scripts/test_dds_profile.py."""

import importlib.util
import os
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
        self.assertIsNone(root.find(".//metatrafficMulticastLocatorList"))
        self.assertEqual(
            root.find(".//metatrafficUnicastLocatorList/locator/udpv4/address").text,
            "192.168.1.20",
        )
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


@unittest.skipUnless(importlib.util.find_spec("rclpy"), "Native parser test requires ROS rclpy")
class NativeParserTests(unittest.TestCase):
    def load_with_fastdds(self, root):
        # Create a real DDS participant in a separate process. Merely parsing the
        # XML with ElementTree cannot detect Fast DDS schema/parser rejection.
        with tempfile.TemporaryDirectory() as directory:
            profile = Path(directory) / "native-parser.xml"
            ET.ElementTree(root).write(profile, encoding="utf-8", xml_declaration=True)
            env = {
                **os.environ,
                "RMW_IMPLEMENTATION": "rmw_fastrtps_cpp",
                "ROS_DOMAIN_ID": "42",
                "FASTRTPS_DEFAULT_PROFILES_FILE": str(profile),
                "FASTDDS_DEFAULT_PROFILES_FILE": str(profile),
            }
            return subprocess.run(
                [
                    sys.executable, "-c",
                    "import rclpy; rclpy.init(); "
                    "node = rclpy.create_node('dds_profile_parser_test'); "
                    "print('DDS participant created'); "
                    "node.destroy_node(); rclpy.shutdown()",
                ],
                env=env, capture_output=True, text=True, timeout=15,
            )

    def test_fastdds_accepts_generated_profile(self):
        result = self.load_with_fastdds(make_profile(["127.0.0.1"], "127.0.0.1"))
        output = result.stdout + result.stderr
        self.assertEqual(result.returncode, 0, output)
        self.assertIn("DDS participant created", output)
        # Rejected profiles can still create a participant using defaults!
        self.assertNotIn("XMLPARSER", output, output)

    def test_native_check_detects_the_previous_empty_locator_bug(self):
        root = make_profile(["127.0.0.1"], "127.0.0.1")
        ET.SubElement(root.find(".//builtin"), "metatrafficMulticastLocatorList")
        result = self.load_with_fastdds(root)
        self.assertIn("XMLPARSER", result.stdout + result.stderr)


if __name__ == "__main__":
    unittest.main()
