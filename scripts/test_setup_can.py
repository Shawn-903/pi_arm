#!/usr/bin/env python3
"""Check setup_can.sh with fake ip/modprobe commands; never open a CAN interface.

Run with sudo so the production script's root check remains in force.
"""
import json
import os
from pathlib import Path
import subprocess
import tempfile
import unittest


SCRIPT = Path(__file__).with_name("setup_can.sh")
FAKE_COMMAND = """#!/usr/bin/env python3
import json
import os
from pathlib import Path
import sys

name = Path(sys.argv[0]).name
args = sys.argv[1:]
with open(os.environ['COMMAND_LOG'], 'a') as output:
    output.write(json.dumps([name, *args]) + '\\n')
state = Path(os.environ['INTERFACE_STATE'])
kind = state.read_text()
if name == 'ip' and args[:2] == ['link', 'add']:
    state.write_text(args[args.index('type') + 1])
if name == 'ip' and 'show' in args:
    if not kind:
        sys.exit(1)
    if 'type' not in args or args[args.index('type') + 1] == kind:
        print('7: test_interface: <UP>')
if name == 'ip' and args[-1:] == ['down'] and os.environ.get('FAIL_DOWN'):
    sys.exit(1)
"""


class SetupCanTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix="pi-arm-setup-can-")
        self.addCleanup(self.temp.cleanup)
        self.directory = Path(self.temp.name)
        self.log = self.directory / "commands.jsonl"
        for name in ("ip", "modprobe"):
            command = self.directory / name
            command.write_text(FAKE_COMMAND, encoding="utf-8")
            command.chmod(0o755)

    def run_script(self, *args, kind="", fail_down=False):
        state = self.directory / "interface-kind"
        state.write_text(kind, encoding="utf-8")
        env = dict(os.environ, PATH=f"{self.directory}:{os.defpath}",
                   COMMAND_LOG=str(self.log), INTERFACE_STATE=str(state))
        env.pop("FAIL_DOWN", None)
        if fail_down:
            env["FAIL_DOWN"] = "1"
        self.log.write_text("", encoding="utf-8")
        result = subprocess.run(["bash", str(SCRIPT), *args], env=env,
                                capture_output=True, text=True, timeout=5)
        calls = [json.loads(line) for line in self.log.read_text().splitlines()]
        return result, calls

    def test_invalid_bitrate_has_no_side_effects(self):
        for bitrate in ("invalid", "0", "-1"):
            with self.subTest(bitrate=bitrate):
                result, calls = self.run_script("real", "can0", bitrate, kind="can")
                self.assertEqual(result.returncode, 2)
                self.assertEqual(calls, [])

    def test_unknown_mode_has_no_side_effects(self):
        result, calls = self.run_script("unknown")
        self.assertEqual(result.returncode, 2)
        self.assertEqual(calls, [])

    def test_real_mode_rejects_missing_or_wrong_interface_type(self):
        for kind in ("", "vcan", "ether"):
            with self.subTest(kind=kind):
                result, calls = self.run_script("real", "can0", kind=kind)
                self.assertEqual(result.returncode, 2)
                self.assertTrue(all("show" in call for call in calls))

    def test_real_mode_preserves_configuration_order(self):
        result, calls = self.run_script("real", "can0", "500000", kind="can")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual([call for call in calls if "set" in call], [
            ["ip", "link", "set", "can0", "down"],
            ["ip", "link", "set", "can0", "type", "can", "bitrate", "500000", "restart-ms", "100"],
            ["ip", "link", "set", "can0", "txqueuelen", "1000"],
            ["ip", "link", "set", "can0", "up"],
        ])

    def test_failed_down_does_not_continue_configuration(self):
        result, calls = self.run_script("real", "can0", kind="can", fail_down=True)
        self.assertNotEqual(result.returncode, 0)
        self.assertEqual(calls[-1], ["ip", "link", "set", "can0", "down"])

    def test_vcan_rejects_existing_physical_interface(self):
        result, calls = self.run_script("vcan", "can0", kind="can")
        self.assertEqual(result.returncode, 2)
        self.assertTrue(all(call[0] == "ip" and "show" in call for call in calls))

    def test_vcan_defaults_to_virtual_interface_name(self):
        result, calls = self.run_script("vcan", kind="vcan")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn(["ip", "link", "set", "vcan0", "up"], calls)
        self.assertFalse(any(call[0] == "modprobe" or "add" in call for call in calls))

    def test_vcan_creation(self):
        result, calls = self.run_script("vcan", "vcan_test")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn(["modprobe", "vcan"], calls)
        self.assertIn(["ip", "link", "add", "dev", "vcan_test", "type", "vcan"], calls)
        self.assertIn(["ip", "link", "set", "vcan_test", "up"], calls)

    def test_down_only_changes_the_requested_interface(self):
        result, calls = self.run_script("down", "vcan_test", kind="vcan")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual([call for call in calls if "set" in call],
                         [["ip", "link", "set", "vcan_test", "down"]])


if __name__ == "__main__":
    if os.geteuid() != 0:
        raise SystemExit("Run with sudo; ip and modprobe are replaced by temporary fakes.")
    unittest.main()
