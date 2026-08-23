#!/usr/bin/env python3
"""TANG/DNE自動起動モード選択を、実プロセスを起動せず確認する。"""

import os
from pathlib import Path
import subprocess
import tempfile
import unittest


REPOSITORY = Path(__file__).resolve().parents[1]
AUTO_START = REPOSITORY / "shell_scripts" / "auto_start.sh"
SELECT_MODE = REPOSITORY / "shell_scripts" / "select_startup_mode.sh"


class AutoStartScriptsTest(unittest.TestCase):
    def setUp(self):
        self.temporary_directory = tempfile.TemporaryDirectory()
        self.mode_file = Path(self.temporary_directory.name) / "startup_mode"
        self.environment = os.environ.copy()
        self.environment["TANG_STARTUP_MODE_FILE"] = str(self.mode_file)
        self.environment["TANG_STARTUP_VALIDATE_ONLY"] = "1"

    def tearDown(self):
        self.temporary_directory.cleanup()

    def run_script(self, script, *arguments, check=True):
        return subprocess.run(
            [str(script), *arguments],
            check=check,
            capture_output=True,
            text=True,
            env=self.environment,
        )

    def test_tang_mode_selects_docker_bringup(self):
        self.run_script(SELECT_MODE, "tang")

        result = self.run_script(AUTO_START)

        self.assertEqual("TANG", self.mode_file.read_text().strip())
        self.assertIn("Selected startup mode: TANG", result.stdout)
        self.assertIn(
            "ros2 launch tang_bringup tang_bringup.launch.py",
            result.stdout,
        )

    def test_dne_mode_selects_host_rs485_handler(self):
        self.run_script(SELECT_MODE, "dne")

        result = self.run_script(AUTO_START)

        self.assertEqual("DNE", self.mode_file.read_text().strip())
        self.assertIn("Selected startup mode: DNE", result.stdout)
        self.assertIn("rs485Handler.py", result.stdout)
        self.assertIn("--robot CuGoV4", result.stdout)
        self.assertNotIn("docker/run.sh", result.stdout)

    def test_missing_mode_file_defaults_to_tang(self):
        result = self.run_script(AUTO_START)

        self.assertIn("Selected startup mode: TANG", result.stdout)

    def test_invalid_mode_is_rejected(self):
        self.mode_file.write_text("INVALID\n", encoding="utf-8")

        result = self.run_script(AUTO_START, check=False)

        self.assertEqual(2, result.returncode)
        self.assertIn("Startup mode must be TANG or DNE", result.stderr)


if __name__ == "__main__":
    unittest.main()
