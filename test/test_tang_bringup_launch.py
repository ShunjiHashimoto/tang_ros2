#!/usr/bin/env python3
"""統合bringupがTangControllerの安全な初期モードを指定することを確認する。"""

import ast
from pathlib import Path
import unittest


LAUNCH_FILE = (
    Path(__file__).resolve().parents[1]
    / "tang_bringup"
    / "launch"
    / "tang_bringup.launch.py"
)


def call_name(node):
    """AST Callの関数名末尾を返す。"""
    function = node.func
    if isinstance(function, ast.Name):
        return function.id
    if isinstance(function, ast.Attribute):
        return function.attr
    return ""


class TangBringupLaunchTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tree = ast.parse(LAUNCH_FILE.read_text(encoding="utf-8"))

    def test_initial_mode_launch_argument_defaults_to_manual(self):
        declarations = [
            node
            for node in ast.walk(self.tree)
            if isinstance(node, ast.Call)
            and call_name(node) == "DeclareLaunchArgument"
            and node.args
            and isinstance(node.args[0], ast.Constant)
            and node.args[0].value == "initial_mode"
        ]
        self.assertEqual(1, len(declarations))
        keywords = {keyword.arg: keyword.value for keyword in declarations[0].keywords}
        self.assertEqual("manual", ast.literal_eval(keywords["default_value"]))
        self.assertEqual(["idle", "manual"], ast.literal_eval(keywords["choices"]))

    def test_tang_control_receives_initial_mode_as_string_parameter(self):
        parameter_values = [
            node
            for node in ast.walk(self.tree)
            if isinstance(node, ast.Call)
            and call_name(node) == "ParameterValue"
            and node.args
            and isinstance(node.args[0], ast.Name)
            and node.args[0].id == "initial_mode"
        ]
        self.assertEqual(1, len(parameter_values))
        keywords = {keyword.arg: keyword.value for keyword in parameter_values[0].keywords}
        self.assertIsInstance(keywords["value_type"], ast.Name)
        self.assertEqual("str", keywords["value_type"].id)


if __name__ == "__main__":
    unittest.main()
