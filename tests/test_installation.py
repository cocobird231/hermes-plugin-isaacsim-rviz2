"""Run on a provisioned Linux host; isolated homes never touch production Hermes."""
import importlib.util
import contextlib
import io
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
import hermes_sim_config as config

spec = importlib.util.spec_from_file_location("installer_under_test", ROOT / "scripts/install_plugins.py")
installer = importlib.util.module_from_spec(spec)
spec.loader.exec_module(installer)


class ConfigTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix="sim config test ")
        self.base = Path(self.temp.name)
        self.patch = patch.dict(os.environ, {"HERMES_HOME": str(self.base / "home"), "HERMES_SIM_CONFIG": ""})
        self.patch.start()

    def tearDown(self):
        self.patch.stop()
        self.temp.cleanup()

    def test_defaults_do_not_require_isaac_for_standalone_rviz(self):
        data = config.load_config()
        self.assertEqual(data["isaac_python"], "")
        self.assertEqual(data["display_number"], 1)
        self.assertEqual(data["state_root"], str(self.base / "home/sim-plugins-state"))

    def test_required_missing_explains_configuration(self):
        with self.assertRaisesRegex(config.ConfigError, "Missing isaac_python"):
            config.load_config(required=("isaac_python",))

    def test_relative_paths_anchor_to_json_parent(self):
        selected = self.base / "host.json"
        selected.write_text(json.dumps({"gongguan_usd": "assets/road.usdz", "state_root": "state"}))
        with patch.dict(os.environ, {"HERMES_SIM_CONFIG": str(selected)}):
            data = config.load_config()
        self.assertEqual(data["gongguan_usd"], str(self.base / "assets/road.usdz"))
        self.assertEqual(data["export_root"], str(self.base / "state/exports"))

    def test_invalid_types_ranges_and_unknown_fields_rejected(self):
        cases = [{"display_number": True}, {"isaac_port": 80}, {"ros_domain_id": 233},
                 {"ros_localhost_only": "true"}, {"schema_version": True}, {"extra": "ignored?"},
                 {"rv1_usd": "/x.txt"}, {"ros_setup": ["bad"]}, {"rmw_implementation": "bad; cmd"},
                 {"isaac_port": 9000, "rviz2_port": 9000}, {"xauthority": "bad\npath"}]
        for data in cases:
            with self.subTest(data=data), self.assertRaises(config.ConfigError):
                config.normalize_config(data)

    def test_duplicate_json_rejected(self):
        selected = self.base / "host.json"
        selected.write_text("{\"display_number\":1,\"display_number\":2}")
        with patch.dict(os.environ, {"HERMES_SIM_CONFIG": str(selected)}), self.assertRaisesRegex(config.ConfigError, "Duplicate JSON"):
            config.load_config()

    def test_mutable_outputs_cannot_be_inside_public_repo(self):
        for key in ("state_root", "export_root"):
            with self.subTest(key=key), self.assertRaisesRegex(config.ConfigError, "outside"):
                config.normalize_config({key: str(ROOT / "runtime")})

    def test_virtualenv_executable_symlink_is_preserved(self):
        link = self.base / "venv python"
        link.symlink_to(sys.executable)
        result = config.normalize_config({"isaac_python": str(link)}, required=("isaac_python",))
        self.assertEqual(result["isaac_python"], str(link))

    def test_required_executable_checked(self):
        path = self.base / "python"
        path.write_text("fake")
        with self.assertRaisesRegex(config.ConfigError, "not executable"):
            config.normalize_config({"isaac_python": str(path)}, required=("isaac_python",))


class InstallationTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix="hermes install test ")
        self.base = Path(self.temp.name)
        self.home = self.base / "isolated home"
        self.executable = self.base / "fake python"
        self.executable.write_text("#!/bin/sh\nexit 0\n")
        self.executable.chmod(0o700)
        self.scene = self.base / "road model.usdz"
        self.scene.touch()
        self.robot = self.base / "robot model.usd"
        self.robot.touch()
        self.ros_setup = self.base / "ros setup.bash"
        self.ros_setup.touch()
        self.input = self.base / "host.json"
        self.data = {"isaac_python": str(self.executable), "ros_python": str(self.executable),
                     "screenshot_python": str(self.executable), "hermes_cli": str(self.executable),
                     "ros_setup": str(self.ros_setup), "gongguan_usd": str(self.scene), "rv1_usd": str(self.robot)}
        self.input.write_text(json.dumps(self.data))
        self.env = dict(os.environ)
        self.env.pop("HERMES_SIM_CONFIG", None)
        self.env["HERMES_HOME"] = str(self.home)

    def tearDown(self):
        self.temp.cleanup()

    def run_install(self, *extra, root=ROOT):
        return subprocess.run(["bash", str(root / "install.sh"), "--non-interactive", "--config", str(self.input),
                               "--hermes-home", str(self.home), *extra], text=True, capture_output=True, env=self.env)

    def test_new_install_disabled_private_config_and_valid_links(self):
        result = self.run_install()
        self.assertEqual(result.returncode, 0, result.stderr)
        metadata = json.loads(result.stdout)
        self.assertFalse(metadata["automatic_enablement"])
        self.assertEqual(set(metadata["new_plugins"]), set(installer.PLUGIN_NAMES))
        yaml, Loader = installer.yaml_api()
        selection = yaml.load((self.home / "config.yaml").read_text(), Loader=Loader)
        self.assertEqual(selection["plugins"]["disabled"], list(installer.PLUGIN_NAMES))
        self.assertNotIn("enabled", selection["plugins"])
        self.assertEqual((self.home / "sim-plugins.json").stat().st_mode & 0o777, 0o600)
        for name in installer.PLUGIN_NAMES:
            self.assertEqual((self.home / "plugins" / name).resolve(), ROOT / "codes" / name)

    def test_existing_yaml_values_and_unrelated_comments_preserved(self):
        self.home.mkdir()
        original = "# untouched header\nmodel: example\nsecret: ${PRIVATE_TEST_ENV}\nplugins:\n  enabled: [unrelated]\n  entries:\n    unrelated:\n      settings: {value: keep}\n# untouched tail\nother: 17\n"
        (self.home / "config.yaml").write_text(original)
        result = self.run_install()
        self.assertEqual(result.returncode, 0, result.stderr)
        updated = (self.home / "config.yaml").read_text()
        self.assertTrue(updated.startswith("# untouched header\nmodel: example\nsecret: ${PRIVATE_TEST_ENV}\n"))
        self.assertIn("other: 17", updated)
        yaml, Loader = installer.yaml_api()
        value = yaml.load(updated, Loader=Loader)
        self.assertEqual(value["plugins"]["enabled"], ["unrelated"])
        self.assertEqual(value["plugins"]["entries"]["unrelated"]["settings"], {"value": "keep"})
        backup = Path(json.loads(result.stdout)["hermes_config_backup"])
        self.assertEqual(backup.read_text(), original)
        self.assertEqual(backup.stat().st_mode & 0o777, 0o600)

    def test_idempotent_reinstall_preserves_user_enablement(self):
        first = self.run_install()
        self.assertEqual(first.returncode, 0, first.stderr)
        choice = "plugins:\n  enabled: [isaac-sim, unrelated]\n  disabled: [rviz2-navigation]\n"
        (self.home / "config.yaml").write_text(choice)
        second = self.run_install("--display-number", "3", "--state-root", str(self.base / "alternate state"))
        self.assertEqual(second.returncode, 0, second.stderr)
        self.assertEqual((self.home / "config.yaml").read_text(), choice)
        data = json.loads((self.home / "sim-plugins.json").read_text())
        self.assertEqual(data["display_number"], 3)
        self.assertEqual(data["export_root"], str(self.base / "alternate state/exports"))
        self.assertEqual(json.loads(second.stdout)["new_plugins"], [])

    def test_foreign_collision_refused_without_config_writes(self):
        target = self.home / "plugins/isaac-sim"
        target.mkdir(parents=True)
        (target / "precious").write_text("keep")
        result = self.run_install()
        self.assertEqual(result.returncode, 2)
        self.assertIn("Refusing to replace", result.stderr)
        self.assertFalse((self.home / "sim-plugins.json").exists())
        self.assertFalse((self.home / "config.yaml").exists())
        self.assertEqual((target / "precious").read_text(), "keep")

    def test_broken_foreign_symlink_refused(self):
        folder = self.home / "plugins"
        folder.mkdir(parents=True)
        target = folder / "isaac-sim"
        target.symlink_to(self.base / "missing")
        result = self.run_install()
        self.assertEqual(result.returncode, 2)
        self.assertTrue(target.is_symlink())

    def test_malformed_duplicate_or_custom_yaml_fails_without_overwrite(self):
        self.home.mkdir()
        for text in ("model: [\n", "model: first\nmodel: second\n", "model: !unknown value\n", "plugins: []\n"):
            with self.subTest(text=text):
                (self.home / "config.yaml").write_text(text)
                result = self.run_install()
                self.assertEqual(result.returncode, 2, result.stdout)
                self.assertEqual((self.home / "config.yaml").read_text(), text)
                self.assertFalse((self.home / "plugins/isaac-sim").exists())

    def test_absent_asset_refused_before_install(self):
        self.scene.unlink()
        result = self.run_install()
        self.assertEqual(result.returncode, 2)
        self.assertIn("gongguan_usd file does not exist", result.stderr)
        self.assertFalse(self.home.exists())

    def test_missing_input_noninteractive_refused(self):
        self.input.write_text("{}")
        result = self.run_install()
        self.assertEqual(result.returncode, 2)
        self.assertIn("Missing isaac_python", result.stderr)
        self.assertFalse(self.home.exists())
        args = installer.parser().parse_args(["--non-interactive", "--config", str(self.input), "--hermes-home", str(self.home)])
        with patch.dict(os.environ, self.env), patch.object(sys.stdin, "isatty", return_value=True), patch("builtins.input") as prompt, patch.dict(installer.INTERACTIVE_PATH_DEFAULTS, self.data):
            with self.assertRaisesRegex(config.ConfigError, "Missing isaac_python"):
                installer.install(args)
        prompt.assert_not_called()
        self.assertFalse(self.home.exists())

    def test_interactive_enter_saves_displayed_path_defaults(self):
        paths = {key: self.data.pop(key) for key in ("isaac_python", "gongguan_usd", "rv1_usd", "hermes_cli")}
        self.data.update(dict.fromkeys(paths, ""))
        self.input.write_text(json.dumps(self.data))
        args = installer.parser().parse_args(["--config", str(self.input), "--hermes-home", str(self.home)])
        with patch.dict(os.environ, self.env), patch.object(sys.stdin, "isatty", return_value=True), patch("builtins.input", return_value="") as prompt, patch.dict(installer.INTERACTIVE_PATH_DEFAULTS, paths), contextlib.redirect_stdout(io.StringIO()):
            installer.install(args)
        data = json.loads((self.home / "sim-plugins.json").read_text())
        self.assertEqual(prompt.call_count, len(paths))
        for (key, path), call in zip(paths.items(), prompt.call_args_list):
            self.assertIn(f"[{path}]", call.args[0])
            self.assertEqual(data[key], path)

    def test_interactive_input_overrides_displayed_path_defaults(self):
        paths = {key: self.data.pop(key) for key in ("isaac_python", "gongguan_usd", "rv1_usd", "hermes_cli")}
        self.data.update(dict.fromkeys(paths, ""))
        unused_defaults = {key: str(self.base / "unused" / Path(path).name) for key, path in paths.items()}
        self.input.write_text(json.dumps(self.data))
        args = installer.parser().parse_args(["--config", str(self.input), "--hermes-home", str(self.home)])
        with patch.dict(os.environ, self.env), patch.object(sys.stdin, "isatty", return_value=True), patch("builtins.input", side_effect=list(paths.values())) as prompt, patch.dict(installer.INTERACTIVE_PATH_DEFAULTS, unused_defaults), contextlib.redirect_stdout(io.StringIO()):
            installer.install(args)
        data = json.loads((self.home / "sim-plugins.json").read_text())
        self.assertEqual(prompt.call_count, len(paths))
        for key, path in paths.items():
            self.assertEqual(data[key], path)

    def test_interactive_missing_default_asset_fails_before_writes(self):
        paths = {key: self.data.pop(key) for key in ("isaac_python", "gongguan_usd", "rv1_usd", "hermes_cli")}
        self.data.update(dict.fromkeys(paths, ""))
        self.scene.unlink()
        self.input.write_text(json.dumps(self.data))
        original = self.input.read_text()
        args = installer.parser().parse_args(["--config", str(self.input), "--hermes-home", str(self.home)])
        with patch.dict(os.environ, self.env), patch.object(sys.stdin, "isatty", return_value=True), patch("builtins.input", return_value=""), patch.dict(installer.INTERACTIVE_PATH_DEFAULTS, paths):
            with self.assertRaisesRegex(config.ConfigError, "gongguan_usd file does not exist"):
                installer.install(args)
        self.assertFalse(self.home.exists())
        self.assertEqual(self.input.read_text(), original)

    def test_saved_config_and_cli_paths_take_precedence_without_prompting(self):
        alternate = self.base / "alternate python"
        alternate.symlink_to(self.executable)
        self.home.mkdir()
        (self.home / "sim-plugins.json").write_text(json.dumps(self.data))
        unused_defaults = {key: str(self.base / "unused" / key) for key in ("isaac_python", "gongguan_usd", "rv1_usd", "hermes_cli")}
        args = installer.parser().parse_args(["--hermes-home", str(self.home), "--isaac-python", str(alternate)])
        with patch.dict(os.environ, self.env, clear=True), patch.object(sys.stdin, "isatty", return_value=True), patch("builtins.input") as prompt, patch.dict(installer.INTERACTIVE_PATH_DEFAULTS, unused_defaults), contextlib.redirect_stdout(io.StringIO()):
            installer.install(args)
        prompt.assert_not_called()
        data = json.loads((self.home / "sim-plugins.json").read_text())
        self.assertEqual(data["isaac_python"], str(alternate))
        for key in ("gongguan_usd", "rv1_usd", "hermes_cli"):
            self.assertEqual(data[key], self.data[key])

    def test_missing_input_without_tty_does_not_select_defaults(self):
        self.input.write_text("{}")
        args = installer.parser().parse_args(["--config", str(self.input), "--hermes-home", str(self.home)])
        with patch.dict(os.environ, self.env), patch.object(sys.stdin, "isatty", return_value=False), patch("builtins.input") as prompt, patch.dict(installer.INTERACTIVE_PATH_DEFAULTS, self.data):
            with self.assertRaisesRegex(config.ConfigError, "Missing isaac_python"):
                installer.install(args)
        prompt.assert_not_called()
        self.assertFalse(self.home.exists())

    def test_clone_relocation_with_spaces_works(self):
        relocated = self.base / "relocated clone with spaces"
        relocated.mkdir()
        (relocated / "scripts").mkdir()
        for name in ("install.sh", "hermes_sim_config.py", "scripts/install_plugins.py"):
            shutil.copy2(ROOT / name, relocated / name)
        for name in installer.PLUGIN_NAMES:
            plugin = relocated / "codes" / name
            plugin.mkdir(parents=True)
            (plugin / "plugin.yaml").write_text("name: " + name + "\n")
            (plugin / "__init__.py").write_text("def register(ctx): pass\n")
        result = self.run_install(root=relocated)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual((self.home / "plugins/isaac-sim").resolve(), relocated / "codes/isaac-sim")

    def test_symlinked_config_refused(self):
        self.home.mkdir()
        external = self.base / "external.yaml"
        external.write_text("model: keep\n")
        (self.home / "config.yaml").symlink_to(external)
        result = self.run_install()
        self.assertEqual(result.returncode, 2)
        self.assertEqual(external.read_text(), "model: keep\n")

    def test_cli_preserves_executable_symlink(self):
        link = self.base / "venv" / "bin" / "python"
        link.parent.mkdir(parents=True)
        link.symlink_to(self.executable)
        result = self.run_install("--isaac-python", str(link))
        self.assertEqual(result.returncode, 0, result.stderr)
        data = json.loads((self.home / "sim-plugins.json").read_text())
        self.assertEqual(data["isaac_python"], str(link))

    def test_interactive_prompt_preserves_executable_symlink(self):
        link = self.base / "venv" / "bin" / "python"
        link.parent.mkdir(parents=True)
        link.symlink_to(self.executable)
        self.data.pop("isaac_python")
        self.input.write_text(json.dumps(self.data))
        args = installer.parser().parse_args(["--config", str(self.input), "--hermes-home", str(self.home)])
        with patch.dict(os.environ, self.env), patch.object(sys.stdin, "isatty", return_value=True), patch("builtins.input", return_value=str(link)), contextlib.redirect_stdout(io.StringIO()):
            installer.install(args)
        data = json.loads((self.home / "sim-plugins.json").read_text())
        self.assertEqual(data["isaac_python"], str(link))

    def test_flow_style_yaml_preserves_values(self):
        updated = installer.disabled_configuration("{model: keep, plugins: {enabled: [other]}}\n", installer.PLUGIN_NAMES)
        yaml, Loader = installer.yaml_api()
        value = yaml.load(updated, Loader=Loader)
        self.assertEqual(value["model"], "keep")
        self.assertEqual(value["plugins"]["enabled"], ["other"])


if __name__ == "__main__":
    unittest.main(verbosity=2)
