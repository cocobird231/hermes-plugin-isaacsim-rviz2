#!/usr/bin/env python3
"""Offline installation of a cloned Hermes Isaac Sim + RViz2 plugin bundle."""
from __future__ import annotations

import argparse
from copy import deepcopy
import fcntl
import hashlib
import json
import os
from pathlib import Path
import sys
import tempfile
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from hermes_sim_config import ConfigError, PATH_FIELDS, _unique_object, defaults, normalize_config

PLUGIN_NAMES = ("isaac-sim", "rviz2-navigation")
REQUIRED = ("isaac_python", "gongguan_usd", "rv1_usd", "ros_python", "ros_setup", "screenshot_python", "hermes_cli")
# Reference layout offered only for interactive confirmation, never runtime defaults.
INTERACTIVE_PATH_DEFAULTS = {
    "isaac_python": "/home/user/Workspace/env_isaacsim/bin/python",
    "gongguan_usd": "/home/user/Workspace/scenes/10_gongguan_xinhai_rd_mesh-model.usdz",
    "rv1_usd": "/home/user/Workspace/robots/rv1_odom_tuned.usd",
    "hermes_cli": "/home/user/.local/bin/hermes",
}


def atomic_write(path, text, mode=0o600):
    path.parent.mkdir(parents=True, exist_ok=True, mode=0o700)
    fd, temp = tempfile.mkstemp(prefix="." + path.name + ".", dir=path.parent)
    try:
        os.fchmod(fd, mode)
        with os.fdopen(fd, "w", encoding="utf-8") as stream:
            stream.write(text)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temp, path)
    finally:
        if os.path.exists(temp):
            os.unlink(temp)


def yaml_api():
    try:
        import yaml
    except ImportError as exc:
        raise ConfigError("python3-yaml is required to preserve Hermes configuration safely; provision the same ROS system prerequisites first") from exc

    class UniqueLoader(yaml.SafeLoader):
        pass

    def mapping(loader, node, deep=False):
        result = {}
        for key_node, value_node in node.value:
            key = loader.construct_object(key_node, deep=deep)
            if not isinstance(key, (str, int, float, bool, type(None))):
                raise ConfigError("Unsupported composite YAML key in Hermes configuration")
            if key in result:
                raise ConfigError("Duplicate YAML key in Hermes configuration")
            result[key] = loader.construct_object(value_node, deep=deep)
        return result

    UniqueLoader.add_constructor(yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG, mapping)
    return yaml, UniqueLoader


def disabled_configuration(text, names):
    """Add only new plugin names to disabled; preserve unrelated values and text.

    A backup retains all comments. Only the plugins block is reserialized; the
    rest is kept byte-for-byte. Duplicate keys and custom tags fail closed.
    """
    yaml, Loader = yaml_api()
    try:
        original = yaml.load(text, Loader=Loader) if text.strip() else {}
    except (yaml.YAMLError, TypeError) as exc:
        raise ConfigError("Hermes config.yaml must be valid unambiguous safe YAML; no changes made") from exc
    if original is None:
        original = {}
    if not isinstance(original, dict):
        raise ConfigError("Hermes config.yaml must contain a mapping")
    desired = deepcopy(original)
    plugins = desired.setdefault("plugins", {})
    if not isinstance(plugins, dict):
        raise ConfigError("Hermes plugins configuration must be a mapping")
    for field in ("enabled", "disabled"):
        if field in plugins and (not isinstance(plugins[field], list) or not all(isinstance(v, str) for v in plugins[field])):
            raise ConfigError(f"Hermes plugins.{field} must be a string list")
    disabled = plugins.setdefault("disabled", [])
    for name in names:
        if name not in disabled:
            disabled.append(name)
    if desired == original:
        return text
    block = yaml.safe_dump({"plugins": plugins}, sort_keys=False, allow_unicode=True)
    node = yaml.compose(text, Loader=Loader) if text.strip() else None
    pair = next(((key, value) for key, value in node.value if key.value == "plugins"), None) if isinstance(node, yaml.MappingNode) else None
    if pair and not node.flow_style:
        key, value = pair
        lines = text.splitlines(keepends=True)
        end = value.end_mark.line + (1 if value.end_mark.column else 0)
        updated = "".join(lines[:key.start_mark.line]) + block + "".join(lines[end:])
    elif pair:
        updated = yaml.safe_dump(desired, sort_keys=False, allow_unicode=True)
    else:
        updated = text + ("\n" if text and not text.endswith("\n") else "") + block
    if yaml.load(updated, Loader=Loader) != desired:
        raise ConfigError("Cannot safely update Hermes plugin selection; no changes made")
    return updated


def parser():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--config", type=Path, help="Existing host JSON to read; normalized copy is saved under selected Hermes home")
    p.add_argument("--hermes-home", type=Path, default=Path(os.environ.get("HERMES_HOME") or Path.home() / ".hermes"))
    p.add_argument("--non-interactive", action="store_true", help="Require all mandatory paths through config/flags; never prompt")
    for name in sorted(PATH_FIELDS):
        p.add_argument("--" + name.replace("_", "-"), dest=name)
    for name in ("display_number", "ros_domain_id", "isaac_port", "rviz2_port"):
        p.add_argument("--" + name.replace("_", "-"), dest=name, type=int)
    p.add_argument("--rmw-implementation")
    p.add_argument("--ros-localhost-only", choices=("true", "false"))
    return p


def install(args):
    home = args.hermes_home.expanduser().resolve()
    if home == ROOT or ROOT in home.parents:
        raise ConfigError("Hermes home must be outside the public plugin repository")
    os.environ["HERMES_HOME"] = str(home)
    destination = home / "sim-plugins.json"
    input_path = args.config or (Path(os.environ["HERMES_SIM_CONFIG"]) if os.environ.get("HERMES_SIM_CONFIG") else destination)
    input_path = input_path.expanduser().resolve()
    data = {}
    if input_path.exists():
        try:
            data = json.loads(input_path.read_text(encoding="utf-8"), object_pairs_hook=_unique_object)
        except (OSError, ValueError) as exc:
            raise ConfigError(f"Cannot read host JSON: {input_path}") from exc
        if not isinstance(data, dict):
            raise ConfigError("Host JSON must contain an object")
    elif args.config:
        raise ConfigError(f"Input config does not exist: {input_path}")
    # JSON-relative input paths are anchored before CLI overrides (which use cwd).
    data = normalize_config(data, base_dir=input_path.parent)
    for name in defaults():
        value = getattr(args, name, None)
        if value is not None:
            if name in PATH_FIELDS and value:
                value = os.path.abspath(Path(value).expanduser())
            if name == "ros_localhost_only":
                value = value == "true"
            data[name] = value
    if args.state_root is not None and args.export_root is None:
        data["export_root"] = str(Path(data["state_root"]) / "exports")
    prompts = {"isaac_python": "Isaac Sim 6 Python executable", "gongguan_usd": "Gongguan scene USD/USDZ", "rv1_usd": "RV1 model USD", "hermes_cli": "Hermes CLI executable"}
    for key, label in prompts.items():
        if not data.get(key):
            if args.non_interactive or not sys.stdin.isatty():
                raise ConfigError(f"Missing {key}; pass --{key.replace(chr(95), chr(45))} or provide it in --config")
            suggested = INTERACTIVE_PATH_DEFAULTS[key]
            value = input(f"{label} (absolute path) [{suggested}]: ").strip() or suggested
            data[key] = os.path.abspath(Path(value).expanduser())
    config = normalize_config(data, required=REQUIRED)
    for name in PLUGIN_NAMES:
        source = ROOT / "codes" / name
        if not (source / "plugin.yaml").is_file() or not (source / "__init__.py").is_file():
            raise ConfigError(f"Clone is incomplete: {source}")
    home.mkdir(parents=True, exist_ok=True, mode=0o700)
    lock_path = home / ".sim-plugins-install.lock"
    with lock_path.open("a+") as lock:
        os.chmod(lock_path, 0o600)
        try:
            fcntl.flock(lock.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as exc:
            raise ConfigError("Another plugin installer is running for this Hermes home") from exc
        folder = home / "plugins"
        if folder.exists() and not folder.is_dir():
            raise ConfigError("Hermes plugins path is not a directory")
        new_names = []
        for name in PLUGIN_NAMES:
            target, source = folder / name, ROOT / "codes" / name
            if target.is_symlink() and target.resolve() == source.resolve():
                continue
            if target.exists() or target.is_symlink():
                raise ConfigError(f"Refusing to replace existing plugin: {target}; choose a separate --hermes-home or remove it explicitly after review")
            new_names.append(name)
        hermes_config = home / "config.yaml"
        if hermes_config.is_symlink():
            raise ConfigError("Refusing to rewrite symlinked Hermes config.yaml")
        original = hermes_config.read_text(encoding="utf-8") if hermes_config.exists() else ""
        updated = disabled_configuration(original, new_names) if new_names else original
        config_text = json.dumps(config, ensure_ascii=False, indent=2) + "\n"
        if destination.is_symlink():
            raise ConfigError("Refusing to replace symlinked sim-plugins.json")
        backup = None
        if updated != original and hermes_config.exists():
            digest = hashlib.sha256(original.encode()).hexdigest()[:12]
            backup = home / "plugin-install-backups" / f"config-{time.time_ns()}-{digest}.yaml"
            atomic_write(backup, original)
        # Disabled first: even an interrupted install cannot accidentally activate plugins.
        if updated != original:
            atomic_write(hermes_config, updated)
        atomic_write(destination, config_text)
        folder.mkdir(parents=True, exist_ok=True, mode=0o700)
        created = []
        try:
            for name in new_names:
                (folder / name).symlink_to(ROOT / "codes" / name, target_is_directory=True)
                created.append(name)
        except OSError:
            for name in created:
                (folder / name).unlink()
            raise
        result = {"success": True, "hermes_home": str(home), "config": str(destination),
                  "source": str(ROOT), "plugins": list(PLUGIN_NAMES), "new_plugins": new_names,
                  "automatic_enablement": False, "hermes_config_backup": str(backup) if backup else None,
                  "next_step": "Run hermes plugins, select isaac-sim and rviz2-navigation, then save. Keep this clone in place; symlinks depend on it."}
        print(json.dumps(result, ensure_ascii=False, indent=2))
        return result


def main():
    args = parser().parse_args()
    try:
        install(args)
        return 0
    except (ConfigError, OSError, EOFError) as exc:
        print(f"Installation failed: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
