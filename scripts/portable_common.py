"""Shared CLI configuration and private-output policy for optional helpers."""
import importlib
import os
from pathlib import Path
import sys

PUBLIC_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PUBLIC_ROOT))


def configure(path=None, required=()):
    if path:
        os.environ["HERMES_SIM_CONFIG"] = str(Path(path).expanduser().resolve())
    from hermes_sim_config import load_config
    return load_config(required=required)


def private_output(value):
    path = Path(value).expanduser().resolve()
    if path == PUBLIC_ROOT or path.is_relative_to(PUBLIC_ROOT):
        raise ValueError("Generated evidence/assets must be outside the public plugin checkout")
    return path


def load_hermes_source(config):
    source = config.get("hermes_source")
    if source:
        sys.path.insert(0, str(Path(source).expanduser().resolve()))
    try:
        importlib.import_module("hermes_bootstrap")
    except ImportError as exc:
        raise ValueError("Run with the Hermes Python environment or configure hermes_source") from exc
