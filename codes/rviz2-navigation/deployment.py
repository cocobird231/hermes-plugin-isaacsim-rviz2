"""Load the package deployment profile without depending on the checkout name."""
import importlib.util
import hashlib
from pathlib import Path
import sys

_ROOT = Path(__file__).resolve().parents[2]
_NAME = "_hermes_sim_deployment_config_" + hashlib.sha256(str(_ROOT).encode()).hexdigest()[:16]
if _NAME not in sys.modules:
    _spec = importlib.util.spec_from_file_location(_NAME, _ROOT / "hermes_sim_config.py")
    _module = importlib.util.module_from_spec(_spec)
    sys.modules[_NAME] = _module
    _spec.loader.exec_module(_module)
_config = sys.modules[_NAME]
load_config = _config.load_config
config_path = _config.config_path
PACKAGE_ROOT = _ROOT


def asset_path(name):
    config = load_config(required=(name,))
    value = config.get(name)
    if not isinstance(value, str) or not value.strip():
        raise ValueError(f"Deployment configuration requires {name}; ask the user for its host path and save it with bash install.sh")
    path = Path(value).expanduser().resolve()
    if not path.is_file():
        raise ValueError(f"Configured {name} does not exist: {path}")
    return str(path)
