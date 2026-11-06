"""Generate an isolated RV1 test stage; never modify source assets."""
from pathlib import Path
import uuid
from deployment import asset_path


def prepare_test_scene(run_dir):
    from pxr import Sdf
    model = asset_path("rv1_usd")
    source = Path(__file__).with_name("rv1_test.usda")
    layer = Sdf.Layer.CreateAnonymous("rv1-test.usda")
    if not layer.ImportFromString(source.read_text()):
        raise RuntimeError("Cannot read RV1 test-stage template")
    prim = layer.GetPrimAtPath("/World/RV1")
    if prim is None:
        raise RuntimeError("RV1 template lacks /World/RV1")
    prim.referenceList.prependedItems = [Sdf.Reference(model)]
    destination = Path(run_dir) / ("rv1-test-" + uuid.uuid4().hex + ".usda")
    if not layer.Export(str(destination)):
        raise RuntimeError("Cannot export generated RV1 test stage")
    return str(destination)
