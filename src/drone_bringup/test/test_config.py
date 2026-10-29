from pathlib import Path
import copy
import yaml
import jsonschema
import pytest
from drone_bringup.config import load_rig, camera_parameters, capture_parameters

ROOT = Path(__file__).resolve().parents[1]
RIG = ROOT / "config/rig/d435i.yaml"


def test_real_contract_and_disabled_streams():
    rig = load_rig(RIG)
    params = camera_parameters(rig)
    assert params["depth_module.emitter_enabled"] == 0
    assert type(params["depth_module.emitter_enabled"]) is int
    assert all(params[key] is False for key in
               ("enable_gyro", "enable_accel", "enable_color", "enable_depth"))
    assert params["depth_module.enable_auto_exposure"] is False
    cap = capture_parameters(rig, "/tmp/test")
    assert cap["board_cols"] * cap["board_rows"] == 88
    assert cap["square_size"] == .025
    assert (cap["expected_width"], cap["expected_height"]) == (848, 480)


@pytest.mark.parametrize("section,key,value", [
    ("camera", "profile", "848,480,30"), ("board", "square_m", -1),
    ("camera", "enable_auto_exposure", True), ("imu", "source", "realsense"),
    ("imu", "min_rate_hz", 0), ("camera", "serial_no", ""),
])
def test_bad_rigs_fail_schema(tmp_path, section, key, value):
    rig = yaml.safe_load(RIG.read_text())
    rig[section][key] = value
    target = tmp_path / "bad.yaml"
    target.write_text(yaml.safe_dump(rig))
    with pytest.raises(jsonschema.ValidationError):
        load_rig(target, ROOT / "config/rig.schema.json")
