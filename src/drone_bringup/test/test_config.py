from pathlib import Path
import copy
import yaml
import jsonschema
import pytest
from drone_bringup.config import load_rig, camera_parameters, capture_parameters, imu_provenance

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


@pytest.fixture
def dds_diagnostic():
    """Timing metadata only; no sensor measurements or hardware acceptance."""
    return {"level": 0, "sync_ready": True,
            "timestamp_mode": "agent_synced_no_second_offset",
            "sync_protocol": "uxrce_dds_timesync_status",
            "host_minus_px4_offset_us": 12.5,
            "sync_uncertainty_us": 1200.0, "sync_budget_us": 1500.0}


def test_dds_provenance_preserves_the_recorded_run_budget(dds_diagnostic):
    provenance = imu_provenance(load_rig(RIG), {}, dds_diagnostic)
    assert provenance["timesync_snapshot"]["sync_budget_us"] == 1500.0
    assert provenance["timesync_snapshot"]["sync_uncertainty_us"] == 1200.0
    assert provenance["T_host_px4_offset_s"] == 12.5 / 1e6


def test_old_dds_provenance_keeps_the_one_millisecond_budget(dds_diagnostic):
    del dds_diagnostic["sync_budget_us"]
    dds_diagnostic["sync_uncertainty_us"] = 1000.0
    imu_provenance(load_rig(RIG), {}, dds_diagnostic)
    dds_diagnostic["sync_uncertainty_us"] = 1001.0
    with pytest.raises(ValueError, match="recorded budget"):
        imu_provenance(load_rig(RIG), {}, dds_diagnostic)


@pytest.mark.parametrize("key,value", [
    ("sync_budget_us", 0), ("sync_budget_us", -1),
    ("sync_budget_us", float("nan")), ("sync_budget_us", float("inf")),
    ("sync_budget_us", None), ("sync_budget_us", True),
    ("sync_uncertainty_us", -1), ("sync_uncertainty_us", float("nan")),
    ("sync_uncertainty_us", 1501.0),
])
def test_invalid_dds_timing_evidence_is_rejected(dds_diagnostic, key, value):
    dds_diagnostic[key] = value
    with pytest.raises(ValueError):
        imu_provenance(load_rig(RIG), {}, dds_diagnostic)


def test_missing_dds_uncertainty_is_rejected(dds_diagnostic):
    del dds_diagnostic["sync_uncertainty_us"]
    with pytest.raises(ValueError, match="missing or invalid"):
        imu_provenance(load_rig(RIG), {}, dds_diagnostic)
