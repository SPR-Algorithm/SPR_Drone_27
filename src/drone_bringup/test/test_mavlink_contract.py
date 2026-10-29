"""Replay a real hardware diagnostic and validate the separate bench contract."""
import json
from pathlib import Path
from types import SimpleNamespace

import jsonschema
import pytest
import yaml

from drone_bringup.config import load_rig, imu_provenance
from drone_bringup.mavlink_contract import (diagnostic_snapshot, mavlink_parameters,
                                          profile_annotations, PROFILE)


ROOT = Path(__file__).resolve().parents[1]
RIG = ROOT / "config/rig/d435i_mavlink.yaml"
FIXTURE = Path(__file__).parent / "fixtures/mavlink_diagnostic_real_20261029.json"


def measured_snapshot():
    captured = json.loads(FIXTURE.read_text())["diagnostic"]
    status = SimpleNamespace(level=b"\x00", message=captured["message"], values=[
        SimpleNamespace(key=key, value=value) for key, value in captured.items()
        if key not in ("level", "message")])
    return diagnostic_snapshot(status, 1793279950254803141)


def test_mavlink_schema_and_real_provenance_are_separate_from_dds():
    rig = load_rig(RIG)
    assert rig["imu"]["min_rate_hz"] == 140.0
    assert "agent" not in rig
    assert rig["mavlink"]["requested_rate_hz"] == 1000.0
    params = mavlink_parameters(rig, "/tmp/real-capture-evidence")
    assert params["deployment_scope"] == "mavlink_140hz_test_only"
    assert params["imu_topic"] == "/spr/calibration/imu"
    provenance = imu_provenance(rig, {"commit": "a" * 40, "effective_clean": True}, measured_snapshot())
    assert provenance["source"] == "mavlink_highres_imu"
    assert provenance["raw_imu"] is False
    assert provenance["integration_dt_available"] is False
    assert provenance["bridge_code_dirty"] is False
    assert provenance["timesync_snapshot"]["rate_hz"] > 140
    assert isinstance(provenance["timesync_snapshot"]["reference_host_ns"], int)
    assert profile_annotations(PROFILE)["original_dds_200hz_requirement_satisfied"] is False
    dds = load_rig(ROOT / "config/rig/d435i.yaml")
    assert dds["imu"]["min_rate_hz"] == 200.0
    assert dds["imu"]["source"] == "px4_bridge"


@pytest.mark.parametrize("key,value", [("source", "px4_bridge"), ("topic", "/spr/imu"),
                                      ("min_rate_hz", 100.0)])
def test_mavlink_schema_rejects_mixed_or_weakened_contract(tmp_path, key, value):
    rig = yaml.safe_load(RIG.read_text())
    rig["imu"][key] = value
    target = tmp_path / "bad.yaml"
    target.write_text(yaml.safe_dump(rig))
    with pytest.raises(jsonschema.ValidationError):
        load_rig(target, ROOT / "config/rig_mavlink.schema.json")


@pytest.mark.parametrize("key,value", [("rate_hz", 139.0), ("clock_fit_residual_us", 1001.0),
                                      ("sync_protocol", "uxrce_dds_timesync_status"),
                                      ("deployment_scope", "dds_runtime"),
                                      ("reference_host_ns", 0)])
def test_provenance_rejects_insufficient_or_wrong_timing_evidence(key, value):
    snapshot = measured_snapshot()
    snapshot[key] = value
    with pytest.raises(ValueError):
        imu_provenance(load_rig(RIG), {}, snapshot)
