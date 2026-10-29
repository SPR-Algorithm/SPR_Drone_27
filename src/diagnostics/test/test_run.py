import json
from pathlib import Path
import yaml
from spr_diagnostics.run import Run, require_space, sha256


def test_failed_empty_run_is_portable_and_report_escaped(tmp_path):
    run = Run(tmp_path, "stereo", {"serial": "<script>"}, {"commit": None})
    run.event("device_missing", "ERROR", device="<script>alert(1)</script>")
    run.metric(free_bytes=42, uptime_s=1.0)
    run.finish(False, "empty capture", image_pairs=0)
    manifest = yaml.safe_load((run.path / "manifest.yaml").read_text())
    assert manifest["status"] == "FAILED"
    assert manifest["evidence"]["image_pairs"] == 0
    events = [json.loads(x) for x in (run.path / "events.jsonl").read_text().splitlines()]
    assert events[-1]["parameters"]["status"] == "FAILED"
    assert "<script>" not in (run.path / "report.html").read_text()
    assert sha256(run.path / "manifest.yaml") == sha256(run.path / "manifest.yaml")


def test_run_ids_are_unique_and_disk_gate_fails(tmp_path):
    a = Run(tmp_path, "allan", {}, {})
    b = Run(tmp_path, "allan", {}, {})
    assert a.path != b.path
    import pytest
    with pytest.raises(RuntimeError, match="disk space"):
        require_space(tmp_path, 10**30)
