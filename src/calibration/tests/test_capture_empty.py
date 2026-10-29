"""Verify a connected ROS process cannot report an empty capture as success."""

import os
from pathlib import Path
import subprocess
import sys
import tempfile

import yaml


with tempfile.TemporaryDirectory(prefix="spr-empty-capture-") as directory:
    result = subprocess.run(
        [sys.argv[1], "--ros-args", "-p", "show_ui:=false", "-p",
         "input_timeout_sec:=0.4", "-p", f"output_dir:={directory}",
         "-p", f"left_image_topic:=/test_empty/p{os.getpid()}/left",
         "-p", f"right_image_topic:=/test_empty/p{os.getpid()}/right"],
        capture_output=True, text=True, timeout=10,
    )
    output = result.stdout + result.stderr
    assert result.returncode == 1, output
    assert "no synchronized stereo input" in output, output
    assert "capture incomplete: 0 pairs" in output, output
    metadata_paths = list(Path(directory).glob("*/meta.yaml"))
    assert len(metadata_paths) == 1, metadata_paths
    meta = yaml.safe_load(metadata_paths[0].read_text())
    assert meta["status"] == "empty", meta
    assert meta["accepted_pairs"] == 0, meta
    assert meta["board_cols"] == 8 and meta["board_rows"] == 11, meta
    assert meta["square_size_m"] == 0.025, meta
    assert not list(Path(directory).glob("*/left/*.png"))
    print("PASS: empty stereo capture rejected; portable YAML metadata retained")
