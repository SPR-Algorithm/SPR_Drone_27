#!/usr/bin/env python3
"""Combine real, compatible capture sessions without modifying source frames."""

import argparse
import csv
from datetime import datetime, timezone
import hashlib
import io
import json
import math
import os
from pathlib import Path
import shutil
import sys
import tempfile

import yaml


COMPATIBLE_KEYS = (
    "board_cols", "board_rows", "square_size_m", "auto_orientation",
    "resolution_width", "resolution_height", "left_topic", "right_topic",
    "left_frame_id", "right_frame_id", "expected_encoding",
    "factory_valid", "factory_k_left", "factory_d_left", "factory_k_right",
    "factory_d_right", "blur_threshold", "edge_margin", "max_stereo_delta_sec",
)


def sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def equivalent(left, right):
    if isinstance(left, (float, int)) and isinstance(right, (float, int)):
        return math.isfinite(left) and math.isfinite(right) and math.isclose(left, right, rel_tol=1e-9, abs_tol=1e-9)
    if type(left) is not type(right):
        return False
    if isinstance(left, dict):
        return left.keys() == right.keys() and all(equivalent(left[key], right[key]) for key in left)
    if isinstance(left, list):
        return len(left) == len(right) and all(equivalent(a, b) for a, b in zip(left, right))
    return left == right


def read_session(path):
    path = path.resolve(strict=True)
    metadata_path = path / "meta.yaml"
    index_path = path / "index.csv"
    metadata_bytes = metadata_path.read_bytes()
    index_bytes = index_path.read_bytes()
    metadata = yaml.safe_load(metadata_bytes)
    for key in COMPATIBLE_KEYS:
        if key not in metadata:
            raise ValueError(f"{path}: missing compatibility metadata {key}")
    # Capture sessions live at runs/<run>/images/<session>. Retain the exact
    # driver configuration, especially exposure/gain and the selected serial.
    camera_path = path / "camera_params.yaml"
    if not camera_path.exists():
        camera_path = path.parent.parent / "camera_params.yaml"
    camera_bytes = camera_path.read_bytes()
    camera = yaml.safe_load(camera_bytes)
    parameters = camera.get("/**", {}).get("ros__parameters", {})
    if parameters.get("depth_module.enable_auto_exposure") is not False or \
            "depth_module.exposure" not in parameters or "depth_module.gain" not in parameters:
        raise ValueError(f"{path}: a recorded fixed exposure/gain configuration is required")
    reader = csv.DictReader(io.StringIO(index_bytes.decode()))
    rows = list(reader)
    if not rows:
        raise ValueError(f"{path}: empty capture index")
    required = {"name", "stamp_ns", "right_stamp_ns", "blur_left", "blur_right", "min_margin"}
    if not required.issubset(reader.fieldnames or []):
        raise ValueError(f"{path}: index lacks exact timestamps or image-quality fields")
    names = [row["name"] for row in rows]
    if len(set(names)) != len(names) or any(Path(name).name != name for name in names):
        raise ValueError(f"{path}: duplicate or unsafe image names")
    for side in ("left", "right"):
        actual = {item.stem for item in (path / side).glob("*.png")}
        if actual != set(names):
            raise ValueError(f"{path}/{side}: PNG/index mismatch")
    for row in rows:
        values = [float(row[key]) for key in ("blur_left", "blur_right", "min_margin")]
        if not all(math.isfinite(value) for value in values):
            raise ValueError(f"{path}: non-finite image quality")
        if min(values[:2]) < metadata["blur_threshold"] or values[2] < metadata["edge_margin"]:
            raise ValueError(f"{path}/{row['name']}: image-quality gate failed; cannot combine into a formal candidate")
        left_stamp, right_stamp = int(row["stamp_ns"]), int(row["right_stamp_ns"])
        if min(left_stamp, right_stamp) <= 0 or abs(right_stamp-left_stamp) > metadata["max_stereo_delta_sec"] * 1e9:
            raise ValueError(f"{path}/{row['name']}: invalid stereo timestamps")
    manifest = path.parent.parent / "manifest.yaml"
    run_info = yaml.safe_load(manifest.read_bytes()) if manifest.exists() else {}
    return {"path": path, "meta": metadata, "meta_bytes": metadata_bytes,
            "index_bytes": index_bytes, "rows": rows, "fields": reader.fieldnames,
            "camera": camera, "camera_path": camera_path, "camera_bytes": camera_bytes,
            "run_manifest": manifest if manifest.exists() else None,
            "run_status": run_info.get("status"), "run_experimental": run_info.get("experimental")}


def combine(sessions, output, copy_images=False):
    sources = [read_session(path) for path in sessions]
    first = sources[0]
    for source in sources[1:]:
        for key in COMPATIBLE_KEYS:
            if not equivalent(first["meta"][key], source["meta"][key]):
                raise ValueError(f"{source['path']}: incompatible {key}")
        if not equivalent(first["camera"], source["camera"]):
            raise ValueError(f"{source['path']}: camera profile/serial/exposure/gain configuration differs")
        if source["fields"] != first["fields"]:
            raise ValueError(f"{source['path']}: index schema differs")
    output = output.resolve()
    if any(output == source["path"] or output.is_relative_to(source["path"]) for source in sources):
        raise ValueError("combined output must be outside every original capture session")
    if output.exists():
        raise FileExistsError(f"refusing to overwrite {output}")
    output.parent.mkdir(parents=True, exist_ok=True)
    temporary = Path(tempfile.mkdtemp(prefix=output.name + ".tmp-", dir=output.parent))
    try:
        for side in ("left", "right"):
            (temporary / side).mkdir()
        pending = sorted(((int(row["stamp_ns"]), number, row) for number, source in enumerate(sources)
                          for row in source["rows"]), key=lambda item: (item[0], item[1]))
        mapping, rows, seen, duplicates = [], [], {}, []
        for stamp, source_number, row in pending:
            source = sources[source_number]
            paths = {side: source["path"] / side / (row["name"] + ".png") for side in ("left", "right")}
            hashes = {side: sha256(path) for side, path in paths.items()}
            identity = (stamp, int(row["right_stamp_ns"]))
            if identity in seen:
                if seen[identity] != hashes:
                    raise ValueError("identical stereo timestamps refer to different pixels")
                duplicates.append({"source": source_number, "source_name": row["name"], "stamp_ns": stamp})
                continue
            seen[identity] = hashes
            name = f"{len(rows):06d}"
            new_row = dict(row, name=name)
            rows.append(new_row)
            mapping.append({"name": name, "source": source_number, "source_name": row["name"],
                            "stamp_ns": stamp, "right_stamp_ns": identity[1],
                            "images": {side: {"path": str(path), "sha256": hashes[side]} for side, path in paths.items()}})
            for side, path in paths.items():
                destination = temporary / side / (name + ".png")
                if copy_images:
                    shutil.copy2(path, destination)
                else:
                    destination.symlink_to(os.path.relpath(path, temporary / side))
        metadata = dict(first["meta"])
        metadata.update(status="combined_requires_quality_evaluation", accepted_pairs=len(rows),
                        source="real capture sessions; see combination_manifest.yaml",
                        source_session_count=len(sources), coverage_left_percent=None)
        (temporary / "meta.yaml").write_text(yaml.safe_dump(metadata, sort_keys=False))
        (temporary / "camera_params.yaml").write_bytes(first["camera_bytes"])
        with (temporary / "index.csv").open("w", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=first["fields"])
            writer.writeheader()
            writer.writerows(rows)
        for source in sources:
            if (source["path"] / "meta.yaml").read_bytes() != source["meta_bytes"] or \
                    (source["path"] / "index.csv").read_bytes() != source["index_bytes"] or \
                    source["camera_path"].read_bytes() != source["camera_bytes"]:
                raise ValueError("a source session changed during combination; stop capture before combining")
        manifest = {"schema_version": 1, "created_utc": datetime.now(timezone.utc).isoformat(),
                    "kind": "REAL_CAPTURE_COMBINATION", "formal_calibration_complete": False,
                    "storage": "copied_pngs" if copy_images else "relative_symlinks_keep_original_sessions",
                    "paired_images": len(rows), "deduplicated": duplicates,
                    "sources": [{"path": str(source["path"]),
                                 "meta_sha256": hashlib.sha256(source["meta_bytes"]).hexdigest(),
                                 "index_sha256": hashlib.sha256(source["index_bytes"]).hexdigest(),
                                 "camera_config_path": str(source["camera_path"]),
                                 "camera_config_sha256": hashlib.sha256(source["camera_bytes"]).hexdigest(),
                                 "run_manifest_path": str(source["run_manifest"]) if source["run_manifest"] else None,
                                 "run_manifest_sha256": sha256(source["run_manifest"]) if source["run_manifest"] else None,
                                 "run_status": source["run_status"], "run_experimental": source["run_experimental"]}
                                for source in sources],
                    "combined_meta_sha256": sha256(temporary / "meta.yaml"),
                    "combined_index_sha256": sha256(temporary / "index.csv"), "mapping": mapping}
        (temporary / "combination_manifest.yaml").write_text(yaml.safe_dump(manifest, sort_keys=False))
        temporary.rename(output)
        return {"output": str(output), "paired_images": len(rows), "formal_calibration_complete": False}
    except BaseException:
        shutil.rmtree(temporary)
        raise


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--session", type=Path, action="append", required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--copy", action="store_true", help="Copy original PNG bytes instead of relative symlinks")
    args = parser.parse_args()
    try:
        if len(args.session) < 2:
            raise ValueError("provide at least two --session arguments")
        print(json.dumps(combine(args.session, args.output, args.copy), indent=2))
        return 0
    except (OSError, ValueError, KeyError, yaml.YAMLError) as error:
        print(f"FAILED: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
