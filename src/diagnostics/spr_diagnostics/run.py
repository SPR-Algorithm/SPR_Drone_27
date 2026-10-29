"""Run-scoped, portable evidence. This module has no ROS dependency."""
import csv
from datetime import datetime, timezone
import hashlib
import html
import json
import os
from pathlib import Path
import shutil
import socket
import subprocess
import tempfile
import uuid

import yaml


def utc_now():
    return datetime.now(timezone.utc).isoformat()


def sha256(path):
    h = hashlib.sha256()
    with Path(path).open("rb") as source:
        for block in iter(lambda: source.read(1024 * 1024), b""):
            h.update(block)
    return h.hexdigest()


OPENVINS_BASE = "69488123ed9362dd44b6f28e7f4680abbff1442b"
OPENVINS_PATCH_SHA256 = "0a8cbc71ee55baefb673f51f6c2a9fa19b3956891518e2fcd15aaea926e608cf"
OPENVINS_CXX20_PATCH_SHA256 = "634aec98e1688c4fbfc1ee412fba914d8fff6dc60cfa593e386480cf0f56c6a1"
OPENVINS_COMBINED_DIFF_SHA256 = "5c70ab56ae38bac04b71d69e1710b990ac42f71765892b24a24d90f4bda3b7c2"
OPENVINS_APPROVED_PATCHES = {
    "docker/openvins/jazzy_compat.patch": OPENVINS_PATCH_SHA256,
    "docker/openvins/cxx20.patch": OPENVINS_CXX20_PATCH_SHA256,
}
OPENVINS_PATCHED_FILES = {
    "ov_core/CMakeLists.txt": "609f3ae53e8dcacd6369e6d62c07014c4c97aa10f6153f396062cd32fac9bbdb",
    "ov_eval/CMakeLists.txt": "39152a7e54066db03d6909b78310ca4f040da37b17eb14a3b4966a5f98ceb4cd",
    "ov_init/CMakeLists.txt": "0a906bcdf8c2215e9ae835a7e4005d1191caa94913153c59310feeb76b64c24b",
    "ov_msckf/CMakeLists.txt": "970047d4541ddf49ef9db1768a1c5ef22bc61ef4d1a4617e49d393733f27ca2c",
    "ov_msckf/src/ros/ROS2Visualizer.h": "d1b14228b0779c81b029b3d6da1fe8494f4712641e4fb20827ac626b87b36b26",
    "ov_msckf/src/ros/ROSVisualizerHelper.h": "4594c6a8dde74c95fde48db78b0a07db9f1e6e858e5dff5d7206f659f4a2839e",
}


def git_baseline(root):
    """A clean root plus the exact committed compatibility patches is reproducible.

    Never hide status: retain the complete raw dirty baseline and explicitly
    describe the reviewed patch set. Untracked, staged, unrelated source
    edits, changed gitlinks, or changed patch bytes make effective_clean false.
    """
    root = Path(root).resolve()
    def command(*args, cwd=root):
        return subprocess.run(["git", "-C", str(cwd), *args], capture_output=True,
                              text=False, check=False)
    def git(*args, cwd=root):
        result = command(*args, cwd=cwd)
        return result.stdout.decode("utf-8", errors="replace").strip() if result.returncode == 0 else None
    baseline = {"describe": git("describe", "--tags", "--always", "--dirty"),
                "commit": git("rev-parse", "HEAD"),
                "status": git("status", "--porcelain=v1", "--ignore-submodules=none", "--untracked-files=all"),
                "submodules": git("submodule", "status"), "effective_clean": False,
                "approved_patches": {}, "provenance_errors": []}
    errors = baseline["provenance_errors"]
    if not baseline["commit"]:
        errors.append("Root Git HEAD is unavailable; an initial committed baseline is required")
    submodule = root / "src/open_vins"
    head = git("rev-parse", "HEAD", cwd=submodule)
    recorded = git("ls-tree", "HEAD", "--", "src/open_vins")
    expected_gitlink = f"160000 commit {OPENVINS_BASE}\tsrc/open_vins"
    if head != OPENVINS_BASE or recorded != expected_gitlink:
        errors.append("OpenVINS checkout and committed root gitlink must match the pinned base")
    if not errors and baseline["status"] == "":
        baseline["effective_clean"] = True
        return baseline
    # Root staged/untracked/modified files are never covered by the exception.
    root_other_changes = git("status", "--porcelain=v1", "--ignore-submodules=all", "--untracked-files=all")
    if root_other_changes is None or root_other_changes:
        errors.append("Root contains staged, untracked or modified files outside the approved submodule patch")
    if command("diff", "--cached", "--quiet").returncode != 0:
        errors.append("Root index contains uncommitted changes")
    tree = git("ls-tree", "-r", "HEAD") or ""
    for entry in tree.splitlines():
        if not entry.startswith("160000 "):
            continue
        metadata, name = entry.split("\t", 1)
        revision = metadata.split()[2]
        if name == "src/open_vins":
            continue
        other = root / name
        if (git("rev-parse", "HEAD", cwd=other) != revision or
                git("status", "--porcelain=v1", "--ignore-submodules=none", "--untracked-files=all", cwd=other) != ""):
            errors.append(f"Another submodule is changed or untracked: {name}")
    for patch_relative, expected_sha256 in OPENVINS_APPROVED_PATCHES.items():
        patch_path = root / patch_relative
        committed = command("show", "HEAD:" + patch_relative)
        if (not patch_path.is_file() or sha256(patch_path) != expected_sha256
                or committed.returncode != 0 or hashlib.sha256(committed.stdout).hexdigest() != expected_sha256):
            errors.append(f"Compatibility patch is missing, modified, or uncommitted: {patch_relative}")
    difference = command("diff", "--binary", "HEAD", cwd=submodule)
    diff_sha = hashlib.sha256(difference.stdout).hexdigest() if difference.returncode == 0 else None
    files = {name: sha256(submodule / name) if (submodule / name).is_file() else None
             for name in OPENVINS_PATCHED_FILES}
    if diff_sha != OPENVINS_COMBINED_DIFF_SHA256 or files != OPENVINS_PATCHED_FILES:
        errors.append("OpenVINS changes differ from the exact reviewed patch set or patched file hashes")
    if git("ls-files", "--others", "--exclude-standard", cwd=submodule):
        errors.append("OpenVINS contains untracked files")
    if command("diff", "--cached", "--quiet", cwd=submodule).returncode != 0:
        errors.append("OpenVINS index contains uncommitted staged changes")
    if not errors:
        baseline["effective_clean"] = True
        baseline["approved_patches"]["open_vins"] = {
            "base_commit": OPENVINS_BASE,
            "patches": [{"path": name, "sha256": digest}
                        for name, digest in OPENVINS_APPROVED_PATCHES.items()],
            "diff_sha256": diff_sha,
            "patched_files_sha256": files}
    return baseline


def atomic_yaml(path, document):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, temporary = tempfile.mkstemp(prefix=path.name + ".", dir=path.parent)
    try:
        with os.fdopen(fd, "w") as output:
            yaml.safe_dump(document, output, sort_keys=False)
            output.flush()
            os.fsync(output.fileno())
        os.replace(temporary, path)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)


def require_space(path, min_free_bytes):
    free = shutil.disk_usage(path).free
    if free < min_free_bytes:
        raise RuntimeError(f"disk space insufficient: {free} < {min_free_bytes} bytes")
    return free


class Run:
    def __init__(self, root, mode, config, baseline):
        stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S")
        self.path = Path(root) / (stamp + "_" + uuid.uuid4().hex[:8])
        self.path.mkdir(parents=True, exist_ok=False)
        (self.path / "ros").mkdir()
        self.manifest = {
            "schema_version": 1, "run_id": self.path.name, "mode": mode,
            "started_at": utc_now(), "status": "RUNNING", "git": baseline,
            "config": config, "host": socket.gethostname(), "artifacts": {},
        }
        self.write()
        self.event("run_started", mode=mode)

    def write(self):
        atomic_yaml(self.path / "manifest.yaml", self.manifest)

    def event(self, kind, severity="INFO", component="drone_bringup", **parameters):
        entry = {"timestamp": utc_now(), "host": socket.gethostname(),
                 "component": component, "severity": severity, "type": kind,
                 "parameters": parameters}
        with (self.path / "events.jsonl").open("a") as stream:
            stream.write(json.dumps(entry, allow_nan=False) + "\n")
            stream.flush()

    def metric(self, **values):
        fields = ["timestamp", *values]
        destination = self.path / "metrics.csv"
        new = not destination.exists()
        with destination.open("a", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=fields)
            if new:
                writer.writeheader()
            writer.writerow({"timestamp": utc_now(), **values})

    def finish(self, success, reason, **evidence):
        # Caller must prove the mode-specific output exists and is nonempty.
        self.manifest.update(status="SUCCEEDED" if success else "FAILED",
                             finished_at=utc_now(), reason=reason, evidence=evidence)
        self.event("run_finished", "INFO" if success else "ERROR",
                   status=self.manifest["status"], reason=reason)
        self.write()
        self.report()

    def report(self):
        events = []
        for line in (self.path / "events.jsonl").read_text().splitlines():
            events.append(json.loads(line))
        first_error = next((e for e in events if e["severity"] == "ERROR"), None)
        details = html.escape(yaml.safe_dump(self.manifest, sort_keys=False))
        timeline = "".join(
            "<tr><td>" + html.escape(e["timestamp"]) + "</td><td>" +
            html.escape(e["severity"]) + "</td><td>" + html.escape(e["type"]) +
            "</td><td><pre>" + html.escape(json.dumps(e["parameters"])) +
            "</pre></td></tr>" for e in events)
        report = ('<!doctype html><meta charset="utf-8"><title>SPR run report</title>'
                  '<style>body{font:16px sans-serif;margin:2rem}table{border-collapse:collapse}'
                  'td,th{border:1px solid #ccc;padding:.4rem}pre{white-space:pre-wrap}</style>'
                  '<h1>Run ' + html.escape(self.path.name) + '</h1><p>Status: ' +
                  html.escape(self.manifest["status"]) + '</p><h2>First error</h2><pre>' +
                  html.escape(json.dumps(first_error, ensure_ascii=False)) +
                  '</pre><h2>Manifest</h2><pre>' + details +
                  '</pre><h2>Timeline</h2><table><tr><th>UTC</th><th>Level</th>'
                  '<th>Event</th><th>Parameters</th></tr>' + timeline +
                  '</table><p>Metrics: <a href="metrics.csv">CSV</a>. '
                  'No ground truth was evaluated in this capture report.</p>')
        (self.path / "report.html").write_text(report)
