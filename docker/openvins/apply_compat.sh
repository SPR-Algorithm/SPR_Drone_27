#!/usr/bin/env bash
# Apply only the recorded compatibility patches; preserve unrelated edits.
set -euo pipefail

if (( $# > 1 )); then
  echo "Usage: bash docker/openvins/apply_compat.sh [OpenVINS checkout]" >&2
  exit 2
fi

compat_script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
compat_checkout=${1:-"$compat_script_dir/../../src/open_vins"}
compat_revision=69488123ed9362dd44b6f28e7f4680abbff1442b
compat_patch_names=(jazzy_compat.patch cxx20.patch)
compat_patch_hashes=(
  0a8cbc71ee55baefb673f51f6c2a9fa19b3956891518e2fcd15aaea926e608cf
  634aec98e1688c4fbfc1ee412fba914d8fff6dc60cfa593e386480cf0f56c6a1
)
compat_pending=()
compat_checkout=$(cd -- "$compat_checkout" && pwd)
if [[ "$(git -C "$compat_checkout" rev-parse --show-toplevel)" != "$compat_checkout" ]]; then
  echo "Argument must identify the root of the OpenVINS checkout." >&2
  exit 2
fi
if [[ "$(git -C "$compat_checkout" rev-parse HEAD)" != "$compat_revision" ]]; then
  echo "OpenVINS checkout must be exactly $compat_revision." >&2
  exit 2
fi

# Check the entire patch set before changing any file.
for compat_index in "${!compat_patch_names[@]}"; do
  compat_patch="$compat_script_dir/${compat_patch_names[$compat_index]}"
  compat_patch_sha256=${compat_patch_hashes[$compat_index]}
  if [[ "$(sha256sum "$compat_patch" | cut -d ' ' -f 1)" != "$compat_patch_sha256" ]]; then
    echo "Compatibility patch checksum differs from the reviewed patch: $compat_patch" >&2
    exit 2
  fi
  if git -C "$compat_checkout" apply --reverse --check "$compat_patch" >/dev/null 2>&1; then
    echo "OpenVINS ${compat_patch_names[$compat_index]} already applied ($compat_patch_sha256)."
  elif git -C "$compat_checkout" apply --check --whitespace=error-all "$compat_patch"; then
    compat_pending+=("$compat_patch")
  else
    echo "Compatibility patch conflicts or is partially applied; no files changed." >&2
    exit 2
  fi
done
if (( ${#compat_pending[@]} )); then
  git -C "$compat_checkout" apply --whitespace=error-all "${compat_pending[@]}"
  echo "Applied reviewed OpenVINS compatibility patches."
fi
