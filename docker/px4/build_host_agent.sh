#!/usr/bin/env bash
# Build only; never opens a serial device or changes FCU configuration.
set -eo pipefail

agent_revision=73622810d984349b80bbac0ef55fc0b694d62222
agent_tag=v2.4.3
agent_repository=https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
agent_logger_revision=eb3220622e73a4889eee355ffa37972b3cac3df5
agent_logger_repository=https://github.com/gabime/spdlog.git
agent_cache=${SPR_AGENT_CACHE:-"${XDG_CACHE_HOME:-$HOME/.cache}/spr/microxrce-agent/2.4.3"}
agent_prefix=${SPR_AGENT_PREFIX:-"$HOME/.local/opt/spr-microxrce-agent/2.4.3"}
agent_jobs=${SPR_AGENT_BUILD_JOBS:-2}

if [[ ! -f /opt/ros/jazzy/setup.bash ]]; then
  echo 'This pinned host build requires ROS 2 Jazzy in /opt/ros/jazzy.' >&2
  exit 2
fi
source /opt/ros/jazzy/setup.bash
set -u

for agent_command in git cmake g++ dpkg-query python3 rg ldd sha256sum; do
  if ! command -v "$agent_command" >/dev/null; then
    echo "Missing build dependency: $agent_command" >&2
    exit 2
  fi
done

agent_package_names=(ros-jazzy-fastrtps ros-jazzy-fastcdr cmake)
agent_package_versions=(2.14.6-1noble.20260303.233638 2.2.7-1noble.20260225.051855 3.28.3-1build7)
for agent_index in "${!agent_package_names[@]}"; do
  agent_actual=$(dpkg-query -W -f='${Version}' "${agent_package_names[$agent_index]}" 2>/dev/null || true)
  if [[ "$agent_actual" != "${agent_package_versions[$agent_index]}" ]]; then
    echo "Dependency pin mismatch: ${agent_package_names[$agent_index]} expected ${agent_package_versions[$agent_index]}, found ${agent_actual:-missing}. Review agent-lock.yaml before updating." >&2
    exit 2
  fi
done

mkdir -p "$agent_cache"
if [[ ! -d "$agent_cache/source/.git" ]]; then
  git clone --depth 1 --branch "$agent_tag" "$agent_repository" "$agent_cache/source"
fi
if [[ "$(git -C "$agent_cache/source" rev-parse HEAD)" != "$agent_revision" ]]; then
  echo "Agent source revision differs from pin: $agent_cache/source" >&2
  exit 2
fi
if [[ -n "$(git -C "$agent_cache/source" status --porcelain)" ]]; then
  echo "Agent source has local changes; refusing to overwrite: $agent_cache/source" >&2
  exit 2
fi

# Keep the upstream-supported logger private; never replace ROS/system spdlog.
if [[ ! -d "$agent_cache/spdlog/.git" ]]; then
  git clone --depth 1 --branch v1.9.2 "$agent_logger_repository" "$agent_cache/spdlog"
fi
if [[ "$(git -C "$agent_cache/spdlog" rev-parse HEAD)" != "$agent_logger_revision" ||
      -n "$(git -C "$agent_cache/spdlog" status --porcelain)" ]]; then
  echo "Logger source differs from clean pinned revision: $agent_cache/spdlog" >&2
  exit 2
fi
cmake -S "$agent_cache/spdlog" -B "$agent_cache/spdlog-build" \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="$agent_prefix" \
  -DCMAKE_INSTALL_LIBDIR=lib \
  -DCMAKE_POSITION_INDEPENDENT_CODE=ON \
  -DSPDLOG_BUILD_SHARED=OFF \
  -DSPDLOG_FMT_EXTERNAL=OFF \
  -DSPDLOG_BUILD_EXAMPLE=OFF \
  -DSPDLOG_BUILD_TESTS=OFF \
  -DSPDLOG_BUILD_BENCH=OFF
cmake --build "$agent_cache/spdlog-build" --parallel "$agent_jobs"
cmake --install "$agent_cache/spdlog-build"

cmake -S "$agent_cache/source" -B "$agent_cache/build" \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="$agent_prefix" \
  '-DCMAKE_INSTALL_RPATH=$ORIGIN/../lib;/opt/ros/jazzy/lib' \
  -Dspdlog_DIR="$agent_prefix/lib/cmake/spdlog" \
  -DUAGENT_SUPERBUILD=OFF \
  -DUAGENT_USE_SYSTEM_FASTDDS=ON \
  -DUAGENT_USE_SYSTEM_FASTCDR=ON \
  -DUAGENT_USE_SYSTEM_LOGGER=ON \
  -DUAGENT_CED_PROFILE=OFF \
  -DUAGENT_P2P_PROFILE=OFF \
  -DUAGENT_BUILD_TESTS=OFF \
  -DUAGENT_BUILD_USAGE_EXAMPLES=OFF
cmake --build "$agent_cache/build" --parallel "$agent_jobs"
cmake --install "$agent_cache/build"

# Upstream's help handler exits 1 after successfully printing usage.
agent_help_status=0
"$agent_prefix/bin/MicroXRCEAgent" --help > "$agent_cache/agent-help.txt" 2>&1 || agent_help_status=$?
if [[ "$agent_help_status" -gt 1 ]] || ! rg -q "^Usage: 'MicroXRCEAgent" "$agent_cache/agent-help.txt"; then
  cat "$agent_cache/agent-help.txt" >&2
  echo 'Agent help invocation failed.' >&2
  exit 2
fi
ldd -r "$agent_prefix/lib/libmicroxrcedds_agent.so" > "$agent_cache/agent-ldd.txt" 2>&1
ldd "$agent_prefix/bin/MicroXRCEAgent" >> "$agent_cache/agent-ldd.txt" 2>&1
if rg 'not found|undefined symbol' "$agent_cache/agent-ldd.txt"; then
  echo 'Agent runtime library resolution failed.' >&2
  exit 2
fi
{
  echo "agent_version=2.4.3"
  echo "agent_commit=$agent_revision"
  echo "logger_commit=$agent_logger_revision"
  echo 'logger_linkage=private_static_bundled_fmt'
  echo "agent_prefix=$agent_prefix"
  echo "ros_distro=$ROS_DISTRO"
  echo 'relocation_check=passed_ldd_r'
  echo "built_at_utc=$(date -u +%FT%TZ)"
  dpkg-query -W -f='${Package}=${Version}\n' "${agent_package_names[@]}"
  sha256sum "$agent_prefix/bin/MicroXRCEAgent" \
    "$agent_prefix/lib/libmicroxrcedds_agent.so" "$agent_prefix/lib/libspdlog.a"
} > "$agent_prefix/build-manifest.txt"
cp "$agent_cache/agent-ldd.txt" "$agent_prefix/runtime-libraries.txt"
echo "Built pinned agent: $agent_prefix/bin/MicroXRCEAgent"
echo "Manifest: $agent_prefix/build-manifest.txt"
