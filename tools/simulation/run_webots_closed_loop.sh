#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$ROOT"

OUTPUT_DIR="${1:-webots-closed-loop-output}"
if [[ "$OUTPUT_DIR" = /* ]]; then
  echo "output directory must be repository-relative: $OUTPUT_DIR" >&2
  exit 2
fi

WEBOTS_IMAGE="${WEBOTS_IMAGE:-ghcr.io/cyberbotics/webots-docker/webots@sha256:f31b128a3e4c06e54b26ce3d963a0e6b1c9634907978ae4397db9b4cde2d9f0c}"
TRACE="$OUTPUT_DIR/closed-loop-production-path.jsonl"
LOG="$OUTPUT_DIR/webots.log"
WORLD="tools/simulation/webots/worlds/single_wheel_closed_loop_equivalent.wbt"
FIXTURE="tools/simulation/fixtures/closed-loop-aggregate-equivalent.json"
mkdir -p "$OUTPUT_DIR"

# Fail before simulation if the high-fidelity rigid body no longer realizes the
# same upright reduced aggregates as the production bridge's synthetic model.
# This validates a physical realization, not a convenient impossible inertia.
python3 tools/simulation/validate_closed_loop_realization.py "$FIXTURE"

cargo build -p swp-sitl --bin webots_production_bridge \
  --target x86_64-unknown-linux-gnu

BRIDGE="target/x86_64-unknown-linux-gnu/debug/webots_production_bridge"
test -x "$BRIDGE"

# #17 validates semantic loop closure, not the disturbance envelope. Use a
# deliberately tiny pitch-only perturbation and a short evidence window.  The
# Webots body now has a physically realizable aggregate-equivalent model and its
# accelerometer is located at the production measurement model's axle origin.
# Larger/longer disturbances are explicitly owned by #18.
docker run --rm \
  -e LIBGL_ALWAYS_SOFTWARE=true \
  -e SWP_WEBOTS_CLOSED_LOOP=1 \
  -e SWP_WEBOTS_CONTROL_BRIDGE="/workspace/$BRIDGE" \
  -e SWP_WEBOTS_TRACE="/workspace/$TRACE" \
  -e SWP_WEBOTS_DURATION_S="${SWP_WEBOTS_DURATION_S:-0.030}" \
  -e SWP_WEBOTS_INITIAL_PITCH_RAD="${SWP_WEBOTS_INITIAL_PITCH_RAD:-0.0005}" \
  -e SWP_WEBOTS_INITIAL_ROLL_RAD="${SWP_WEBOTS_INITIAL_ROLL_RAD:-0.0}" \
  -v "$PWD:/workspace" \
  -w /workspace \
  "$WEBOTS_IMAGE" \
  bash -lc "set -o pipefail; timeout 90s xvfb-run --auto-servernum webots --stdout --stderr --batch --mode=fast --no-rendering /workspace/$WORLD 2>&1 | tee /workspace/$LOG"

if grep -q '^ERROR:' "$LOG"; then
  cat "$LOG"
  exit 1
fi

python3 tools/simulation/validate_webots_closed_loop.py "$TRACE" --minimum-records 10
