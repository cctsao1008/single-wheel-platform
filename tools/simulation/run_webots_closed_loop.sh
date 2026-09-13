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
mkdir -p "$OUTPUT_DIR"

cargo build -p swp-sitl --bin webots_production_bridge \
  --target x86_64-unknown-linux-gnu

BRIDGE="target/x86_64-unknown-linux-gnu/debug/webots_production_bridge"
test -x "$BRIDGE"

# The initial attitude is intentionally small. This run validates semantic loop
# closure and authority causality; disturbance-envelope characterization belongs
# to #18 and is not smuggled into this acceptance test.
docker run --rm \
  -e LIBGL_ALWAYS_SOFTWARE=true \
  -e SWP_WEBOTS_CLOSED_LOOP=1 \
  -e SWP_WEBOTS_CONTROL_BRIDGE="/workspace/$BRIDGE" \
  -e SWP_WEBOTS_TRACE="/workspace/$TRACE" \
  -e SWP_WEBOTS_DURATION_S="${SWP_WEBOTS_DURATION_S:-0.40}" \
  -e SWP_WEBOTS_INITIAL_PITCH_RAD="${SWP_WEBOTS_INITIAL_PITCH_RAD:-0.005}" \
  -e SWP_WEBOTS_INITIAL_ROLL_RAD="${SWP_WEBOTS_INITIAL_ROLL_RAD:--0.005}" \
  -v "$PWD:/workspace" \
  -w /workspace \
  "$WEBOTS_IMAGE" \
  bash -lc "set -o pipefail; timeout 90s xvfb-run --auto-servernum webots --stdout --stderr --batch --mode=fast --no-rendering /workspace/tools/simulation/webots/worlds/single_wheel_synthetic.wbt 2>&1 | tee /workspace/$LOG"

if grep -q '^ERROR:' "$LOG"; then
  cat "$LOG"
  exit 1
fi

python3 tools/simulation/validate_webots_closed_loop.py "$TRACE" --minimum-records 100
