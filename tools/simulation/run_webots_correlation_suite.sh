#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$ROOT"

OUTPUT_DIR="${1:-webots-output}"
if [[ "$OUTPUT_DIR" = /* ]]; then
  echo "output directory must be repository-relative: $OUTPUT_DIR" >&2
  exit 2
fi

WEBOTS_IMAGE="${WEBOTS_IMAGE:-ghcr.io/cyberbotics/webots-docker/webots@sha256:f31b128a3e4c06e54b26ce3d963a0e6b1c9634907978ae4397db9b4cde2d9f0c}"
export SWP_WEBOTS_IMAGE="$WEBOTS_IMAGE"
mkdir -p "$OUTPUT_DIR"

experiments=(
  synthetic-free-response.json
  synthetic-drive-torque-pulse.json
  synthetic-reaction-torque-pulse.json
  synthetic-small-angle.json
  synthetic-zero-input-equilibrium.json
)

suite_failed=0

for name in "${experiments[@]}"; do
  stem="${name%.json}"
  experiment="tools/simulation/experiments/$name"
  fixture="$OUTPUT_DIR/$stem.fixture.json"
  rust_trace="$OUTPUT_DIR/$stem.rust.json"
  rust_projected="$OUTPUT_DIR/$stem.rust-projected.jsonl"
  analytical_trace="$OUTPUT_DIR/$stem.analytical.jsonl"
  webots_trace="$OUTPUT_DIR/$stem.webots.jsonl"
  summary="$OUTPUT_DIR/$stem.summary.json"
  log="$OUTPUT_DIR/$stem.webots.log"

  python3 tools/simulation/validate_experiment.py "$experiment"
  python3 tools/simulation/open_loop_correlation.py \
    --experiment "$experiment" \
    --materialize-reduced-fixture "$fixture"

  cargo run -p swp-sitl --bin plant_reference_trace \
    --target x86_64-unknown-linux-gnu -- \
    --fixture "$fixture" \
    --output "$rust_trace"

  docker run --rm \
    -e LIBGL_ALWAYS_SOFTWARE=true \
    -e SWP_EXPERIMENT="/workspace/$experiment" \
    -e SWP_WEBOTS_TRACE="/workspace/$webots_trace" \
    -v "$PWD:/workspace" \
    -w /workspace \
    "$WEBOTS_IMAGE" \
    bash -lc "set -o pipefail; timeout 90s xvfb-run --auto-servernum webots --stdout --stderr --batch --mode=fast --no-rendering /workspace/tools/simulation/webots/worlds/single_wheel_synthetic.wbt 2>&1 | tee /workspace/$log"

  if grep -q '^ERROR:' "$log"; then
    cat "$log"
    exit 1
  fi

  python3 tools/simulation/validate_webots_trace.py "$webots_trace" --minimum-records 80

  # The low-level comparator writes evidence even when its legacy scalar
  # equilibrium check returns non-zero. The suite finalizer then applies the
  # dimensioned acceptance policy and becomes the authoritative verdict.
  python3 tools/simulation/open_loop_correlation.py \
    --experiment "$experiment" \
    --rust-trace "$rust_trace" \
    --webots-trace "$webots_trace" \
    --analytical-trace "$analytical_trace" \
    --rust-projected-trace "$rust_projected" \
    --summary "$summary" || true

  if ! python3 tools/simulation/finalize_correlation_summary.py \
    --summary "$summary" \
    --analytical-trace "$analytical_trace" \
    --rust-projected-trace "$rust_projected" \
    --webots-trace "$webots_trace"; then
    suite_failed=1
  fi
done

if ! python3 tools/simulation/summarize_correlation_suite.py \
  --input-dir "$OUTPUT_DIR" \
  --output "$OUTPUT_DIR/suite-summary.json"; then
  suite_failed=1
fi

exit "$suite_failed"
