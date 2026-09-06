#!/usr/bin/env bash
set -euo pipefail

if ! command -v arm-none-eabi-size >/dev/null 2>&1; then
  echo "error: arm-none-eabi-size not found" >&2
  exit 1
fi

cargo build -p swp-stm32f103-observation --release --target thumbv7m-none-eabi
cargo build -p swp-control-footprint-stm32f103 --release --target thumbv7m-none-eabi

baseline="target/thumbv7m-none-eabi/release/swp-stm32f103-observation"
control="target/thumbv7m-none-eabi/release/swp-control-footprint-stm32f103"

echo
echo "Observation-only firmware"
arm-none-eabi-size "$baseline"

echo
echo "Full control-path footprint probe"
arm-none-eabi-size "$control"

echo
echo "Section detail: full control-path footprint probe"
arm-none-eabi-size -A "$control"
