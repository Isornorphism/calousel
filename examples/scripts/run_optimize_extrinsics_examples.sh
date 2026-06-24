#!/usr/bin/env bash
set -euo pipefail

configs=(
  examples/configs/calousel/calousel_testbed_a.yaml
  examples/configs/calousel/calousel_testbed_b.yaml
)

for cfg in "${configs[@]}"; do
  echo "==> ${cfg}"
  ros2 run calousel optimize_extrinsics_cli --config "${cfg}"
done
