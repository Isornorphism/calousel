#!/usr/bin/env bash
set -euo pipefail

configs=(
  examples/configs/discocal/discocal_cam0.yaml
  examples/configs/discocal/discocal_cam1.yaml
)

for cfg in "${configs[@]}"; do
  echo "==> ${cfg}"
  ros2 run discocal run_mono.py "${cfg}"
done
