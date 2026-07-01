#!/usr/bin/env bash
set -euo pipefail

MAP_NAME="${1:-aiclub}"
MAP_DIR="$(rospack find navigate)/map/$MAP_NAME"

mkdir -p "$MAP_DIR"

echo "Saving ROS map to $MAP_DIR/$MAP_NAME.yaml and $MAP_DIR/$MAP_NAME.pgm"
rosrun map_server map_saver -f "$MAP_DIR/$MAP_NAME"
