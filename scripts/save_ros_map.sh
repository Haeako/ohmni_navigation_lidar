#!/usr/bin/env bash
set -euo pipefail

MAP_NAME="${1:-aiclub}"
ROOT_DIR="$(CDPATH= cd "$(dirname "$0")/.." && pwd)"
MAP_DIR="$ROOT_DIR/melodic/src/navigate/map/$MAP_NAME"

mkdir -p "$MAP_DIR"

echo "Saving ROS map to $MAP_DIR/$MAP_NAME.yaml and $MAP_DIR/$MAP_NAME.pgm"
rosrun map_server map_saver -f "$MAP_DIR/$MAP_NAME"
