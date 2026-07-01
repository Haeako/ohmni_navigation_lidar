#!/system/bin/sh
#
# Start Ohmni LiDAR navigation after Android boot.
#
# Android normally has /system/bin/sh, not bash. This script is POSIX-sh
# compatible, so it can be started by Android init/Magisk and also by bash.
#
# Common install options on the robot:
#   Magisk/root: copy to /data/adb/service.d/ohmni_nav_boot.sh and chmod 755
#   Android init: call this script from an init .rc service as root
#   Manual test: sh scripts/ohmni_nav_boot.sh

set -u

CONTAINER_NAME="${CONTAINER_NAME:-ohmni_nav_lidar}"
DOCKER_IMAGE="${DOCKER_IMAGE:-haeako/aiclub_ohmni:dev}"
REPO_ROOT="${REPO_ROOT:-$(CDPATH= cd "$(dirname "$0")/.." && pwd)}"
CATKIN_WS="${CATKIN_WS:-$REPO_ROOT/melodic}"
CONTAINER_WS="${CONTAINER_WS:-/home/ohmni_navigation_lidar/melodic}"
LAUNCH_PACKAGE="${LAUNCH_PACKAGE:-navigate}"
LAUNCH_FILE="${LAUNCH_FILE:-launch.launch}"
LIDAR_DEVICE="${LIDAR_DEVICE:-/dev/ttyUSB0}"
BOOT_DELAY_SECONDS="${BOOT_DELAY_SECONDS:-25}"
DOCKER_WAIT_SECONDS="${DOCKER_WAIT_SECONDS:-90}"
LOG_FILE="${LOG_FILE:-/data/local/tmp/ohmni_nav_boot.log}"

log() {
  mkdir -p "$(dirname "$LOG_FILE")" 2>/dev/null || true
  printf '%s %s\n' "$(date '+%Y-%m-%d %H:%M:%S')" "$*" >>"$LOG_FILE"
}

run() {
  log "+ $*"
  "$@" >>"$LOG_FILE" 2>&1
}

get_robot_ip() {
  if command -v ip >/dev/null 2>&1; then
    ip route get 8.8.8.8 2>/dev/null | sed -n 's/.* src \([0-9.][0-9.]*\).*/\1/p' | head -n 1
    return
  fi

  if command -v ifconfig >/dev/null 2>&1; then
    ifconfig 2>/dev/null | sed -n 's/.*inet addr:\([0-9.][0-9.]*\).*/\1/p; s/.*inet \([0-9.][0-9.]*\).*/\1/p' |
      grep -v '^127\.' | head -n 1
  fi
}

wait_for_docker() {
  waited=0
  while [ "$waited" -lt "$DOCKER_WAIT_SECONDS" ]; do
    if docker info >/dev/null 2>&1; then
      return 0
    fi
    sleep 3
    waited=$((waited + 3))
  done
  return 1
}

main() {
  log "----- Ohmni navigation boot start -----"
  log "repo=$REPO_ROOT catkin_ws=$CATKIN_WS image=$DOCKER_IMAGE"

  sleep "$BOOT_DELAY_SECONDS"

  if command -v setprop >/dev/null 2>&1; then
    run setprop ctl.stop tb-node || true
  fi

  if [ -e "$LIDAR_DEVICE" ]; then
    run chmod 666 "$LIDAR_DEVICE" || true
  else
    log "warning: LiDAR device not found: $LIDAR_DEVICE"
  fi

  if ! command -v docker >/dev/null 2>&1; then
    log "error: docker command not found"
    exit 1
  fi

  if ! wait_for_docker; then
    log "error: docker daemon was not ready after ${DOCKER_WAIT_SECONDS}s"
    exit 1
  fi

  ROS_IP="${ROS_IP:-$(get_robot_ip)}"
  if [ -z "${ROS_IP:-}" ]; then
    log "error: could not detect ROS_IP; set ROS_IP manually before running"
    exit 1
  fi
  log "ROS_IP=$ROS_IP"

  run docker rm -f "$CONTAINER_NAME" || true

  run docker run -d \
    --name "$CONTAINER_NAME" \
    --restart unless-stopped \
    --network host \
    --privileged \
    -v /dev:/dev \
    -v "$CATKIN_WS/src:$CONTAINER_WS/src" \
    -e ROS_IP="$ROS_IP" \
    -e ROS_MASTER_URI="${ROS_MASTER_URI:-http://$ROS_IP:11311}" \
    "$DOCKER_IMAGE" \
    /bin/bash -lc "
      set -e
      source /opt/ros/melodic/setup.bash
      cd $CONTAINER_WS
      if [ ! -f devel/setup.bash ]; then
        catkin_make
      fi
      source $CONTAINER_WS/devel/setup.bash
      roslaunch $LAUNCH_PACKAGE $LAUNCH_FILE
    "

  log "started container $CONTAINER_NAME"
  log "view logs: docker logs -f $CONTAINER_NAME"
}

main "$@"
