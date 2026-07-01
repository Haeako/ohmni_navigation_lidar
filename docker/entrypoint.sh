#!/bin/bash
# default ros_entrypoint.sh, with project navigation launch added in /ros_launch.sh
set -e

if [ -z "$TBCONTROL_DEVICE" ]; then
  export TBCONTROL_DEVICE="/dev/usb/tty1-2.1"
fi

echo "setup ros environment"
source "/opt/ros/${ROS_DISTRO}/setup.bash"

#echo "setup network"
#source "/root/ohmnilabs/tb-control-ros/setup_network.sh"

echo "launch basic nodes"
source /ros_launch.sh
exec "$@"
