#!/bin/bash
# Dev launch script based on the manufacturer ros_launch.sh.
# It starts the base Ohmni nodes and the LiDAR navigation stack in tmux.

set -e

function wait_roscore {
    while [ -z "$(rostopic list 2>/dev/null | grep rosout)" ]; do sleep 0.2 ; done;
}
export -f wait_roscore

session="work"
OHMNI_WS="${OHMNI_WS:-/home/ohmni_navigation_lidar/melodic}"
MAP_NAME="${MAP_NAME:-aiclub}"
BASE_FRAME="${BASE_FRAME:-base_link}"
ODOM_FRAME="${ODOM_FRAME:-odom}"
SCAN_TOPIC="${SCAN_TOPIC:-scan}"
ODOM_TOPIC="${ODOM_TOPIC:-odom}"

tmux start-server
tmux has-session -t "$session" 2>/dev/null && tmux kill-session -t "$session"

tmux new-session -d -s "$session" -n base
tmux send-keys "source /opt/ros/${ROS_DISTRO}/setup.bash; roscore" C-m

tmux selectp -t 0
tmux splitw -v -p 50
tmux send-keys "source /opt/ros/${ROS_DISTRO}/setup.bash; wait_roscore; roslaunch tb_control tb_control.launch port_name:=$TBCONTROL_DEVICE use_servo_ext1:=true use_servo_ext2:=true use_servo_neck:=true" C-m

tmux selectp -t 1
tmux splitw -h -p 50
tmux send-keys "source /opt/ros/${ROS_DISTRO}/setup.bash; wait_roscore; cd $OHMNI_WS; if [ ! -f devel/setup.bash ]; then catkin_make; fi; source devel/setup.bash; roslaunch navigate launch.launch map_name:=$MAP_NAME base_frame:=$BASE_FRAME odom_frame:=$ODOM_FRAME scan_topic:=$SCAN_TOPIC odom_topic:=$ODOM_TOPIC" C-m

tmux select-layout -t "$session" tiled
