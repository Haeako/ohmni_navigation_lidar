# Ohmni Navigation LiDAR Run Manifest

This document records the exact run flow for starting the LiDAR navigation
stack on an Ohmni robot running Android with Docker.

## 1. Runtime Target

- Robot OS: Android
- ROS version: Melodic
- Docker image: `haeako/aiclub_ohmni:dev`
- Base image: `ohmnilabsvn/ohmni_ros:ohmni_ros_tbcontrol_0.0.13`
- Workspace path inside container: `/home/ohmni_navigation_lidar/melodic`
- Main launch file: `navigate launch.launch`

## 2. Manual Startup Flow

Open an Android shell on the robot:

```bash
adb shell
su
```

Stop the default Ohmni control node before running the custom navigation stack:

```bash
setprop ctl.stop tb-node
```

Allow access to the LiDAR device:

```bash
chmod 666 /dev/ttyUSB0
```

If the LiDAR appears on a different port, replace `/dev/ttyUSB0` with the
actual device, for example `/dev/ttyUSB1` or `/dev/ttyACM0`.

## 3. Start Or Recreate The Docker Container

Use host networking and privileged mode so ROS can communicate with the robot
and access the USB LiDAR.

Replace `<BOT_IP>` with the robot IP address.

Preferred Docker Compose flow:

```bash
cd /home/haeako/Documents/ohmni_navigation_lidar/docker
cp .env.example .env
```

Edit `.env` and set:

```bash
ROS_IP=<BOT_IP>
ROS_MASTER_URI=http://<BOT_IP>:11311
```

Build the image:

```bash
docker compose build
```

Start the container:

```bash
docker compose up -d
```

This creates the `melodic` container from `haeako/aiclub_ohmni:dev`, mounts the
workspace at `/home/ohmni_navigation_lidar/melodic`, and launches the dev test
stack in a tmux session named `work`.

The startup files are:

- `docker/entrypoint.sh`: based on the manufacturer `ros_entrypoint.sh`
- `docker/ros_launch.sh`: based on the manufacturer `ros_launch.sh`

The tmux session starts:

- `roscore`
- `roslaunch tb_control tb_control.launch`
- `roslaunch navigate launch.launch`

Equivalent raw Docker command:

```bash
docker rm -f melodic 2>/dev/null || true

docker run -itd \
  --name melodic \
  --restart unless-stopped \
  --network host \
  --privileged \
  -v /dev:/dev \
  -e ROS_IP=<BOT_IP> \
  -e ROS_MASTER_URI=http://<BOT_IP>:11311 \
  haeako/aiclub_ohmni:dev \
  bash
```

The `--restart unless-stopped` option makes Docker restart the container after a
robot reboot, as long as Docker itself starts after boot.

## 4. Attach To The Work Session

Enter the container:

```bash
docker exec -it melodic bash
```

If the image already starts a tmux session named `work`, attach to it:

```bash
tmux attach -t work
```

From outside the container, attach directly:

```bash
docker exec -it melodic tmux attach -t work
```

## 5. Run Navigation

Inside the container:

```bash
cd /home/ohmni_navigation_lidar/melodic
source /opt/ros/melodic/setup.bash
source devel/setup.bash
roslaunch navigate launch.launch
```

For mapping instead of navigation:

```bash
roslaunch navigate discovery.launch
```

When the map is ready, save it into `navigate/map/<map_name>/`:

```bash
roscd navigate
scripts/save_map.sh aiclub
```

Then launch navigation with that map:

```bash
roslaunch navigate launch.launch map_name:=aiclub
```

## 6. Automatic Boot Script

This repository includes:

```text
scripts/ohmni_nav_boot.sh
```

The script performs the boot sequence automatically:

- waits for Android boot services to settle
- stops `tb-node`
- gives permission to the LiDAR device
- waits for Docker
- detects `ROS_IP`
- starts the ROS navigation container
- launches `roslaunch navigate launch.launch`
- writes logs to `/data/local/tmp/ohmni_nav_boot.log`

Manual test:

```bash
sh scripts/ohmni_nav_boot.sh
tail -f /data/local/tmp/ohmni_nav_boot.log
docker logs -f ohmni_nav_lidar
```

Install for Magisk/root boot startup:

```bash
cp scripts/ohmni_nav_boot.sh /data/adb/service.d/ohmni_nav_boot.sh
chmod 755 /data/adb/service.d/ohmni_nav_boot.sh
```

After reboot, check:

```bash
docker ps
docker logs -f ohmni_nav_lidar
tail -f /data/local/tmp/ohmni_nav_boot.log
```

If using Docker Compose instead of the boot script:

```bash
cd /home/haeako/Documents/ohmni_navigation_lidar/docker
docker compose ps
docker compose logs -f melodic
```

## 7. Useful Checks

Check LiDAR device:

```bash
ls -l /dev/ttyUSB*
```

Check ROS environment inside the container:

```bash
echo "$ROS_IP"
echo "$ROS_MASTER_URI"
rostopic list
```

Expected topics include:

- `/scan`
- `/tb_cmd_vel`
- `/odom` or `/tb_control/odom_wheel`

Check whether navigation sends velocity commands:

```bash
rostopic echo /tb_cmd_vel
```

## 8. Common Problems

If the robot does not move:

- confirm `/cmd_vel` is remapped to `/tb_cmd_vel`
- confirm the default `tb-node` service was stopped
- confirm `/tb_cmd_vel` receives messages

If LiDAR fails to start:

- check the actual device path with `ls /dev/ttyUSB*`
- run `chmod 666` on the correct device
- restart the container after the device is available

If ROS nodes cannot communicate:

- confirm `ROS_IP` matches the robot IP
- confirm `ROS_MASTER_URI` is `http://<BOT_IP>:11311`
- use Docker host networking

## 9. Current Preferred Command Summary

```bash
adb shell
su
setprop ctl.stop tb-node
chmod 666 /dev/ttyUSB0
cd /home/haeako/Documents/ohmni_navigation_lidar/docker
docker compose up -d
docker exec -it melodic bash
cd /home/ohmni_navigation_lidar/melodic
source /opt/ros/melodic/setup.bash
source devel/setup.bash
roslaunch navigate launch.launch
```
