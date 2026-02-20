# **Ohmni Robot LiDAR Navigation Setup (ROS Melodic)**

This guide documents how to run LiDAR-based autonomous navigation on the **Ohmni Robot** using **ROS Melodic** and the Ohmni ROS Docker environment.
It includes environment setup, required commands, launch flow, and common issues.

---

## **1. System Requirements**

* **Ohmni Robot** with access to bot CLI
* **ROS Melodic** (inside Docker container)
* External **LiDAR sensor** (USB)

---

## **2. Prepare the Robot Environment**

### **2.1 Stop default Ohmni services**

Before running your own navigation stack, stop the default control node:

```bash
In bot cli> setprop ctl.stop tb-node
```

---

### **2.2 Enable LiDAR USB device**

Give permission for the LiDAR device:

```bash
sudo chmod 666 /dev/ttyUSB0
```

> ⚠️ If your LiDAR device uses another port (`/dev/ttyUSB1`, `/dev/ttyACM0`, etc.), update the command accordingly.

---

### **2.3 Check robot network configuration**

Verify the robot has a valid IP. You will use this IP for `ROS_IP` when running Docker.

---

## **3. Start the ROS Docker Environment**

Run the Ohmni ROS container with host networking and device access:

```bash
In bot cli> docker run -it \
  --network host \
  --privileged \
  -v /dev:/dev \
  -e ROS_IP=[bot_ip] \
  ohmnilabsvn/ohmni_ros:ohmni_ros_tbcontrol_0.0.13
```

Replace `[bot_ip]` with the robot’s actual IP address.

---

## **4. Inside the Docker Container**

### **4.1 Source ROS and your workspace**

```bash
source /opt/ros/melodic/setup.bash
source ~/catkin_ws/devel/setup.bash
```

---

### **4.2 Check available ROS topics and services**

Verify odometry and velocity control:

```bash
rostopic list
```

You should see:

* `/odom` (or `tb_control/odom_wheel`)
* `/tb_cmd_vel` (default Ohmni velocity topic)
* `/scan` (LiDAR)

---

## **5. Topic Remapping Notes**

### **5.1 Odometry**

Some Ohmni setups publish odom as:

```
/tb_control/odom_wheel
```

If your navigation stack expects **/odom**, remap it.
(Already handled in the provided launch files, but recheck if it fails.)

---

### **5.2 Velocity Command**

Ohmni listens on:

```
/tb_cmd_vel
```

But standard ROS navigation uses:

```
/cmd_vel
```

A remap is needed inside your launch file.
If navigation does not move the robot, recheck this mapping.

---

## **6. Editing Configuration**

Use:

```bash
roscd navigate
```

From here you can modify:

* **config files**
* **launch files**

There are two important launch files:

### **`discovery.launch`**

Used for **mapping** (creating the map with LiDAR).

### **`launch.launch`**

Used for **navigation** (localization + path planning).

---

## **7. Running Mapping and Navigation**

### **7.1 Create a Map**

Run:

```bash
roslaunch navigate discovery.launch
```

Drive the robot manually to build a map.

---

### **7.2 Run Navigation**

After the map is created:

```bash
roslaunch navigate launch.launch
```

---

## **8. Important Warning**

⚠️ **LiDAR may not initialize correctly during navigation**
If you see an error like:

```
cannot read lidar sensor
```

Then:

1. Run the LiDAR driver **alone** first
2. Then run:

```bash
roslaunch navigate launch.launch
```

This ensures the LiDAR node is fully active before navigation starts.

---

## **9. Troubleshooting**

### **Navigation does not move the robot**

* Check if `/cmd_vel` is correctly remapped to `/tb_cmd_vel`
* Run:

```bash
rostopic echo /tb_cmd_vel
```

If nothing prints → remap issue.

---

### **Odometry missing**

If `/odom` is not found:

* Check if `/tb_control/odom_wheel` is available
* Add remap in launch file:

  ```xml
  <remap from="/odom" to="/tb_control/odom_wheel"/>
  ```

---

### **LiDAR not detected**

Check device:

```bash
ls /dev/ttyUSB*
```

Fix permissions again:

```bash
sudo chmod 666 /dev/ttyUSB0
```

---

### **ROS communication issues**

Verify inside Docker:

```bash
echo $ROS_IP
echo $ROS_MASTER_URI
```

`ROS_MASTER_URI` should match bot IP.

---

## **10. Summary**

This README covers:

* Robot preparation
* Docker environment setup
* ROS topic remapping for Ohmni
* Mapping and navigation workflow
* LiDAR initialization fix
* Troubleshooting steps
