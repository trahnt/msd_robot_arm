# MSD Spot Arm
This repository is for all of the ROS2 packages made for the MSD Spot robot arm. 

# Required software
- Ubuntu 22.04 (Jammy)
- ROS2 Humble
- Ros2\_control
- Moveit

# Project Layout

`arm_motor_controller`
: This has the `ros2_control` controllers and their 

`arm_moveit_config`
: The moveit launch and launch configurations for inverse kinematics solving, and
motion planning

`arm_servoing`
: Contains the realtime servoing custom node for robot control and its
respective launch files and launch file configs

`sw_arm_desc`
: "Solidworks arm description". This is the result of the [Solidworks URDF
exporter](http://wiki.ros.org/sw_urdf_exporter), and the
[sw2urdf\_ros2](https://github.com/xiaoming-sun6/sw2urdf_ros2) tool. Contains
the true arm URDF that gets referenced by many other packages.



# Building

Build like you would any other ROS2 package. 

Before building the packages, the repository needs to be cloned to a ROS workspace. This can be setup by running
To set up a ROS2 workspace 

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone <REPOSITORY PAGE>
cd ~/ros2_ws/
rosdep update --rosdistro=$ROS_DISTRO
rosdep install --from-paths ./ -i -y --rosdistro ${ROS_DISTRO}
```
You may need to `sudo apt-get update` before the `rosdep install` step

Once the workspace is setup along with any dependencies, the packages can be built with

```bash
cd ~/ros2_ws/
. /opt/ros/${ROS_DISTRO}/setup.sh
colcon build --merge-install
source install/setup.sh
```
