# MSD Spot Arm
This repository is for all of the ROS2 packages made for the MSD Spot robot arm. 

# Required software
- Ubuntu 22.04 (Jammy)
- ROS2 Humble

# Usage

# Building
Before building the packages, the repository needs to be cloned to a ROS workspace. This can be setup by running

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone <REPOSITORY PAGE>
cd ~/ros2_ws/
rosdep update --rosdistro=$ROS_DISTRO
sudo apt-get update
sudo rosdep install --from-paths ./ -i -y --rosdistro ${ROS_DISTRO}
```

Once the workspace is setup along with any dependencies, the packages can be built with

```bash
cd ~/ros2_ws/
. /opt/ros/${ROS_DISTRO}/setup.sh
colcon build --merge-install
```