To run this be sure to have ros2_control install along with the hardware interfaces

```shell
sudo apt install ros-humble-ros2-control ros-humble-ros2-controllers
sudo apt install -y ros-humble-hardware-interface
```

Workspace setup:
```shell
mkdir -p ros2WS/src
cd ros2_ws/src
git clone ...
cd ..
rosdep update --rosdistro=$ROS_DISTRO
sudo apt-get update
sudo rosdep install --from-paths ./ -i -y --rosdistro ${ROS_DISTRO}
```