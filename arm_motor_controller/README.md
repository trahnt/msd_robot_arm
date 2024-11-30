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

## Motor configuration:
Which motors are used by which joints along their corresponding configuration parameters is set with the ros2_control.xacro to keep all the relevant information in one place. 


Motor direction is assumed to be positive in the clockwise direction

## TODOs

- Add support for ICL and Servo42D motors
- Implement enable/disable functionality in motors
