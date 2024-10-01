# sw_arm_desc
This package contains the core robot description (URDF) and a simple script for
viewing the robot in Rviz with the joint state publisher gui.

```bash
ros2 launch sw_arm_desc launch.py
```

This is the folder output by the [solidworks URDF export tool](http://wiki.ros.org/sw_urdf_exporter). And [sw2urdf_ros2](https://github.com/xiaoming-sun6/sw2urdf_ros2) Some manual modifications were made, such as the joint limits, but most of this is as it was "out of the box". 

# Future Improvements

- create simplified collision meshes, moveit's planning is incredibly slow
- Re-export, but this time with proper gripper manipulation. Right now, the gripper doesn't move at all, and we're using the 6th DoF as the pseudo-gripper. This is fine for now, but this will need to be changed for the final product
