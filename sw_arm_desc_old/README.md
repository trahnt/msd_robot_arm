# sw_arm_desc_old
This package contains the core robot description (URDF) and a simple script for
viewing the robot in Rviz with the joint state publisher gui.

```bash
ros2 launch sw_arm_desc_old launch.py
```

This is the folder output by the [solidworks URDF export tool](http://wiki.ros.org/sw_urdf_exporter). And [sw2urdf_ros2](https://github.com/xiaoming-sun6/sw2urdf_ros2) Some manual modifications were made, such as the joint limits, but most of this is as it was "out of the box". 

# Future Improvements

- create simplified collision meshes, moveit's planning is slow
- The gripper's complex kinematics (I believe) are impossible to model with
  URDF. The gripper has been replaced with a simple box based gripper that was
  manually written in the URDF
