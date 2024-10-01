# arm_moveit_config

This is the config generated from running the moveit setup assistant. This assumes you know what that is, and that you have moveit compiled and sourced.

Most of the important stuff you the reader probably care about is in the `./config` folder, which contains the following.

Because we will most likely be re-doing the generation (so it won't be called arm_three.urdf) at some point, I'm leaving this generic for now, indiciated by `<name>`

`<name>.urdf.xacro`
: the overall urdf, that combines the urdf from the solidworks export, and the auto-generated ros2_control urdf file `<name>.ros2_control.xacro`

`<name>.ros2_control.xacro`
: the ros2_control high level interface, which defines all the different command interfaces, and state interfaces. The `<hardware>` tags currently are defined for `gazebo_ros2_control`
