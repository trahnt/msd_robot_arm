from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch


# Comment buried says:
"""
Includes:
     * static_virtual_joint_tfs
     * robot_state_publisher
     * move_group
     * moveit_rviz
     * warehouse_db (optional)
     * ros2_control_node + controller spawners
"""

def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("arm_three", package_name="arm_moveit_config").to_moveit_configs()
    return generate_demo_launch(moveit_config)
