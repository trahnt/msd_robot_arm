# Copyright 2023 ros2_control Development Team
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
from launch.actions import RegisterEventHandler, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "gui",
            default_value="true",
            description="Start RViz2 automatically with this launch file.",
        )
    )

    should_publish = True

    # Get URDF and Moveit configuration
    moveit_config = (
        MoveItConfigsBuilder("arm_end_effector", package_name="arm_moveit_config")
        .robot_description(file_path="../arm_motor_controller/config/armtest.urdf.xacro")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_scene_monitor(
            publish_robot_description=True, publish_robot_description_semantic=True
        )
        .planning_pipelines(
            pipelines=["ompl"]#, "stomp", "pilz_industrial_motion_planner"]
        )
        .to_moveit_configs()
    )

    robot_controllers = PathJoinSubstitution(
        [FindPackageShare("arm_motor_controller"), "config", "arm_controllers.yaml"]
    )

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("arm_motor_controller"), "config", "moveit.rviz"]
    )

    controller_manager  = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[ robot_controllers],
        output="both",
        remappings=[
                ("~/robot_description", "robot_description"),
            ],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager-timeout",
            "300",
            "--controller-manager",
            "/controller_manager",
        ],
        remappings=[
                ("~/robot_description", "robot_description"),
            ],
    )

    # Publish TF
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description],
        remappings=[
                ("~/robot_description", "robot_description"),
            ],
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        # arguments=["arm_planning_group_controller", "--param-file", robot_controllers],
        arguments=["arm_planning_group_controller", "-c", "/controller_manager"],
        remappings=[
                ("~/robot_description", "robot_description"),
            ],
    )

    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        # arguments=["gripper_group_controller", "--param-file", robot_controllers],
        arguments=["gripper_group_controller", "-c", "/controller_manager"],
        remappings=[
                ("~/robot_description", "robot_description"),
            ],
    )

    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["--frame-id", "world", "--child-frame-id", "base_link"],
        remappings=[
                ("~/robot_description", "robot_description"),
            ],
    )

    move_group_configuration = {
        "publish_robot_description_semantic": True,
        "allow_trajectory_execution": True,
        # Note: Wrapping the following values is necessary so that the parameter value can be the empty string
        "capabilities": "",
        "disable_capabilities": "",
        # Publish the planning scene of the physical robot so that rviz plugin can know actual robot
        "publish_planning_scene": should_publish,
        "publish_geometry_updates": should_publish,
        "publish_state_updates": should_publish,
        "publish_transforms_updates": should_publish,
        "monitor_dynamics": False,
    }

    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict(), move_group_configuration],
        remappings=[
                ("~/robot_description", "robot_description"),
            ],
    )
    
    gui = LaunchConfiguration("gui")
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[moveit_config.robot_description],
        condition=IfCondition(gui),
    )

    # Delay rviz start after `joint_state_broadcaster`
    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        )
    )
    
    # delay_planning_group_controllers_after_robot_controller_spawner = RegisterEventHandler(
    #     event_handler=OnProcessExit(
    #         target_action=controller_manager,
    #         on_exit=[joint_state_broadcaster_spawner, arm_controller_spawner, gripper_controller_spawner],
    #     )
    # )

    nodes = [
        robot_state_publisher,
        controller_manager,
        static_tf,
        joint_state_broadcaster_spawner, 
        arm_controller_spawner, 
        gripper_controller_spawner,
        run_move_group_node,
        # delay_planning_group_controllers_after_robot_controller_spawner,
        delay_rviz_after_joint_state_broadcaster_spawner,
        # delay_joint_state_broadcaster_after_robot_controller_spawner,
    ]

    return LaunchDescription(declared_arguments + nodes)
