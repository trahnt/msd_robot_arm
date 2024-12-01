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

    # Get URDF via xacro
    # go int othe other package to get the code
    robot_description_content = Command( [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([
                FindPackageShare("arm_motor_controller"),
                "config",
                "armtest.urdf.xacro"
            ])
        ]
    )

    robot_description = {"robot_description": robot_description_content}

    # The yaml of the ros2_control controller yaml
    robot_controllers_yaml = PathJoinSubstitution(
        [
            FindPackageShare("marge"),
            "config",
            "marge_config.yaml",
        ]
    )

    # will prob fail but idc about rviz config file
    # rviz_config_file = PathJoinSubstitution(
        # [FindPackageShare("marge"), "r6bot/rviz", "view_robot.rviz"]
    # )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers_yaml],
        output="both",
        remappings=[
             ("~/robot_description", "/robot_description"),
         ],
    )


    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )


    '''
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
    '''
   


    gui = LaunchConfiguration("gui")
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        # arguments=["-d", rviz_config_file],
        condition=IfCondition(gui),
    )


    # tells the controller manager to spawn the jsb
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    marge_spawner = Node(
        package="controller_manager",
        executable="spawner",
        # may have to change 'marge_controller' here
        arguments=["marge_controller", "--param-file", robot_controllers_yaml],
    )
    
    # marge_spawner = Node(
        # package="controller_manager",
        # executable="spawner",
        # arguments=["marge_controller", "--param-file", robot_controllers],
    # )


    # I think something is broken here?
    # i can get ithis going just fine by doing 
    # `ros2 control load_controller marge_spawner` manually
    # marge_spawner = Node(
        # package="controller_manager",
        # executable="spawner",
        # arguments=["marge_controller", "-c", "/controller_manager"],
        # remappings=[
            # ("~/robot_description", "/robot_description"),
        # ],
    # )

    # Delay rviz start after `joint_state_broadcaster`
    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        )
    )

    # Delay start of joint_state_broadcaster after `robot_controller`
    # TODO(anyone): This is a workaround for flaky tests. Remove when fixed.
    delay_joint_state_broadcaster_after_marge_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=marge_spawner,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )

    '''
    nodes = [
        robot_state_pub_node,
        static_tf,
        controller_manager,
        # joint_state_broadcaster_spawner,
        # spawn_marge_after_cm,
        marge_spawner,
        delay_joint_state_broadcaster_after_marge_spawner,
        # delay_rviz_after_joint_state_broadcaster_spawner,
    ]
    '''

    nodes = [
        control_node,
        robot_state_pub_node,
        marge_spawner,
        delay_rviz_after_joint_state_broadcaster_spawner,
        delay_joint_state_broadcaster_after_marge_spawner,
    ]

    return LaunchDescription(declared_arguments + nodes)

