import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro


def generate_launch_description():

    # Specify the name of the package and path to xacro file within the package
    pkgName = 'micro_robot_arm'
    descriptionSubpath = 'description/arm.urdf.xacro'
    rvizConfigSubpath = 'launch/config.rviz'


    # Use xacro to process the file
    xacro_file = os.path.join(get_package_share_directory(pkgName), descriptionSubpath)
    robot_description_raw = xacro.process_file(xacro_file).toxml()

    rviz_config_file = os.path.join(get_package_share_directory(pkgName),rvizConfigSubpath)


    # Configure the node
    publisherUI = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui'
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        output="log",
        arguments=["-d", rviz_config_file],
    )


    # Run the node
    return LaunchDescription([
        publisherUI,
        rviz
    ])
