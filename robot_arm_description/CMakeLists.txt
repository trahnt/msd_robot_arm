cmake_minimum_required(VERSION 3.5)
project(robot_arm_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY micro_arm/urdf micro_arm/rviz
  DESTINATION share/${PROJECT_NAME}/micro_arm
)

install(
  DIRECTORY spot_arm/urdf spot_arm/rviz
  DESTINATION share/${PROJECT_NAME}/spot_arm
)

ament_package()
