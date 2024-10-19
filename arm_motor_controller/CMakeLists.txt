cmake_minimum_required(VERSION 3.8)
project(arm_motor_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

# find dependencies
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

## Compile
add_library(
  arm_motor_controller
  SHARED
  src/ArmSystem.cpp
  src/Motor.cpp
)
target_compile_features(arm_motor_controller PUBLIC cxx_std_17)
target_include_directories(arm_motor_controller PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include/arm_motor_controller>
)
ament_target_dependencies(
  arm_motor_controller PUBLIC
  ${PACKAGE_INCLUDE_DEPENDS}
)

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface arm_motor_controller.xml)

# INSTALL
install(
  DIRECTORY include/
  DESTINATION include/arm_motor_controller
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

install(TARGETS arm_motor_controller
  EXPORT export_arm_motor_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_targets(export_arm_motor_controller HAS_LIBRARY_TARGET)
ament_export_dependencies(${PACKAGE_INCLUDE_DEPENDS})
ament_package()
