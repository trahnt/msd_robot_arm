# https://github.com/ros-controls/ros2_control_demos/blob/7b7f70c992e1ab6a42ec095a24f2c731573d4e28/example_7/CMakeLists.txt

cmake_minimum_required(VERSION 3.8)
project(marge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# I think I need these?
set(CONTROLLER_INCLUDE_DEPENDS
    pluginlib
    rcpputils
    controller_interface
    realtime_tools
    trajectory_msgs
)


#####################
# find dependencies #
#####################

find_package(ament_cmake REQUIRED)

# Depends on verison of ros2_control you have
find_package(controller_manager 2.43.1)

if(NOT controller_manager_FOUND)
  message(FATAL_ERROR "use ros2_control version 2.43.1 or higher u ding dong. "
      "edit CMakeLists.txt, get a different version, or just quit trying")
endif()

foreach(dep IN ITEMS ${CONTROLLER_INCLUDE_DEPENDS})
    find_package(${dep} REQUIRED)
endforeach()


## Compile stuff

# maybe?
add_library(
    marge
    SHARED
    src/marge.cpp
)


# dawg i'm not gonna lie idek 
target_include_directories(marge PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
)

ament_target_dependencies(
    marge PUBLIC
    ${CONTROLLER_INCLUDE_DEPENDS}
)

# Export controller plugin
pluginlib_export_plugin_description_file(controller_interface marge.xml)







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

ament_package()
