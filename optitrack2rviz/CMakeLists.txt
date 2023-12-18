cmake_minimum_required(VERSION 3.8)
project(optitrack2rviz)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(pose_to_tf_STL src/pose_to_tf_STL.cpp)
add_executable(pose_to_tf_camera src/pose_to_tf_camera.cpp)

ament_target_dependencies(pose_to_tf_STL rclcpp std_msgs geometry_msgs tf2_ros)
ament_target_dependencies(pose_to_tf_camera rclcpp std_msgs geometry_msgs tf2_ros)

# Manually set Eigen include directories
set(Eigen3_INCLUDE_DIRS /usr/include/eigen3)  # Adjust the path based on your Eigen3 installation

# Use the manually set include directories
target_include_directories(pose_to_tf_STL PRIVATE ${Eigen3_INCLUDE_DIRS})
target_include_directories(pose_to_tf_camera PRIVATE ${Eigen3_INCLUDE_DIRS})

install(TARGETS
  pose_to_tf_STL
  pose_to_tf_camera
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
