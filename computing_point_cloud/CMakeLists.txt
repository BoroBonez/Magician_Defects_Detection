cmake_minimum_required(VERSION 3.8)
project(computing_point_cloud)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)

# point_cloud_sub
add_executable(point_cloud_sub src/point_cloud_sub.cpp)
ament_target_dependencies(point_cloud_sub rclcpp std_msgs sensor_msgs pcl_conversions)

target_link_libraries(point_cloud_sub ${PCL_LIBRARIES}) 
target_include_directories(point_cloud_sub PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(point_cloud_sub PUBLIC cxx_std_17)

# pc_stl_relationship
add_executable(pc_stl_relationship src/pc_stl_relationship.cpp)
ament_target_dependencies(pc_stl_relationship rclcpp std_msgs sensor_msgs pcl_conversions)

target_link_libraries(pc_stl_relationship ${PCL_LIBRARIES}) 
target_include_directories(pc_stl_relationship PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(pc_stl_relationship PUBLIC cxx_std_17)

# stl_point_cloud
add_executable(stl_point_cloud src/stl_point_cloud.cpp)
ament_target_dependencies(stl_point_cloud rclcpp std_msgs sensor_msgs pcl_conversions)

target_link_libraries(stl_point_cloud ${PCL_LIBRARIES}) 
target_include_directories(stl_point_cloud PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(stl_point_cloud PUBLIC cxx_std_17)

#other
install(TARGETS point_cloud_sub pc_stl_relationship stl_point_cloud
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
