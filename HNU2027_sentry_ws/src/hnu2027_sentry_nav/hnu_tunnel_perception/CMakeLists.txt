cmake_minimum_required(VERSION 3.8)
project(hnu_tunnel_perception)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(hnu_sentry_interfaces REQUIRED)
find_package(PCL REQUIRED COMPONENTS common)
find_package(pcl_conversions REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)

add_library(tunnel_geometry_detector src/tunnel_geometry_detector.cpp)
target_compile_features(tunnel_geometry_detector PUBLIC cxx_std_17)
target_include_directories(tunnel_geometry_detector PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_executable(tunnel_detector_node src/tunnel_detector_node.cpp)
target_compile_features(tunnel_detector_node PUBLIC cxx_std_17)
target_include_directories(tunnel_detector_node PRIVATE include)
target_link_libraries(tunnel_detector_node tunnel_geometry_detector ${PCL_LIBRARIES})
ament_target_dependencies(tunnel_detector_node
  geometry_msgs hnu_sentry_interfaces pcl_conversions rclcpp sensor_msgs tf2 tf2_ros tf2_sensor_msgs)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS tunnel_geometry_detector tunnel_detector_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_tunnel_geometry_detector test/test_tunnel_geometry_detector.cpp)
  target_link_libraries(test_tunnel_geometry_detector tunnel_geometry_detector)
endif()

ament_export_include_directories(include)
ament_package()
