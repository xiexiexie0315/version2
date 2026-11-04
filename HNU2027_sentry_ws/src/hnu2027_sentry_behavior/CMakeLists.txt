cmake_minimum_required(VERSION 3.8)
project(hnu2027_sentry_behavior)

find_package(ament_cmake REQUIRED)
find_package(behaviortree_ros2 REQUIRED)
find_package(hnu_sentry_interfaces REQUIRED)
find_package(rclcpp_action REQUIRED)

add_library(traverse_tunnel_bt_plugin SHARED plugins/action/traverse_tunnel_action.cpp)
target_compile_features(traverse_tunnel_bt_plugin PUBLIC cxx_std_17)
target_compile_options(traverse_tunnel_bt_plugin PRIVATE -Wall -Wextra -Werror)
target_include_directories(traverse_tunnel_bt_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(traverse_tunnel_bt_plugin
  behaviortree_ros2 hnu_sentry_interfaces rclcpp_action)

install(TARGETS traverse_tunnel_bt_plugin
  LIBRARY DESTINATION share/${PROJECT_NAME}/bt_plugins
  ARCHIVE DESTINATION share/${PROJECT_NAME}/bt_plugins
  RUNTIME DESTINATION share/${PROJECT_NAME}/bt_plugins
)
install(DIRECTORY include/ behavior_trees config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_scenario_reports test/test_scenario_reports.py)
endif()

ament_package()
