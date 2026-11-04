cmake_minimum_required(VERSION 3.8)
project(hnu_tunnel_controller)

find_package(ament_cmake REQUIRED)
find_package(hnu_tunnel_core REQUIRED)

add_library(fold_supervisor src/fold_supervisor.cpp)
target_compile_features(fold_supervisor PUBLIC cxx_std_17)
target_include_directories(fold_supervisor PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)

add_library(tunnel_control_core
  src/tunnel_cmd_mux.cpp
  src/tunnel_clearance_filter.cpp
  src/tunnel_traverse_manager.cpp
)
target_compile_features(tunnel_control_core PUBLIC cxx_std_17)
target_include_directories(tunnel_control_core PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
target_link_libraries(tunnel_control_core PUBLIC hnu_tunnel_core::hnu_tunnel_core)

install(TARGETS fold_supervisor tunnel_control_core EXPORT export_${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(hnu_tunnel_core)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_fold_supervisor test/test_fold_supervisor.cpp)
  target_link_libraries(test_fold_supervisor fold_supervisor)
  ament_add_gtest(test_tunnel_cmd_mux test/test_tunnel_cmd_mux.cpp)
  target_link_libraries(test_tunnel_cmd_mux tunnel_control_core)
  ament_add_gtest(test_tunnel_clearance_filter test/test_tunnel_clearance_filter.cpp)
  target_link_libraries(test_tunnel_clearance_filter tunnel_control_core)
  ament_add_gtest(test_tunnel_traverse_manager test/test_tunnel_traverse_manager.cpp)
  target_link_libraries(test_tunnel_traverse_manager tunnel_control_core)
endif()

ament_package()
