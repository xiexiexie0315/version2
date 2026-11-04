cmake_minimum_required(VERSION 3.8)
project(hnu_tunnel_controller)

find_package(ament_cmake REQUIRED)

add_library(fold_supervisor src/fold_supervisor.cpp)
target_compile_features(fold_supervisor PUBLIC cxx_std_17)
target_include_directories(fold_supervisor PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

install(TARGETS fold_supervisor EXPORT export_${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
ament_export_targets(export_${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_fold_supervisor test/test_fold_supervisor.cpp)
  target_link_libraries(test_fold_supervisor fold_supervisor)
endif()

ament_package()
