cmake_minimum_required(VERSION 3.8)
project(hnu_tunnel_core)

find_package(ament_cmake REQUIRED)

add_library(${PROJECT_NAME}
  src/clearance.cpp
  src/traverse_state_machine.cpp
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_clearance test/test_clearance.cpp)
  target_link_libraries(test_clearance ${PROJECT_NAME})
  ament_add_gtest(test_traverse_state_machine test/test_traverse_state_machine.cpp)
  target_link_libraries(test_traverse_state_machine ${PROJECT_NAME})
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_package()
