cmake_minimum_required(VERSION 3.8)
project(hnu2027_sentry_simulation)

find_package(ament_cmake REQUIRED)

install(DIRECTORY resource config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_world_coordinates test/test_world_coordinates.py)
endif()

ament_package()
