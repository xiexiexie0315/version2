cmake_minimum_required(VERSION 3.8)
project(hnu2027_sentry_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf config resource
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_original_model test/test_original_model.py)
endif()

ament_package()
