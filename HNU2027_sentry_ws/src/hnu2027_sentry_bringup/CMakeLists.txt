cmake_minimum_required(VERSION 3.8)
project(hnu2027_sentry_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/validate_field_report.py DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_field_report test/test_field_report.py)
endif()

ament_package()
