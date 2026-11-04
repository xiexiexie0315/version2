cmake_minimum_required(VERSION 3.8)
project(hnu2027_sentry_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
