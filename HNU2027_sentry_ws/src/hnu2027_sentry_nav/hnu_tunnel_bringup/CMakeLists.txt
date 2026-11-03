cmake_minimum_required(VERSION 3.8)
project(hnu_tunnel_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config map pcd DESTINATION share/${PROJECT_NAME})

ament_package()
