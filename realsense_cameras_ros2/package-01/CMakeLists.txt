
cmake_minimum_required(VERSION 3.5)
project(package-01)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch
  worlds
  models
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
