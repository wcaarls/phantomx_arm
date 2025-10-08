cmake_minimum_required(VERSION 3.8)
project(phantomx_arm_description)

# find dependencies
find_package(ament_cmake REQUIRED)

install(
  DIRECTORY
    meshes
    urdf
  DESTINATION
    share/${PROJECT_NAME}
)

ament_package()
