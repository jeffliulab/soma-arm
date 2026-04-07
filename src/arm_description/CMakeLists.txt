cmake_minimum_required(VERSION 3.8)
project(arm_description)

find_package(ament_cmake REQUIRED)

# Install URDF, meshes, worlds, launch, config
install(DIRECTORY
  urdf meshes worlds launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
