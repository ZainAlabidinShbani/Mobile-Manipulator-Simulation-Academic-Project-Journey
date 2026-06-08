cmake_minimum_required(VERSION 3.5)
project(assembly_of_robot)

find_package(ament_cmake REQUIRED)

# Install all resource directories into the package share folder.
# 'textures' is intentionally excluded — not produced by SolidWorks exporter.
install(
  DIRECTORY config launch meshes rviz urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
