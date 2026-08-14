cmake_minimum_required(VERSION 3.8)
project(subcat)

find_package(ament_cmake REQUIRED)

install(
		DIRECTORY meshes launch config worlds urdf matlab
		DESTINATION share/${PROJECT_NAME}/
)

ament_package()
