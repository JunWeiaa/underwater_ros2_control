cmake_minimum_required(VERSION 3.8)
project(bluerov2_heavy)

find_package(ament_cmake REQUIRED)

install(
		DIRECTORY meshes launch config sdf worlds urdf matlab
		DESTINATION share/${PROJECT_NAME}/
)

ament_package()
