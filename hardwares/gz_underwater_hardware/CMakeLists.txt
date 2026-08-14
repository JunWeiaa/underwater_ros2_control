cmake_minimum_required(VERSION 3.8)
project(gz_underwater_hardware)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Default to C11
if (NOT CMAKE_C_STANDARD)
    set(CMAKE_C_STANDARD 11)
endif ()
# Default to C++17
if (NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif ()

# Compiler options
if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(controller_manager REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)

find_package(gz_sim_vendor REQUIRED)
find_package(gz-sim REQUIRED)

find_package(gz_plugin_vendor REQUIRED)
find_package(gz-plugin REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(include)

add_library(${PROJECT_NAME}-system SHARED
        src/gz_underwater_plugin.cpp
)

target_link_libraries(${PROJECT_NAME}-system
        gz-sim::gz-sim
        gz-plugin::register
)
ament_target_dependencies(${PROJECT_NAME}-system
        ament_index_cpp
        controller_manager
        hardware_interface
        pluginlib
        rclcpp
        yaml_cpp_vendor
        rclcpp_lifecycle
        Eigen3
)

#########

add_library(gz_underwater_plugin SHARED
        src/gz_system.cpp
)
ament_target_dependencies(gz_underwater_plugin
        rclcpp_lifecycle
        hardware_interface
        rclcpp
        Eigen3
)
target_link_libraries(gz_underwater_plugin
        gz-sim::gz-sim
)

## Install
install(TARGETS
gz_underwater_plugin
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

install(DIRECTORY
        include/
        DESTINATION include
)



# Testing and linting
if (BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
endif ()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}-system gz_underwater_plugin)

# Install directories
install(TARGETS ${PROJECT_NAME}-system
        DESTINATION lib
)

pluginlib_export_plugin_description_file(gz_underwater_hardware gz_underwater_hardware.xml)

# Setup the project
ament_package()
