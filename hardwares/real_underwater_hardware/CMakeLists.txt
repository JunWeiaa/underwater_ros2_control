cmake_minimum_required(VERSION 3.8)
project(real_underwater_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  mavros_msgs
  realtime_tools
  sensor_msgs
)


# find_package(libi2c-dev REQUIRED)


# find dependencies
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

## COMPILE
add_library(
  real_underwater_hardware
  SHARED
  src/real_system.cpp
  src/servo_controller.cpp
)


target_compile_features(real_underwater_hardware PUBLIC cxx_std_17)
target_include_directories(real_underwater_hardware PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include/real_underwater_hardware>
)
# target_link_libraries(real_underwater_hardware PUBLIC i2c)
ament_target_dependencies(
  real_underwater_hardware PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)
pluginlib_export_plugin_description_file(hardware_interface real_underwater_hardware.xml)
install(TARGETS real_underwater_hardware
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include/real_underwater_hardware
)

# 测试
if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   set(AMENT_LINT_AUTO_EXCLUDE)
#   ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(controller_manager REQUIRED)

  ament_add_gtest(test_load_real_system test/test_load_real_system.cpp)
  ament_target_dependencies(test_load_real_system
    controller_manager
    hardware_interface
    pluginlib
    rclcpp
    rclcpp_lifecycle
    mavros_msgs
    realtime_tools
    sensor_msgs
  )

  # ament_add_gmock(test_real_system_gmock test/test_real_system_gmock.cpp)
  # ament_target_dependencies(test_real_system_gmock
  #   hardware_interface
  #   pluginlib
  #   rclcpp
  #   rclcpp_lifecycle
  # )
  # target_compile_definitions(test_real_system_gmock PRIVATE "TESTING")
endif()

# 导出库
ament_export_include_directories(include)
ament_export_libraries(real_underwater_hardware)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

ament_package()
