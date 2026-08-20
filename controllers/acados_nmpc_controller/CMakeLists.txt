cmake_minimum_required(VERSION 3.8)
project(acados_nmpc_controller)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CONTROLLER_INCLUDE_DEPENDS
  pluginlib
  rcpputils
  controller_interface
  nav_msgs
  hardware_interface
  realtime_tools
  Eigen3
  tf2_ros
  Boost
  std_msgs
  geometry_msgs
  sensor_msgs
  std_srvs
  rclcpp
  rclcpp_lifecycle
  ament_index_cpp
)

find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${CONTROLLER_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()
find_package(OpenMP QUIET)

set(ACADOS_INSTALL_DIR "$ENV{ACADOS_INSTALL_DIR}")
if("${ACADOS_INSTALL_DIR}" STREQUAL "")
  message(FATAL_ERROR "The environment variable ACADOS_INSTALL_DIR is not set. Please set it to the acados install/source directory.")
endif()

set(acados_include "${ACADOS_INSTALL_DIR}/include")
set(acados_lib "${ACADOS_INSTALL_DIR}/lib")
set(acados_libraries
  "${acados_lib}/libacados.so"
  "${acados_lib}/libhpipm.so"
  "${acados_lib}/libblasfeo.so"
  m
)

function(read_solver_yaml_field metadata_file field_name output_var)
  file(STRINGS "${metadata_file}" field_lines REGEX "^[ \t]*${field_name}[ \t]*:")
  if(NOT field_lines)
    message(FATAL_ERROR "Missing '${field_name}' in solver metadata: ${metadata_file}")
  endif()
  list(GET field_lines 0 field_line)
  string(REGEX REPLACE "^[ \t]*${field_name}[ \t]*:[ \t]*\"?([^\"# \t]+)\"?.*$" "\\1" field_value "${field_line}")
  set(${output_var} "${field_value}" PARENT_SCOPE)
endfunction()

function(add_generated_solver metadata_file solver_targets_var)
  get_filename_component(solver_dir "${metadata_file}" DIRECTORY)
  read_solver_yaml_field("${metadata_file}" "solver_id" SOLVER_ID)
  read_solver_yaml_field("${metadata_file}" "model_name" MODEL_NAME)
  read_solver_yaml_field("${metadata_file}" "macro_prefix" MACRO_PREFIX)
  read_solver_yaml_field("${metadata_file}" "has_p_global" HAS_P_GLOBAL_VALUE)

  string(TOLOWER "${HAS_P_GLOBAL_VALUE}" HAS_P_GLOBAL_LOWER)
  if(HAS_P_GLOBAL_LOWER STREQUAL "true" OR HAS_P_GLOBAL_LOWER STREQUAL "yes" OR HAS_P_GLOBAL_LOWER STREQUAL "1")
    set(HAS_P_GLOBAL 1)
  else()
    set(HAS_P_GLOBAL 0)
  endif()

  string(MAKE_C_IDENTIFIER "${SOLVER_ID}" solver_target_prefix)
  set(solver_target "${solver_target_prefix}_solver")
  set(generated_dir "${solver_dir}/c_generated_code")
  if(NOT EXISTS "${generated_dir}/acados_solver_${MODEL_NAME}.h")
    message(FATAL_ERROR "Solver '${SOLVER_ID}' is missing ${generated_dir}/acados_solver_${MODEL_NAME}.h")
  endif()

  file(GLOB_RECURSE solver_c_sources
    CONFIGURE_DEPENDS
    FOLLOW_SYMLINKS
    "${generated_dir}/*.c"
  )
  set(ocp_sources)
  foreach(source_file IN LISTS solver_c_sources)
    file(RELATIVE_PATH rel_source "${generated_dir}" "${source_file}")
    if(rel_source MATCHES "^main")
      continue()
    endif()
    if(rel_source MATCHES "^acados_mex_")
      continue()
    endif()
    if(rel_source MATCHES "^acados_sim_")
      continue()
    endif()
    if(rel_source MATCHES "sfunction")
      continue()
    endif()
    if(rel_source MATCHES "_sim_model/")
      continue()
    endif()
    list(APPEND ocp_sources "${source_file}")
  endforeach()

  if(NOT ocp_sources)
    message(FATAL_ERROR "Solver '${SOLVER_ID}' has no OCP C sources under ${generated_dir}")
  endif()

  set(shim_file "${CMAKE_CURRENT_BINARY_DIR}/generated_solver_shims/${SOLVER_ID}_solver_shim.c")
  configure_file("${CMAKE_CURRENT_SOURCE_DIR}/cmake/acados_solver_shim.c.in" "${shim_file}" @ONLY)

  add_library(${solver_target} SHARED ${ocp_sources} "${shim_file}")
  set_target_properties(${solver_target}
    PROPERTIES
      OUTPUT_NAME "${SOLVER_ID}_solver"
      LIBRARY_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/solvers"
  )
  target_compile_features(${solver_target} PUBLIC c_std_99)
  target_include_directories(${solver_target}
    PUBLIC
      "${generated_dir}"
  )
  target_include_directories(${solver_target}
    SYSTEM PUBLIC
      "${acados_include}"
      "${acados_include}/blasfeo/include"
      "${acados_include}/hpipm/include"
      "${acados_include}/acados"
  )
  target_link_libraries(${solver_target} PUBLIC ${acados_libraries})
  if(OpenMP_C_FOUND)
    target_link_libraries(${solver_target} PUBLIC OpenMP::OpenMP_C)
  endif()
  if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(${solver_target} PRIVATE
      -Wno-unused-parameter
      -Wno-unused-variable
      $<$<COMPILE_LANGUAGE:C>:-Wno-strict-prototypes>
    )
  endif()
  if(CMAKE_C_COMPILER_ID STREQUAL "GNU")
    target_compile_options(${solver_target} PRIVATE
      $<$<COMPILE_LANGUAGE:C>:-Wno-alloc-size-larger-than>
    )
  endif()

  install(
    TARGETS ${solver_target}
    ARCHIVE DESTINATION lib/${PROJECT_NAME}/solvers
    LIBRARY DESTINATION lib/${PROJECT_NAME}/solvers
    RUNTIME DESTINATION bin
  )

  message(STATUS "Configured acados solver '${SOLVER_ID}' from ${generated_dir}")
  set(${solver_targets_var} ${${solver_targets_var}} ${solver_target} PARENT_SCOPE)
endfunction()

file(GLOB SOLVER_METADATA CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/solvers/*/solver.yaml")
if(NOT SOLVER_METADATA)
  message(FATAL_ERROR "No generated acados solvers found. Expected solvers/<solver_id>/solver.yaml")
endif()

set(GENERATED_SOLVER_TARGETS)
foreach(metadata_file IN LISTS SOLVER_METADATA)
  add_generated_solver("${metadata_file}" GENERATED_SOLVER_TARGETS)
endforeach()

add_library(acados_nmpc_controller SHARED
  src/acados_nmpc_controller.cpp
  src/FSM/StateAuto.cpp
  src/FSM/StateNoOutput.cpp
  src/FSM/StateManual.cpp
  src/estimator/FromOdomTopic.cpp
  src/estimator/GroundTruth.cpp
  src/estimator/StateEstimateBase.cpp
  src/control/ControlOutputMapper.cpp
  src/control/CtrlComponent.cpp
  src/control/TargetInfoLoader.cpp
  src/control/TrajectoryGenerator.cpp
  src/control/BsplineTrajectorySource.cpp
  src/control/TargetManager.cpp
)

target_include_directories(acados_nmpc_controller
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_include_directories(acados_nmpc_controller
  SYSTEM PUBLIC
    ${EIGEN3_INCLUDE_DIR}
    ${Boost_INCLUDE_DIRS}
    "${acados_include}"
    "${acados_include}/blasfeo/include"
    "${acados_include}/hpipm/include"
    "${acados_include}/acados"
)
ament_target_dependencies(acados_nmpc_controller PUBLIC ${CONTROLLER_INCLUDE_DEPENDS})
target_link_libraries(acados_nmpc_controller PUBLIC ${acados_libraries})
if(OpenMP_CXX_FOUND)
  target_link_libraries(acados_nmpc_controller PUBLIC OpenMP::OpenMP_CXX)
endif()

pluginlib_export_plugin_description_file(controller_interface acados_nmpc_controller.xml)

install(
  TARGETS acados_nmpc_controller
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib/${PROJECT_NAME}
  RUNTIME DESTINATION bin
)
install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
  PATTERN "acados_NMPC" EXCLUDE
)
foreach(metadata_file IN LISTS SOLVER_METADATA)
  get_filename_component(solver_dir "${metadata_file}" DIRECTORY)
  get_filename_component(solver_id "${solver_dir}" NAME)
  install(
    FILES "${metadata_file}"
    DESTINATION share/${PROJECT_NAME}/solvers/${solver_id}
  )
endforeach()

ament_export_dependencies(${CONTROLLER_INCLUDE_DEPENDS})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
