#
# Copyright (c) The acados authors.
#
# This file is part of acados.
#
# The 2-Clause BSD License
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
# this list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
# this list of conditions and the following disclaimer in the documentation
# and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.;
#



cmake_minimum_required(VERSION 3.13)

project(BlueROV_Heavy_sim)

# build options.

option(BUILD_SIM_EXAMPLE "Should the simulation example main_sim_BlueROV_Heavy_sim be build?" OFF)
option(BUILD_ACADOS_SIM_SOLVER_LIB "Should the simulation solver library acados_sim_solver_BlueROV_Heavy_sim be build?" OFF)


if(CMAKE_CXX_COMPILER_ID MATCHES "GNU" AND CMAKE_SYSTEM_NAME MATCHES "Windows")
    # MinGW: remove prefix and change suffix to match MSVC
    # (such that Matlab mex recognizes the libraries)
    set(CMAKE_SHARED_LIBRARY_PREFIX "")
    set(CMAKE_IMPORT_LIBRARY_SUFFIX ".lib")
    set(CMAKE_IMPORT_LIBRARY_PREFIX "")
    set(CMAKE_STATIC_LIBRARY_SUFFIX ".lib")
    set(CMAKE_STATIC_LIBRARY_PREFIX "")
endif()


if(CMAKE_CXX_COMPILER_ID MATCHES "MSVC")
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE /home/wjun/bluerov2_ws222/src/underwater_ros2_control/controllers/acados_nmpc_controller/matlab/c_generated_code)
    set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY_RELEASE /home/wjun/bluerov2_ws222/src/underwater_ros2_control/controllers/acados_nmpc_controller/matlab/c_generated_code)
    set(CMAKE_LIBRARY_OUTPUT_DIRECTORY_RELEASE /home/wjun/bluerov2_ws222/src/underwater_ros2_control/controllers/acados_nmpc_controller/matlab/c_generated_code)
endif()


# object target names
set(MODEL_OBJ model_BlueROV_Heavy_sim)
set(OCP_OBJ ocp_BlueROV_Heavy_sim)
set(SIM_OBJ sim_BlueROV_Heavy_sim)

# model
set(MODEL_SRC
    BlueROV_Heavy_sim_model/BlueROV_Heavy_sim_expl_ode_fun.c
    BlueROV_Heavy_sim_model/BlueROV_Heavy_sim_expl_vde_forw.c
    BlueROV_Heavy_sim_model/BlueROV_Heavy_sim_expl_vde_adj.c
)
add_library(${MODEL_OBJ} OBJECT ${MODEL_SRC} )



# for sim solver
if(${BUILD_ACADOS_SOLVER_LIB} OR ${BUILD_EXAMPLE}
    OR ${BUILD_SIM_EXAMPLE} OR ${BUILD_ACADOS_SIM_SOLVER_LIB})
    set(SIM_SRC acados_sim_solver_BlueROV_Heavy_sim.c)
    add_library(${SIM_OBJ} OBJECT ${SIM_SRC})
endif()
set(EXTERNAL_DIR)
set(EXTERNAL_LIB)

# set some search paths for preprocessor and linker
set(ACADOS_INCLUDE_PATH /home/wjun/acados/include CACHE PATH "Define the path which contains the include directory for acados.")
set(ACADOS_LIB_PATH /home/wjun/acados/lib CACHE PATH "Define the path which contains the lib directory for acados.")

# c-compiler flags for debugging
set(CMAKE_C_FLAGS_DEBUG "-O0 -ggdb")

set(CMAKE_C_FLAGS "-fPIC -std=c99   -O2  ")
#-fno-diagnostics-show-line-numbers -g

include_directories(
   ${ACADOS_INCLUDE_PATH}
   ${ACADOS_INCLUDE_PATH}/acados
   ${ACADOS_INCLUDE_PATH}/blasfeo/include
   ${ACADOS_INCLUDE_PATH}/hpipm/include
)

# linker flags
link_directories(${ACADOS_LIB_PATH})

# link to libraries
if(UNIX)
    link_libraries(acados hpipm blasfeo m )
else()
    link_libraries(acados hpipm blasfeo )
endif()

# the targets

# example_sim
if(${BUILD_SIM_EXAMPLE})
    set(EX_SIM_SRC main_sim_BlueROV_Heavy_sim.c)
    set(EX_SIM_EXE main_sim_BlueROV_Heavy_sim)
    add_executable(${EX_SIM_EXE} ${EX_SIM_SRC} $<TARGET_OBJECTS:${MODEL_OBJ}> $<TARGET_OBJECTS:${SIM_OBJ}>)
    install(TARGETS ${EX_SIM_EXE} DESTINATION ${CMAKE_INSTALL_PREFIX})
endif(${BUILD_SIM_EXAMPLE})

# sim_shared_lib
if(${BUILD_ACADOS_SIM_SOLVER_LIB})
    set(LIB_ACADOS_SIM_SOLVER acados_sim_solver_BlueROV_Heavy_sim)
    add_library(${LIB_ACADOS_SIM_SOLVER} SHARED $<TARGET_OBJECTS:${MODEL_OBJ}> $<TARGET_OBJECTS:${SIM_OBJ}>)
    install(TARGETS ${LIB_ACADOS_SIM_SOLVER} DESTINATION ${CMAKE_INSTALL_PREFIX})
endif(${BUILD_ACADOS_SIM_SOLVER_LIB})

# unset options for clean cmake cache on subsequent cmake runs
unset(BUILD_SIM_EXAMPLE CACHE)
unset(BUILD_ACADOS_SIM_SOLVER_LIB CACHE)
