# Copyright 2025 Enactic, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

cmake_minimum_required(VERSION 3.22)
project(openarm_can VERSION 1.2.2)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Testing support
include(CTest)

include(GNUInstallDirs)

# Create the main library
add_library(
  openarm_can
  src/openarm/can/socket/arm_component.cpp
  src/openarm/can/socket/gripper_component.cpp
  src/openarm/can/socket/openarm.cpp
  src/openarm/canbus/can_device_collection.cpp
  src/openarm/canbus/can_socket.cpp
  src/openarm/damiao_motor/dm_motor.cpp
  src/openarm/damiao_motor/dm_motor_control.cpp
  src/openarm/damiao_motor/dm_motor_device.cpp
  src/openarm/damiao_motor/dm_motor_device_collection.cpp)
set_target_properties(
  openarm_can
  PROPERTIES POSITION_INDEPENDENT_CODE ON
             VERSION ${PROJECT_VERSION}
             SOVERSION ${PROJECT_VERSION_MAJOR})
if(CMAKE_VERSION VERSION_GREATER_EQUAL 3.23)
  target_sources(
    openarm_can
    PUBLIC FILE_SET
           HEADERS
           TYPE
           HEADERS
           BASE_DIRS
           include
           FILES
           include/openarm/can/socket/arm_component.hpp
           include/openarm/can/socket/gripper_component.hpp
           include/openarm/can/socket/openarm.hpp
           include/openarm/canbus/can_device.hpp
           include/openarm/canbus/can_device_collection.hpp
           include/openarm/canbus/can_socket.hpp
           include/openarm/damiao_motor/dm_motor.hpp
           include/openarm/damiao_motor/dm_motor_constants.hpp
           include/openarm/damiao_motor/dm_motor_control.hpp
           include/openarm/damiao_motor/dm_motor_device.hpp
           include/openarm/damiao_motor/dm_motor_device_collection.hpp)
  install(
    TARGETS openarm_can
    EXPORT openarm_can_export
    FILE_SET HEADERS)
else()
  target_include_directories(
    openarm_can PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                       $<INSTALL_INTERFACE:include>)
  install(TARGETS openarm_can EXPORT openarm_can_export)
  install(DIRECTORY include/openarm TYPE INCLUDE)
endif()

# CMake package
set(INSTALL_CMAKE_DIR ${CMAKE_INSTALL_LIBDIR}/cmake/OpenArmCAN)
install(
  EXPORT openarm_can_export
  DESTINATION ${INSTALL_CMAKE_DIR}
  NAMESPACE OpenArmCAN::
  FILE OpenArmCANTargets.cmake)
include(CMakePackageConfigHelpers)
configure_package_config_file(
  OpenArmCANConfig.cmake.in ${CMAKE_CURRENT_BINARY_DIR}/OpenArmCANConfig.cmake
  INSTALL_DESTINATION ${INSTALL_CMAKE_DIR})
write_basic_package_version_file(
  ${CMAKE_CURRENT_BINARY_DIR}/OpenArmCANConfigVersion.cmake
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY SameMajorVersion)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/OpenArmCANConfig.cmake
              ${CMAKE_CURRENT_BINARY_DIR}/OpenArmCANConfigVersion.cmake
        DESTINATION ${INSTALL_CMAKE_DIR})

# pkg-config package
if(IS_ABSOLUTE "${CMAKE_INSTALL_INCLUDEDIR}")
  set(PKG_CONFIG_INCLUDEDIR "${CMAKE_INSTALL_INCLUDEDIR}")
else()
  set(PKG_CONFIG_INCLUDEDIR "\${prefix}/${CMAKE_INSTALL_INCLUDEDIR}")
endif()
if(IS_ABSOLUTE "${CMAKE_INSTALL_LIBDIR}")
  set(PKG_CONFIG_LIBDIR "${CMAKE_INSTALL_LIBDIR}")
else()
  set(PKG_CONFIG_LIBDIR "\${prefix}/${CMAKE_INSTALL_LIBDIR}")
endif()
configure_file(openarm-can.pc.in ${CMAKE_CURRENT_BINARY_DIR}/openarm-can.pc
               @ONLY)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/openarm-can.pc
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/pkgconfig)

add_executable(openarm-can-motor-check setup/motor_check.cpp)
target_link_libraries(openarm-can-motor-check openarm_can)
install(TARGETS openarm-can-motor-check DESTINATION ${CMAKE_INSTALL_BINDIR})

add_executable(openarm-can-diagnosis setup/openarm_can_diagnosis.cpp)
target_link_libraries(openarm-can-diagnosis openarm_can)
install(TARGETS openarm-can-diagnosis DESTINATION ${CMAKE_INSTALL_BINDIR})

# Add motor control example executable
add_executable(openarm-can-demo examples/demo.cpp)
target_link_libraries(openarm-can-demo openarm_can)
install(TARGETS openarm-can-demo DESTINATION ${CMAKE_INSTALL_BINDIR})

# Install scripts
install(
  PROGRAMS setup/openarm-can-change-baudrate
           setup/openarm-can-configure-socketcan
           setup/openarm-can-configure-socketcan-4-arms
           setup/openarm-can-set-zero
           setup/openarm-can-zero-position-calibration
  DESTINATION ${CMAKE_INSTALL_BINDIR})

# Add tests
if(BUILD_TESTING)
  # add_subdirectory(test)
endif()
