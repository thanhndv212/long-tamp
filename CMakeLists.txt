# Copyright (c) 2025 CNRS
# Author: Thanh Nguyen
#
# This file is part of long-tamp.
# long-tamp is free software: you can redistribute it
# and/or modify it under the terms of the GNU Lesser General Public
# License as published by the Free Software Foundation, either version
# 3 of the License, or (at your option) any later version.

cmake_minimum_required(VERSION 3.22)

set(PROJECT_NAME long_tamp)
set(PROJECT_DESCRIPTION "Long-TAMP - Manipulation Planning Framework")
set(PROJECT_URL "https://github.com/thanhndv212/long-tamp")

set(PROJECT_USE_CMAKE_EXPORT TRUE)

# Check if the submodule cmake have been initialized
set(JRL_CMAKE_MODULES "${CMAKE_CURRENT_LIST_DIR}/cmake")
if(NOT EXISTS "${JRL_CMAKE_MODULES}/base.cmake")
  message(STATUS "JRL cmakemodules not found. Trying to find system package.")
  find_package(jrl-cmakemodules REQUIRED CONFIG)
  get_property(
    JRL_CMAKE_MODULES
    TARGET jrl-cmakemodules::jrl-cmakemodules
    PROPERTY INTERFACE_INCLUDE_DIRECTORIES)
endif()

include("${JRL_CMAKE_MODULES}/hpp.cmake")
include("${JRL_CMAKE_MODULES}/python.cmake")

compute_project_args(PROJECT_ARGS LANGUAGES CXX)
project(${PROJECT_NAME} ${PROJECT_ARGS})

findpython()

# Backend / feature options.
#   PyHPP is the only backend (ON by default).
#   TOPPRA is an optional time-parameterization optimizer (OFF by default).
option(WITH_PYHPP "Build with PyHPP support (default backend)" ON)
option(WITH_TOPPRA "Build with TOPPRA time-parameterization support" OFF)
option(BUILD_TESTING "Build tests" ON)
option(BUILD_BEHAVIORTREE_EXAMPLES "Build ROS-free BehaviorTree.CPP examples" OFF)

# Find required dependencies
find_package(pinocchio REQUIRED)
find_package(hpp-pinocchio REQUIRED)

# hpp-gepetto-viewer provides the viewer bindings: `hpp.gepetto` (Qt) and
# `pyhpp_viser` (browser). Look it up whenever PyHPP is enabled.
if(WITH_PYHPP)
  find_package(hpp-gepetto-viewer)
  if(hpp-gepetto-viewer_FOUND)
    add_compile_definitions(HAS_GEPETTO_VIEWER_SUPPORT)
  else()
    message(WARNING "hpp-gepetto-viewer not found. Viser/Gepetto viewers will be disabled.")
  endif()
endif()

# Optional PyHPP dependencies (default backend)
if(WITH_PYHPP)
  find_package(hpp-python)
  if(hpp-python_FOUND)
    set(HAS_PYHPP TRUE)
    add_compile_definitions(HAS_PYHPP_SUPPORT)
  else()
    message(WARNING "PyHPP dependencies not found. PyHPP backend will be disabled.")
    set(HAS_PYHPP FALSE)
  endif()
endif()

# Optional TOPPRA optimizer (pyhpp_toppra ships in hpp-toppra, needs toppra C++)
if(WITH_TOPPRA)
  find_package(hpp-toppra)
  if(hpp-toppra_FOUND)
    set(HAS_TOPPRA TRUE)
    add_compile_definitions(HAS_TOPPRA_SUPPORT)
  else()
    message(WARNING "hpp-toppra not found. TOPPRA optimizer will be disabled.")
    set(HAS_TOPPRA FALSE)
  endif()
endif()

# Install URDF/SRDF/mesh files (if they exist)
file(GLOB URDF_FILES "${CMAKE_CURRENT_SOURCE_DIR}/urdf/*.urdf")
file(GLOB SRDF_FILES "${CMAKE_CURRENT_SOURCE_DIR}/srdf/*.srdf")

if(URDF_FILES)
  install(
    FILES ${URDF_FILES}
    DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/${PROJECT_NAME}/urdf)
endif()

if(SRDF_FILES)
  install(
    FILES ${SRDF_FILES}
    DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/${PROJECT_NAME}/srdf)
endif()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/meshes")
  install(DIRECTORY meshes
          DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/${PROJECT_NAME})
endif()

# Install Python modules
install(
  DIRECTORY ${PROJECT_SOURCE_DIR}/src/long_tamp
  DESTINATION ${PYTHON_SITELIB}
  PATTERN "*.pyc" EXCLUDE
  PATTERN "__pycache__" EXCLUDE
)

# Install example scripts
install(
  DIRECTORY ${PROJECT_SOURCE_DIR}/script/
  DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/${PROJECT_NAME}/script
  USE_SOURCE_PERMISSIONS
  PATTERN "*.pyc" EXCLUDE
  PATTERN "__pycache__" EXCLUDE
)

# Testing
if(BUILD_TESTING)
  enable_testing()
  # Add test subdirectory if it has CMakeLists.txt
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tests/CMakeLists.txt")
    add_subdirectory(tests)
  endif()
endif()

if(BUILD_BEHAVIORTREE_EXAMPLES)
  add_subdirectory(examples/behaviortree)
endif()

add_library(${PROJECT_NAME} INTERFACE)
install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${TARGETS_EXPORT_NAME}
  DESTINATION lib)
