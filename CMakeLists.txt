# Copyright (c) 2025 CNRS
# Author: Thanh Nguyen
#
# This file is part of agimus-spacelab.
# agimus-spacelab is free software: you can redistribute it
# and/or modify it under the terms of the GNU Lesser General Public
# License as published by the Free Software Foundation, either version
# 3 of the License, or (at your option) any later version.

cmake_minimum_required(VERSION 3.22)

set(PROJECT_NAME agimus_spacelab)
set(PROJECT_DESCRIPTION "Agimus Spacelab - Manipulation Planning Framework")
set(PROJECT_URL "https://gitlab.laas.fr/agimus-project/agimus_spacelab")

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

# Optional dependencies - can be disabled
option(WITH_CORBA "Build with CORBA support" ON)
option(WITH_PYHPP "Build with PyHPP support" ON)
option(BUILD_TESTING "Build tests" ON)

# Find required dependencies
find_package(pinocchio REQUIRED)
find_package(hpp-pinocchio REQUIRED)

# Optional CORBA dependencies
if(WITH_CORBA)
  find_package(hpp-manipulation-corba)
  find_package(hpp-gepetto-viewer)
  if(hpp-manipulation-corba_FOUND AND hpp-gepetto-viewer_FOUND)
    set(HAS_CORBA TRUE)
    add_compile_definitions(HAS_CORBA_SUPPORT)
  else()
    message(WARNING "CORBA dependencies not found. CORBA backend will be disabled.")
    set(HAS_CORBA FALSE)
  endif()
endif()

# Optional PyHPP dependencies
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
  DIRECTORY ${PROJECT_SOURCE_DIR}/src/agimus_spacelab
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

add_library(${PROJECT_NAME} INTERFACE)
install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${TARGETS_EXPORT_NAME}
  DESTINATION lib)
