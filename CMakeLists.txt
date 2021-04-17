#===============================================================================
#
# Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT)
#
# This software may be modified and distributed under the terms of the
# GPL-2+ license. See the accompanying LICENSE file for details.
#
#===============================================================================

cmake_minimum_required(VERSION 3.13)

project(6d-of-tracking
        LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 11)

include(GNUInstallDirs)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_BINDIR}")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR}")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR}")

# Default build type to Release
if(NOT CMAKE_CONFIGURATION_TYPES)
  if(NOT CMAKE_BUILD_TYPE)
    message(STATUS "Setting build type to 'Release' as none was specified.")
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY VALUE "Release")
  endif()
endif()

add_subdirectory(tools)
