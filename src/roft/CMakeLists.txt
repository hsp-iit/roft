#===============================================================================
#
# Copyright (C) 2022 Istituto Italiano di Tecnologia (IIT)
#
# This software may be modified and distributed under the terms of the
# GPL-2+ license. See the accompanying LICENSE file for details.
#
#===============================================================================

project(ROFT
        LANGUAGES CXX
        VERSION 1.2.1)

set(CMAKE_CXX_STANDARD 14)

# BayesFilters
find_package(BayesFilters REQUIRED)

# Eigen 3
find_package(Eigen3 REQUIRED)

# RobotsIO
find_package(RobotsIO REQUIRED)

# OpenCV
find_package(OpenCV REQUIRED)

# PkgConfig
find_package(PkgConfig REQUIRED)

# libconfig
pkg_check_modules(LIBCONFIG++ REQUIRED IMPORTED_TARGET libconfig++)

# tclap
pkg_check_modules(TCLAP REQUIRED tclap)

# Support for NVIDIA Optical Flow within OpenCV is optional
if (OpenCV_opencv_cudaoptflow_FOUND)
  add_definitions(-DHAS_NVOF)
endif()

add_executable(ROFT-tracker include/ConfigParser.h src/ConfigParser.cpp src/main.cpp)

target_include_directories(ROFT-tracker
                           PRIVATE
                           ${CMAKE_CURRENT_SOURCE_DIR}/include
                           ${TCLAP_INCLUDEDIR}
)

target_link_libraries(ROFT-tracker
                      PRIVATE
                      BayesFilters::BayesFilters
                      Eigen3::Eigen
                      PkgConfig::LIBCONFIG++
                      RobotsIO::RobotsIO
                      ROFT
)

install(TARGETS ROFT-tracker DESTINATION bin)
