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
        VERSION 1.0.0)

find_package(Eigen3 REQUIRED)

find_package(RobotsIO REQUIRED)

find_package(OpenCV REQUIRED)

# Support for NVIDIA Optical Flow within OpenCV is optional
if (OpenCV_opencv_cudaoptflow_FOUND)
  add_definitions(-DHAS_NVOF)
endif()

add_executable(ROFT-tracker src/main.cpp)

target_link_libraries(ROFT-tracker
                      PRIVATE
                      Eigen3::Eigen
                      RobotsIO::RobotsIO
                      ROFTLib
)

install(TARGETS ROFT-tracker DESTINATION bin)
