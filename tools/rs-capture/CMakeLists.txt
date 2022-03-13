#===============================================================================
#
# Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT)
#
# This software may be modified and distributed under the terms of the
# GPL-2+ license. See the accompanying LICENSE file for details.
#
#===============================================================================

set(EXE_TARGET_NAME ROFT-rs-capture)

find_package(OpenCV REQUIRED)

find_package(RobotsIO REQUIRED)

find_package(realsense2 REQUIRED)

set(${EXE_TARGET_NAME}_SRC
    src/main.cpp
)

add_executable(${EXE_TARGET_NAME} ${${EXE_TARGET_NAME}_SRC})

target_link_libraries(${EXE_TARGET_NAME}
                      PRIVATE
                      RobotsIO::RobotsIO
                      ${OpenCV_LIBS}
                      realsense2::realsense2
)
