#===============================================================================
#
# Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT)
#
# This software may be modified and distributed under the terms of the
# GPL-2+ license. See the accompanying LICENSE file for details.
#
#===============================================================================

set(EXE_TARGET_NAME 6d-of-tracking-of-dumper)

find_package(OTL REQUIRED)

find_package(RobotsIO REQUIRED)

set(${EXE_TARGET_NAME}_HDR
)

set(${EXE_TARGET_NAME}_SRC
    src/main.cpp
)

add_executable(${EXE_TARGET_NAME} ${${EXE_TARGET_NAME}_HDR} ${${EXE_TARGET_NAME}_SRC})

target_include_directories(${EXE_TARGET_NAME}
                           PRIVATE
                           ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(${EXE_TARGET_NAME}
                      PRIVATE
                      RobotsIO::RobotsIO
                      OTL::OTL
)

install(TARGETS ${EXE_TARGET_NAME} DESTINATION bin)