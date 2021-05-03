#===============================================================================
#
# Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT)
#
# This software may be modified and distributed under the terms of the
# GPL-2+ license. See the accompanying LICENSE file for details.
#
#===============================================================================

set(LIBRARY_NAME object_renderer)

find_package(Python COMPONENTS Interpreter Development)

find_package(pybind11 CONFIG)

find_package(OpenCV REQUIRED)

find_package(SuperimposeMesh 0.11.100 REQUIRED)

set(${LIBRARY_NAME}_SRC src/renderer.cpp)

pybind11_add_module(${LIBRARY_NAME} ${${LIBRARY_NAME}_SRC})
target_link_libraries(${LIBRARY_NAME} PRIVATE SI::SuperimposeMesh ${OpenCV_LIBS})
