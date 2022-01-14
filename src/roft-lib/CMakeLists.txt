#===============================================================================
#
# Copyright (C) 2022 Istituto Italiano di Tecnologia (IIT)
#
# This software may be modified and distributed under the terms of the
# GPL-2+ license. See the accompanying LICENSE file for details.
#
#===============================================================================

set(LIBRARY_TARGET_NAME RoftLib)

# CMakeRC
include(CMakeRC)

# BayesFilters
find_package(BayesFilters REQUIRED)

# Eigen
find_package(Eigen3 REQUIRED)

# OpenCV
find_package(OpenCV REQUIRED)

# RobotsIO
find_package(RobotsIO REQUIRED)

# SuperimposeMesh
find_package(SuperimposeMesh 0.11.100 REQUIRED)

# Header files
set(${LIBRARY_TARGET_NAME}_HDR
    include/ROFT/CameraMeasurement.h
    include/ROFT/CartesianQuaternionMeasurement.h
    include/ROFT/CartesianQuaternionModel.h
    include/ROFT/DatasetImageOpticalFlow.h
    include/ROFT/DatasetImageSegmentation.h
    include/ROFT/DatasetImageSegmentationDelayed.h
    include/ROFT/ImageOpticalFlowMeasurement.hpp
    include/ROFT/ImageOpticalFlowSource.h
    include/ROFT/ImageSegmentationMeasurement.h
    include/ROFT/ImageSegmentationOFAidedSource.hpp
    include/ROFT/ImageSegmentationSource.h
    include/ROFT/MeshResource.h
    include/ROFT/ModelParameters.h
    include/ROFT/OFAidedFilter.h
    include/ROFT/OpticalFlowUtilities.h
    include/ROFT/QuaternionModel.h
    include/ROFT/SpatialVelocityModel.h
    include/ROFT/SKFCorrection.h
)

# Source files
set(${LIBRARY_TARGET_NAME}_SRC
    src/CameraMeasurement.cpp
    src/CartesianQuaternionMeasurement.cpp
    src/CartesianQuaternionModel.cpp
    src/DatasetImageOpticalFlow.cpp
    src/DatasetImageSegmentation.cpp
    src/DatasetImageSegmentationDelayed.cpp
    src/ImageOpticalFlowSource.cpp
    src/ImageSegmentationMeasurement.cpp
    src/ImageSegmentationSource.cpp
    src/MeshResource.cpp
    src/ModelParameters.cpp
    src/OFAidedFilter.cpp
    src/OpticalFlowUtilities.cpp
    src/QuaternionModel.cpp
    src/SpatialVelocityModel.cpp
    src/SKFCorrection.cpp
)
# Resources
cmrc_add_resource_library(${LIBRARY_TARGET_NAME}_Resources
                          ALIAS ${LIBRARY_TARGET_NAME}::Resources
                          NAMESPACE roft_lib_resources
                          PREFIX __prc
                          meshes/DOPE/003_cracker_box.obj
                          meshes/DOPE/004_sugar_box.obj
                          meshes/DOPE/005_tomato_soup_can.obj
                          meshes/DOPE/006_mustard_bottle.obj
                          meshes/DOPE/009_gelatin_box.obj
                          meshes/DOPE/010_potted_meat_can.obj
                          meshes/DOPE/021_bleach_cleanser.obj
)

# Add library
add_library(${LIBRARY_TARGET_NAME} ${${LIBRARY_TARGET_NAME}_SRC} ${${LIBRARY_TARGET_NAME}_HDR})

# Library properties
set_target_properties(${LIBRARY_TARGET_NAME} PROPERTIES VERSION       ${${PROJECT_NAME}_VERSION}
                                                        PUBLIC_HEADER "${${LIBRARY_TARGET_NAME}_HDR}")

# Include directories
target_include_directories(${LIBRARY_TARGET_NAME} PUBLIC
                                                  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                  "$<INSTALL_INTERFACE:$<INSTALL_PREFIX>/${CMAKE_INSTALL_INCLUDEDIR}>"
)

# Linker configuration
target_link_libraries(${LIBRARY_TARGET_NAME}
                      PUBLIC
                      BayesFilters::BayesFilters
                      Eigen3::Eigen
                      RobotsIO::RobotsIO
                      SI::SuperimposeMesh
                      ${LIBRARY_TARGET_NAME}::Resources
                      ${MLPACK_LIBRARIES}
                      ${OpenCV_LIBS}
)

# Specify installation targets, typology and destination folders.
install(TARGETS ${LIBRARY_TARGET_NAME}
        EXPORT  ${PROJECT_NAME}
        LIBRARY       DESTINATION "${CMAKE_INSTALL_LIBDIR}"                                   COMPONENT shlib
        ARCHIVE       DESTINATION "${CMAKE_INSTALL_LIBDIR}"                                   COMPONENT lib
        RUNTIME       DESTINATION "${CMAKE_INSTALL_BINDIR}"                                   COMPONENT bin
        PUBLIC_HEADER DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/${LIBRARY_TARGET_NAME}"        COMPONENT dev
)

set_property(GLOBAL APPEND PROPERTY ${PROJECT_NAME}_TARGETS ${LIBRARY_TARGET_NAME})