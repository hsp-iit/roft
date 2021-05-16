#===============================================================================
#
# Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT)
#
# This software may be modified and distributed under the terms of the
# GPL-2+ license. See the accompanying LICENSE file for details.
#
#===============================================================================

option(BUILD_NVOF       "Build NVIDIA NVOF-based optical flow dumper" OFF)
option(BUILD_RENDERER   "Build OpenGL-based object renderer" OFF)
option(BUILD_RS_CAPTURE "Build realsense-based sequence recording tool" OFF)

if (BUILD_NVOF)
   add_subdirectory(nvof)
endif()

if (BUILD_RENDERER)
  add_subdirectory(object_renderer)
endif()

if (BUILD_RS_CAPTURE)
  add_subdirectory(rs-capture)
endif()
