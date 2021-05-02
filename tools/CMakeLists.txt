#===============================================================================
#
# Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT)
#
# This software may be modified and distributed under the terms of the
# GPL-2+ license. See the accompanying LICENSE file for details.
#
#===============================================================================

option(BUILD_NVOF "Build NVIDIA NVOF-based optical flow dumper" OFF)

if (BUILD_NVOF)
   add_subdirectory(nvof)
endif()

add_subdirectory(object_renderer)
