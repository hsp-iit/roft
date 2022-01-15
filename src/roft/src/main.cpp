/*
 * Copyright (C) 2022 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#include <ROFT/CameraMeasurement.h>
#include <ROFT/DatasetImageSegmentation.h>
#include <ROFT/DatasetImageSegmentationDelayed.h>
#include <ROFT/DatasetImageOpticalFlow.h>
#ifdef HAS_NVOF
#include <ROFT/ImageOpticalFlowNVOF.h>
#endif
#include <ROFT/ImageOpticalFlowSource.h>
#include <ROFT/ImageSegmentationSource.h>
#include <ROFT/ModelParameters.h>
#include <ROFT/OFAidedFilter.h>

#include <RobotsIO/Camera/Camera.h>
#include <RobotsIO/Camera/DatasetCamera.h>
#include <RobotsIO/Camera/CameraParameters.h>
#include <RobotsIO/Utils/DatasetTransform.h>
#include <RobotsIO/Utils/DatasetTransformDelayed.h>
#include <RobotsIO/Utils/ImageFileProbe.h>
#include <RobotsIO/Utils/Parameters.h>

using namespace Eigen;
using namespace ROFT;
using namespace RobotsIO::Camera;
using namespace RobotsIO::Utils;


int main(int argc, char** argv)
{
    return EXIT_SUCCESS;
}
