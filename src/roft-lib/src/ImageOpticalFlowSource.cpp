/*
 * Copyright (C) 2022 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#include <ROFT/ImageOpticalFlowSource.h>

using namespace ROFT;


ImageOpticalFlowSource::~ImageOpticalFlowSource()
{}


bool ImageOpticalFlowSource::reset()
{
    return true;
}


bool ImageOpticalFlowSource::step_frame()
{
    return true;
}


double ImageOpticalFlowSource::get_data_loading_time() const
{
    return 0.0;
}
