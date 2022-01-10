/*
 * Copyright (C) 2022 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#include <ROFT/ImageSegmentationSource.h>

using namespace ROFT;


ImageSegmentationSource::~ImageSegmentationSource()
{}


bool ImageSegmentationSource::reset()
{
    return true;
}


bool ImageSegmentationSource::step_frame()
{
    return true;
}


void ImageSegmentationSource::reset_data_loading_time()
{
}


double ImageSegmentationSource::get_data_loading_time() const
{
    return 0.0;
}


int ImageSegmentationSource::get_frames_between_iterations() const
{
    /* 1 indicates that the segmentation is, by default, available at all frames. */
    return 1;
}
