/*
 * Copyright (C) 2022 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#include <ROFT/OpticalFlowQueueHandler.h>

using namespace ROFT;


OpticalFlowQueueHandler::OpticalFlowQueueHandler(const std::size_t& window_size) :
    window_size_(window_size)
{}


void OpticalFlowQueueHandler::add_flow(const cv::Mat& frame, const double& time_stamp)
{
    OpticalFlowQueueHandler::Entry entry{frame.clone(), time_stamp};
    buffer_.push_back(entry);

    /* Enforce the maximum size. */
    if (buffer_.size() > window_size_)
        buffer_.pop_front();
}


std::vector<cv::Mat> OpticalFlowQueueHandler::get_buffer_region(const double& initial_time_stamp)
{
    std::vector<cv::Mat> output_region;
    output_region.clear();

    std::size_t index;
    bool found = false;
    for (index = 0; index < buffer_.size(); index++)
        if (abs(buffer_[index].timestamp - initial_time_stamp) < 1e-3)
            found = true;

    /* Return empty buffer if not found. */
    if (!found)
        return output_region;

    /* The optical flow alway refer to the previous RGB image, this we need the next frame. */
    index++;

    for (; index < buffer_.size(); index++)
        // output_region.push_back(buffer_[index].frame);
        output_region.push_back(buffer_[index].frame.clone());

    return output_region;
}


void OpticalFlowQueueHandler::clear()
{
    buffer_.clear();
}
