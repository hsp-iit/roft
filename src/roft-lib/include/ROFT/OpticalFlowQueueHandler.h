/*
 * Copyright (C) 2022 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#ifndef ROFT_OPTICALFLOWQUEUEHANDLER_HPP
#define ROFT_OPTICALFLOWQUEUEHANDLER_HPP

#include <cstddef>
#include <deque>

#include <opencv2/opencv.hpp>

namespace ROFT {
     class OpticalFlowQueueHandler;
}


class ROFT::OpticalFlowQueueHandler
{
public:
    OpticalFlowQueueHandler(const std::size_t& window_size);

    void add_flow(const cv::Mat& frame, const double& time_stamp);

    std::vector<cv::Mat> get_buffer_region(const double& initial_time_stamp);

    void clear();

    struct Entry
    {
    public:
        cv::Mat frame;
        double timestamp;
    };

private:
    std::size_t window_size_;

    std::deque<ROFT::OpticalFlowQueueHandler::Entry> buffer_;
};

#endif /* ROFT_FLOWQUEUEHANDLER_HPP */
