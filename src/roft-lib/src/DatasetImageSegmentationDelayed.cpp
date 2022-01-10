/*
 * Copyright (C) 2022 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#include <ROFT/DatasetImageSegmentationDelayed.h>

#include <chrono>

using namespace ROFT;


DatasetImageSegmentationDelayed::DatasetImageSegmentationDelayed
(
    const float& fps,
    const float& simulated_fps,
    const bool simulate_inference_time,
    const std::string& dataset_path,
    const std::string& format,
    const std::size_t width,
    const std::size_t height,
    const std::string& segmentation_set,
    const ModelParameters& model_parameters,
    const std::size_t heading_zeros,
    const std::size_t index_offset
) :
    DatasetImageSegmentation(dataset_path, format, width, height, segmentation_set, model_parameters, heading_zeros, index_offset),
    fps_(fps),
    simulated_fps_(simulated_fps),
    simulate_inference_time_(simulate_inference_time),
    head_0_(head_ + 1),
    delay_(static_cast<int>(fps_ / simulated_fps_))
{}


DatasetImageSegmentationDelayed::~DatasetImageSegmentationDelayed()
{}


std::pair<bool, cv::Mat> DatasetImageSegmentationDelayed::segmentation(const bool& blocking)
{
    int index = head_;
    if (simulate_inference_time_)
        index -= delay_;

    if (((index - head_0_) % delay_) != 0)
        return std::make_pair(false, cv::Mat());

    if (index < 0)
        index = head_0_;

    auto t0 = std::chrono::steady_clock::now();

    auto output = read_file(index);

    auto t1 = std::chrono::steady_clock::now();

    data_loading_time_ = std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count();

    return output;
}


void DatasetImageSegmentationDelayed::reset_data_loading_time()
{
    data_loading_time_ = 0.0;
}


double DatasetImageSegmentationDelayed::get_data_loading_time() const
{
    return data_loading_time_;
}


int DatasetImageSegmentationDelayed::get_frames_between_iterations() const
{
    return int(fps_ / simulated_fps_);
}
