/*
 * Copyright (C) 2022 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#include <ROFT/DatasetImageSegmentation.h>

#include <opencv2/opencv.hpp>

#include <chrono>
#include <iostream>
#include <iomanip>
#include <sstream>

using namespace ROFT;
using namespace cv;


DatasetImageSegmentation::DatasetImageSegmentation
(
    const std::string& dataset_path,
    const std::string& format,
    const std::size_t width,
    const std::size_t height,
    const std::string& segmentation_set,
    const ModelParameters& model_parameters,
    const std::size_t heading_zeros,
    const std::size_t index_offset,
    const bool simulate_missing_detections
) :
    format_(format),
    width_(width),
    height_(height),
    object_name_(model_parameters.name()),
    head_(-1 + index_offset),
    index_offset_(index_offset),
    heading_zeros_(heading_zeros),
    simulate_missing_detections_(simulate_missing_detections)
{
    /* Compose dataset path. */
    std::string root = dataset_path;
    if (root.back() != '/')
        root += '/';
    dataset_path_ = root + "masks/" + segmentation_set + "/";

    /* Test. */
    // set_frame(0 + index_offset_);
    // bool valid_dataset = false;
    // std::tie(valid_dataset, std::ignore) = segmentation(false);
    // if (!valid_dataset)
    //     throw(std::runtime_error(log_name_ + "::ctor. Cannot load dataset."));
    // else
    //     std::cout << log_name_ + "::ctor. Dataset loaded succesfully." << std::endl;
    // set_frame(-1 + index_offset_);
}


DatasetImageSegmentation::~DatasetImageSegmentation()
{}


bool DatasetImageSegmentation::reset()
{
    head_ = -1 + index_offset_;

    return true;
}


bool DatasetImageSegmentation::step_frame()
{
    head_++;

    auto t0 = std::chrono::steady_clock::now();

    output_ = read_file(head_);

    auto t1 = std::chrono::steady_clock::now();

    data_loading_time_ = std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count();

    return true;
}


// bool DatasetImageSegmentation::set_frame(const std::size_t& index)
// {
//     head_ = index;

//     return true;
// }


bool DatasetImageSegmentation::is_stepping_required() const
{
    return true;
}


void DatasetImageSegmentation::reset_data_loading_time()
{
    data_loading_time_ = 0.0;
}


double DatasetImageSegmentation::get_data_loading_time() const
{
    return data_loading_time_;
}


std::pair<bool, cv::Mat> DatasetImageSegmentation::segmentation(const bool& blocking)
{
    return output_;
}


std::string DatasetImageSegmentation::compose_file_name(const int& index, const std::size_t& number_of_digits)
{
    std::ostringstream ss;
    ss << std::setw(number_of_digits) << std::setfill('0') << index;
    return ss.str();
}


std::pair<bool, cv::Mat> DatasetImageSegmentation::read_file(const std::size_t& frame_index)
{
    std::string file_name = dataset_path_ + object_name_ + "_" + compose_file_name(frame_index, heading_zeros_) + "." + format_;
    cv::Mat segmentation = cv::imread(file_name, cv::IMREAD_UNCHANGED);
    segmentation.convertTo(segmentation, CV_8UC1);

    if (segmentation.data == nullptr)
    {
        if (simulate_missing_detections_)
            segmentation = cv::Mat(cv::Size(width_, height_), CV_8UC1, cv::Scalar(0.0));
        else
        {
            std::cout << log_name_ << "::segmentation. Error: cannot load segmentation data for frame" + file_name << std::endl;

            return std::make_pair(false, cv::Mat());
        }
    }

    return std::make_pair(true, segmentation);
}
