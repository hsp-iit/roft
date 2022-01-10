/*
 * Copyright (C) 2022 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#include <ROFT/DatasetImageOpticalFlow.h>
#include <ROFT/OpticalFlowUtilities.h>

#include <opencv2/core/hal/interface.h>
#include <opencv2/opencv.hpp>

#include <chrono>

#include <iomanip>
#include <sstream>

using namespace ROFT;
using namespace ROFT::OpticalFlowUtils;
using namespace cv;


DatasetImageOpticalFlow::DatasetImageOpticalFlow(const std::string& dataset_path, const std::string& set, const std::size_t width, const std::size_t height, const std::size_t& heading_zeros, const std::size_t& index_offset) :
    width_(width),
    height_(height),
    head_(-1 + index_offset),
    index_offset_(index_offset),
    heading_zeros_(heading_zeros)
{
    /* Compose dataset path. */
    std::string root = dataset_path;
    if (root.back() != '/')
        root += '/';
    dataset_path_ = root + "optical_flow/" + set + "/";

    /* Find parameters of this dataset. */
    bool valid = false;
    cv::Mat tmp;
    int counter = 0;
    while (!valid)
    {
        std::string file_name = dataset_path_ + compose_file_name(counter++, heading_zeros_) + ".float";
        std::tie(valid, tmp) = read_flow(file_name);
    }
    grid_size_ = width_ / tmp.cols;
    matrix_type_ = tmp.type();
    scaling_factor_ = 1;
    if (matrix_type_ == CV_16SC2)
        scaling_factor_ = float(1 << 5);

    /* Log loaded information. */
    std::string matrix_type_str = (matrix_type_ == CV_32FC2) ? std::string("CV_32FC2") : std::string("CV_16SC2");
    std::cout << log_name_ + "::ctor." << std::endl;
    std::cout << log_name_ + "   - grid size: " << grid_size_ << std::endl;
    std::cout << log_name_ + "   - scaling factor: " << scaling_factor_ << std::endl;
    std::cout << log_name_ + "   - matrix type: " << matrix_type_str << std::endl;
}


DatasetImageOpticalFlow::~DatasetImageOpticalFlow()
{}


bool DatasetImageOpticalFlow::reset()
{
    head_ = -1 + index_offset_;

    return true;
}


bool DatasetImageOpticalFlow::step_frame()
{
    head_++;

    std::string file_name = dataset_path_ + compose_file_name(head_, heading_zeros_) + ".float";

    auto t0 = std::chrono::steady_clock::now();

    output_ = read_flow(file_name);

    auto t1 = std::chrono::steady_clock::now();

    data_loading_time_ = std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count();

    return true;
}


// bool DatasetImageOpticalFlow::set_frame(const std::size_t& index)
// {
//     head_ = index;

//     return true;
// }


bool DatasetImageOpticalFlow::is_stepping_required() const
{
    return true;
}


std::size_t DatasetImageOpticalFlow::get_grid_size() const
{
    return grid_size_;
}


float DatasetImageOpticalFlow::get_scaling_factor() const
{
    return scaling_factor_;
}


int DatasetImageOpticalFlow::get_matrix_type() const
{
    return matrix_type_;
}


double DatasetImageOpticalFlow::get_data_loading_time() const
{
    return data_loading_time_;
}


std::tuple<bool, cv::Mat> DatasetImageOpticalFlow::flow(const bool& blocking)
{
    return output_;
}


std::string DatasetImageOpticalFlow::compose_file_name(const int& index, const std::size_t& number_of_digits)
{
    std::ostringstream ss;
    ss << std::setw(number_of_digits) << std::setfill('0') << index;
    return ss.str();
}
