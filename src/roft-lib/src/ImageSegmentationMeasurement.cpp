/*
 * Copyright (C) 2022 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#include <ROFT/ImageSegmentationMeasurement.h>

#include <Eigen/Dense>

using namespace Eigen;
using namespace RobotsIO::Utils;
using namespace ROFT;
using namespace bfl;
using namespace cv;


ImageSegmentationMeasurement::ImageSegmentationMeasurement(std::shared_ptr<Segmentation> segmentation_source, const std::size_t& width, const std::size_t& height) :
    segmentation_source_(segmentation_source),
    width_(width),
    height_(height)
{}


ImageSegmentationMeasurement::~ImageSegmentationMeasurement()
{}


bool ImageSegmentationMeasurement::freeze(const Data& data)
{
    /* Step frame in case of an offline segmentation source. */
    if (segmentation_source_->is_stepping_required())
        segmentation_source_->step_frame();

    cv::Mat segmentation;
    new_segmentation_ = false;
    std::tie(new_segmentation_, segmentation) = segmentation_source_->segmentation(false);

    if (new_segmentation_)
    {
        segmentation_available_ = true;
        segmentation_ = segmentation.clone();
        if (segmentation_.channels() == 3 || segmentation_.channels() == 4)
            cv::cvtColor(segmentation_, segmentation_, cv::COLOR_BGR2GRAY);
#if CV_MAJOR_VERSION >= 4
        cv::threshold(segmentation_, segmentation_, 1, 255, cv::THRESH_BINARY);
#else
        cv::threshold(segmentation_, segmentation_, 1, 255, CV_THRESH_BINARY);
#endif

        if ((width_ != 0) && (height_ != 0))
            cv::resize(segmentation_, segmentation_, cv::Size(width_, height_));
    }

    return segmentation_available_;
}


std::pair<bool, Data> ImageSegmentationMeasurement::measure(const Data& data) const
{
    return std::make_pair(segmentation_available_, std::make_pair(new_segmentation_, segmentation_));
}


std::pair<bool, Data> ImageSegmentationMeasurement::predictedMeasure(const Eigen::Ref<const Eigen::MatrixXd>& cur_states) const
{
    /* Not implemented. */
    throw(std::runtime_error(log_name_ + "::innovation. Not implemented."));
}


std::pair<bool, Data> ImageSegmentationMeasurement::innovation(const Data& predicted_measurements, const Data& measurements) const
{
    /* Not implemented. */
    throw(std::runtime_error(log_name_ + "::innovation. Not implemented."));
}


void ImageSegmentationMeasurement::reset()
{
    segmentation_available_ = false;
    segmentation_source_->reset();
}


void ImageSegmentationMeasurement::reset_data_loading_time()
{
    segmentation_source_->reset_data_loading_time();
}


double ImageSegmentationMeasurement::get_data_loading_time() const
{
    return segmentation_source_->get_data_loading_time();
}
