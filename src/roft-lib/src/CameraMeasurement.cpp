/*
 * Copyright (C) 2022 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#include <ROFT/CameraMeasurement.h>

#include <cstdint>
#include <opencv2/opencv.hpp>

using namespace Eigen;
using namespace ROFT;
using namespace RobotsIO::Camera;
using namespace bfl;


CameraMeasurement::CameraMeasurement(std::shared_ptr<Camera> camera) :
    camera_(camera)
{}


CameraMeasurement::~CameraMeasurement()
{}


bool CameraMeasurement::freeze(const Data& type)
{
    /* Step frame in case of an offline camera. */
    if (!camera_->step_frame())
        return false;

    /* Determine type of image required by the user. */
    measure_type_ = bfl::any::any_cast<CameraMeasurementType>(type);
    const bool use_rgb = (measure_type_ == CameraMeasurementType::RGB) || (measure_type_ == CameraMeasurementType::RGBD);
    const bool use_d = (measure_type_ == CameraMeasurementType::D) || (measure_type_ == CameraMeasurementType::RGBD);
    const bool use_pc = (measure_type_ == CameraMeasurementType::PC) || (measure_type_ == CameraMeasurementType::RGBPC);
    const bool use_rgbpc = (measure_type_ == CameraMeasurementType::RGBPC);

    bool valid_data = false;
    bool blocking_read = true;

    /* Pick new rgb image. */
    if (use_rgb)
    {
        std::tie(valid_data, rgb_) = camera_->rgb(blocking_read);
        if (!valid_data)
            return false;

        std::tie(is_time_stamp_rgb_, time_stamp_rgb_) = camera_->time_stamp_rgb();
    }
    valid_data = false;

    /* Pick new depth image. */
    if (use_d)
    {
        std::tie(valid_data, depth_) = camera_->depth(blocking_read);
        if (!valid_data)
            return false;

        std::tie(is_time_stamp_depth_, time_stamp_depth_) = camera_->time_stamp_depth();
    }
    valid_data = false;

    /* Pick new pose. */
    std::tie(valid_data, pose_) = camera_->pose(blocking_read);
    if (!valid_data)
        return false;

    valid_data = false;
    /* Pick new point cloud. */
    if (use_pc || use_rgbpc)
    {
        std::tie(valid_data, point_cloud_) = camera_->point_cloud(blocking_read, 10.0, false, use_rgbpc);
        if (!valid_data)
            return false;
    }

    measurement_available_ = true;

    return true;
}


std::pair<bool, Data> CameraMeasurement::measure(const Data& data) const
{
    const bool use_pc = (measure_type_ == CameraMeasurementType::PC) || (measure_type_ == CameraMeasurementType::RGBPC);

    if (use_pc)
        return std::make_pair(measurement_available_, std::make_tuple(pose_, point_cloud_));

    return std::make_pair(measurement_available_, std::make_tuple(pose_, rgb_, depth_));
}


std::pair<bool, Data> CameraMeasurement::predictedMeasure(const Ref<const MatrixXd>& cur_states) const
{
    /* Not implemented. */
    throw(std::runtime_error(log_name_ + "::step_frame. Not implemented in base class."));
}


std::pair<bool, Data> CameraMeasurement::innovation(const Data& predicted_measurements, const Data& measurements) const
{
    /* Not implemented. */
    throw(std::runtime_error(log_name_ + "::step_frame. Not implemented in base class."));
}


std::pair<bool, CameraParameters> CameraMeasurement::camera_parameters() const
{
    return camera_->parameters();
}


std::pair<bool, MatrixXd> CameraMeasurement::camera_deprojection_matrix() const
{
    return camera_->deprojection_matrix();
}


std::pair<bool, double> CameraMeasurement::camera_time_stamp_rgb() const
{
    return std::make_pair(is_time_stamp_rgb_, time_stamp_rgb_);
}


std::pair<bool, double> CameraMeasurement::camera_time_stamp_depth() const
{
    return std::make_pair(is_time_stamp_depth_, time_stamp_depth_);
}


std::int32_t CameraMeasurement::camera_frame_index() const
{
    return camera_->frame_index();
}


void CameraMeasurement::reset() const
{
    camera_->reset();
}
