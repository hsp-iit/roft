/*
 * Copyright (C) 2022 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#ifndef ROFT_IMAGEOPTICAFLOWMEASUREMENT_HPP
#define ROFT_IMAGEOPTICAFLOWMEASUREMENT_HPP

#include <BayesFilters/any.h>
#include <BayesFilters/Data.h>
#include <BayesFilters/LinearMeasurementModel.h>
#include <BayesFilters/VectorDescription.h>

#include <Eigen/Dense>

#include <ROFT/CameraMeasurement.h>
#include <ROFT/ImageOpticalFlowSource.h>
#include <ROFT/ImageSegmentationMeasurement.h>
#include <ROFT/OpticalFlowUtilities.h>

#include <RobotsIO/Camera/CameraParameters.h>

#include <memory>

namespace ROFT {
    class ImageOpticalFlowMeasurementBase;

    template<class T>
    class ImageOpticalFlowMeasurement;
}


class ROFT::ImageOpticalFlowMeasurementBase
{
public:
    enum class FreezeType { OnlyStepSource, ExceptStepSource, Complete };
};


template<class T>
class ROFT::ImageOpticalFlowMeasurement : public bfl::LinearMeasurementModel, ROFT::ImageOpticalFlowMeasurementBase
{
public:

    ImageOpticalFlowMeasurement(std::shared_ptr<ImageOpticalFlowSource> flow_source, std::shared_ptr<CameraMeasurement> camera_measurement, std::shared_ptr<ROFT::ImageSegmentationMeasurement> segmentation, const std::size_t& segmentation_radius, const double& maximum_depth, Eigen::Ref<const Eigen::MatrixXd> covariance, const bool use_full_covariance_matrix);

    ~ImageOpticalFlowMeasurement();

    bool freeze(const bfl::Data& data = bfl::Data()) override;

    std::pair<bool, bfl::Data> measure(const bfl::Data& data = bfl::Data()) const override;

    std::pair<bool, bfl::Data> predictedMeasure(const Eigen::Ref<const Eigen::MatrixXd>& cur_states) const override;

    std::pair<bool, bfl::Data> innovation(const bfl::Data& predicted_measurements, const bfl::Data& measurements) const override;

    Eigen::MatrixXd getMeasurementMatrix() const override;

    std::pair<bool, Eigen::MatrixXd> getNoiseCovarianceMatrix() const override;

    bfl::VectorDescription getInputDescription() const override;

    bfl::VectorDescription getMeasurementDescription() const override;

    bool setProperty(const std::string& property) override;

    void reset_data_loading_time();

    double get_data_loading_time();

private:

    std::shared_ptr<ImageOpticalFlowSource> flow_;

    std::shared_ptr<CameraMeasurement> camera_;

    std::shared_ptr<ImageSegmentationMeasurement> segmentation_;

    /* Storage. */

    Eigen::MatrixXd measurement_matrix_;

    Eigen::MatrixXd measurement_;

    Eigen::MatrixXd covariance_;

    Eigen::MatrixXf previous_depth_;

    cv::Mat previous_segmentation_;

    bool use_full_covariance_;

    RobotsIO::Camera::CameraParameters camera_parameters_;

    /* Segmentation parameters. */

    const float segmentation_radius_;

    /* Depth parameters. */

    const double maximum_depth_;

    /* Optical flow parameters. */

    const std::size_t flow_grid_size_;

    const float flow_scaling_factor_;

    /* Sample time. */

    double sample_time_;

    /* State machine parameters. */

    bool flow_available_ = false;

    bool is_first_frame_ = true;

    ROFT::ImageOpticalFlowMeasurementBase::FreezeType freeze_type_;

    /* Logging. */

    double data_loading_time_ = 0.0;

    const std::string log_name_ = "ImageOpticalFlowMeasurement";
};


template<class T>
ROFT::ImageOpticalFlowMeasurement<T>::ImageOpticalFlowMeasurement
(
    std::shared_ptr<ImageOpticalFlowSource> flow_source,
    std::shared_ptr<CameraMeasurement> camera_measurement,
    std::shared_ptr<ROFT::ImageSegmentationMeasurement> segmentation,
    const std::size_t& segmentation_radius,
    const double& maximum_depth,
    Eigen::Ref<const Eigen::MatrixXd> covariance,
    const bool use_full_covariance_matrix
) :
    flow_(flow_source),
    camera_(camera_measurement),
    segmentation_(segmentation),
    covariance_(covariance),
    use_full_covariance_(use_full_covariance_matrix),
    segmentation_radius_(segmentation_radius),
    maximum_depth_(maximum_depth),
    flow_grid_size_(flow_source->get_grid_size()),
    flow_scaling_factor_(flow_source->get_scaling_factor())
{
    /* Get camera parameters. */
    bool valid_camera_parameters = false;
    std::tie(valid_camera_parameters, camera_parameters_) = camera_->camera_parameters();
    if (!valid_camera_parameters)
    {
        throw(std::runtime_error(log_name_ + "::ctor. Error: cannot get camera parameters."));
    }
}


template<class T>
ROFT::ImageOpticalFlowMeasurement<T>::~ImageOpticalFlowMeasurement()
{}


template<class T>
bool ROFT::ImageOpticalFlowMeasurement<T>::freeze(const bfl::Data& data)
{
    /* Recover freezing type and the sample time. */
    std::tie(freeze_type_, sample_time_) = bfl::any::any_cast<std::pair<FreezeType, double>>(data);

    /* Step source if it is required. */
    if (freeze_type_ != FreezeType::ExceptStepSource)
    {
        if (flow_->is_stepping_required())
            flow_->step_frame();
    }

    /* If only source stepping is required, stop here. */
    if (freeze_type_ == FreezeType::OnlyStepSource)
        return true;

    /* Get segmentation.
       Up to this point we assume that segmentation_ has been already frozen, i.e. segmentation_->freeze(), somewhere else. */
    bfl::Data segmentation_data;
    bool valid_segmentation = false;
    std::tie(valid_segmentation, segmentation_data) = segmentation_->measure();
    if (!valid_segmentation)
    {
        std::cout << log_name_ << "::freeze. Warning: segmentation source is not available." << std::endl;
        return false;
    }

    cv::Mat segmentation;
    std::tie(std::ignore, segmentation) = bfl::any::any_cast<std::pair<bool, cv::Mat>>(segmentation_data);

    /* Get depth.
       Up to this point we assume that camera_ has been already frozen, i.e. camera_->freeze(), somewhere else. */
    bfl::Data camera_data;
    bool valid_data = false;
    std::tie(valid_data, camera_data) = camera_->measure();
    if (!valid_data)
    {
        std::cout << log_name_ << "::freeze. Warning: depth from camera measurement is not available." << std::endl;
        return false;
    }

    Eigen::MatrixXf depth;
    std::tie(std::ignore, std::ignore, depth) = bfl::any::any_cast<CameraMeasurement::CameraMeasurementTuple>(camera_data);

    /* Get optical flow. */
    cv::Mat flow;
    flow_available_ = false;
    std::tie(flow_available_, flow) = flow_->flow(false);

    if ((!flow_available_) || is_first_frame_)
    {
        std::cout << log_name_ << "::freeze. Warning: flow source is not available." << std::endl;

        /* Store depth and segmentation for the next execution. */
        previous_depth_ = depth;
        previous_segmentation_ = segmentation;

        /* Reset status flag. */
        is_first_frame_ = false;

        return false;
    }

    /* Find non zero coordinates within the segmentation. */
    cv::Mat coordinates;
    cv::findNonZero(previous_segmentation_, coordinates);

    /* Store as a measurement vector. */
    std::vector<cv::Point> valid_coordinates;
    for (std::size_t i = 0; i < coordinates.total(); i+= segmentation_radius_)
    {
        /* Coordinates. */
        const int& u = coordinates.at<cv::Point>(i).x;
        const int& v = coordinates.at<cv::Point>(i).y;

        /* Depth. */
        const float& depth_v_u = previous_depth_(v, u);

        /* Flow. */
        const T& flow_v_u = flow.at<T>(v / flow_grid_size_, u / flow_grid_size_);
        /* Given that T is template paramter, f might not be neessary a float, hence it is required to cast it. */
        float dx = float(flow_v_u(0)) / flow_scaling_factor_;
        float dy = float(flow_v_u(1)) / flow_scaling_factor_;

        if (ROFT::OpticalFlowUtils::is_flow_valid(dx, dy) && depth_v_u > 0 && depth_v_u < maximum_depth_)
            valid_coordinates.push_back(cv::Point(u, v));
    }
    measurement_.resize(2 * valid_coordinates.size(), 1);
    measurement_matrix_.resize(2 * valid_coordinates.size(), 6);

    for (std::size_t i = 0; i < valid_coordinates.size(); i++)
    {
        /* Coordinates. */
        const int& u = valid_coordinates[i].x;
        const int& v = valid_coordinates[i].y;

        /* Depth. */
        const float& depth_v_u = previous_depth_(v, u);

        /* Flow. */
        const T& flow_v_u = flow.at<T>(v / flow_grid_size_, u / flow_grid_size_);

        /* Measurement. */
        /* Given that T is template paramter, flow_v_u might not be neessary a float, hence it is required to cast it. */
        measurement_.col(0).segment<2>(2 * i)(0) = float(flow_v_u(0)) / flow_scaling_factor_;
        measurement_.col(0).segment<2>(2 * i)(1) = float(flow_v_u(1)) / flow_scaling_factor_;

        /* Measurement matrix. */
        Eigen::MatrixXd measurement_matrix(2, 6);
        double uu = (u - camera_parameters_.cx());
        double vv = (v - camera_parameters_.cy());
        measurement_matrix << camera_parameters_.fx() / depth_v_u, 0.0, -uu / depth_v_u, -uu * vv / camera_parameters_.fy(), camera_parameters_.fx() + uu * uu / camera_parameters_.fx(), -vv * camera_parameters_.fx() / camera_parameters_.fy(),
                              0.0, camera_parameters_.fy() / depth_v_u, -vv / depth_v_u, -(camera_parameters_.fy() + vv * vv / camera_parameters_.fy()), vv * uu / camera_parameters_.fx(), uu * camera_parameters_.fy() / camera_parameters_.fx();
        measurement_matrix *= sample_time_;
        measurement_matrix_.middleRows<2>(2 * i) = measurement_matrix;
    }

    /* Store depth and segmentation for the next execution. */
    previous_depth_ = depth;
    previous_segmentation_ = segmentation;

    data_loading_time_ = 0.0;
    data_loading_time_ += flow_->get_data_loading_time();
    data_loading_time_ += segmentation_->get_data_loading_time();

    return flow_available_;
}


template<class T>
std::pair<bool, bfl::Data> ROFT::ImageOpticalFlowMeasurement<T>::measure(const bfl::Data& data) const
{
    return std::make_pair(flow_available_, measurement_);
}


template<class T>
std::pair<bool, bfl::Data> ROFT::ImageOpticalFlowMeasurement<T>::predictedMeasure(const Eigen::Ref<const Eigen::MatrixXd>& cur_states) const
{
    if (!flow_available_)
        return std::make_pair(false, bfl::Data());

    return std::make_pair(true, Eigen::MatrixXd(measurement_matrix_ * cur_states));
}


template<class T>
std::pair<bool, bfl::Data> ROFT::ImageOpticalFlowMeasurement<T>::innovation(const bfl::Data& predicted_measurements, const bfl::Data& measurements) const
{
    Eigen::MatrixXd innovation = -(bfl::any::any_cast<Eigen::MatrixXd>(predicted_measurements).colwise() - bfl::any::any_cast<Eigen::MatrixXd>(measurements).col(0));

    return std::make_pair(true, std::move(innovation));
}


template<class T>
Eigen::MatrixXd ROFT::ImageOpticalFlowMeasurement<T>::getMeasurementMatrix() const
{
    return measurement_matrix_;
}


template<class T>
std::pair<bool, Eigen::MatrixXd> ROFT::ImageOpticalFlowMeasurement<T>::getNoiseCovarianceMatrix() const
{
    if (use_full_covariance_)
    {
        Eigen::MatrixXd covariance = Eigen::MatrixXd::Zero(measurement_.col(0).size(), measurement_.col(0).size());
        for (std::size_t i = 0; i < (measurement_.col(0).size() / 2); i++)
            covariance.block<2, 2>(2 * i, 2* i) = covariance_;

        return std::make_pair(true, covariance);
    }

    return std::make_pair(true, covariance_);
}


template<class T>
bfl::VectorDescription ROFT::ImageOpticalFlowMeasurement<T>::getInputDescription() const
{
    return bfl::VectorDescription(6, 0, measurement_.size());
}


template<class T>
bfl::VectorDescription ROFT::ImageOpticalFlowMeasurement<T>::getMeasurementDescription() const
{
    return bfl::VectorDescription(measurement_.size(), 0);
}


template<class T>
bool ROFT::ImageOpticalFlowMeasurement<T>::setProperty(const std::string& property)
{
    if (property == "check_observability")
    {
        return ((measurement_.col(0).size() / 2) >= 3);
    }
    else if (property == "reset")
    {
        flow_available_ = false;
        is_first_frame_ = true;
        return true;
    }

    return false;
}


template<class T>
void ROFT::ImageOpticalFlowMeasurement<T>::reset_data_loading_time()
{
    segmentation_->reset_data_loading_time();
}


template<class T>
double ROFT::ImageOpticalFlowMeasurement<T>::get_data_loading_time()
{
    return data_loading_time_;
}


#endif /* ROFT_IMAGEOPTICALFLOWMEASUREMENT_HPP */
