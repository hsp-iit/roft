/*
 * Copyright (C) 2022 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#ifndef ROFT_CAMERAMEASUREMENTS_H
#define ROFT_CAMERAMEASUREMENTS_H

#include <BayesFilters/Data.h>
#include <BayesFilters/MeasurementModel.h>

#include <RobotsIO/Camera/Camera.h>
#include <RobotsIO/Camera/CameraParameters.h>

#include <Eigen/Dense>

#include <memory>

namespace ROFT {
    class CameraMeasurement;

    enum class CameraMeasurementType { RGB, D, RGBD, PC , RGBPC};
}


class ROFT::CameraMeasurement : public bfl::MeasurementModel
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    CameraMeasurement(std::shared_ptr<RobotsIO::Camera::Camera> camera);

    ~CameraMeasurement();

    bool freeze(const bfl::Data& type = bfl::Data()) override;

    std::pair<bool, bfl::Data> measure(const bfl::Data& data = bfl::Data()) const override;

    std::pair<bool, bfl::Data> predictedMeasure(const Eigen::Ref<const Eigen::MatrixXd>& cur_states) const override;

    std::pair<bool, bfl::Data> innovation(const bfl::Data& predicted_measurements, const bfl::Data& measurements) const override;

    std::pair<bool, RobotsIO::Camera::CameraParameters> camera_parameters() const;

    std::pair<bool, Eigen::MatrixXd> camera_deprojection_matrix() const;

    std::pair<bool, double> camera_time_stamp_rgb() const;

    std::pair<bool, double> camera_time_stamp_depth() const;

    std::int32_t camera_frame_index() const;

    void reset() const;

    using CameraMeasurementTuple = std::tuple<Eigen::Transform<double, 3, Eigen::Affine>, cv::Mat, Eigen::MatrixXf>;

private:
    std::shared_ptr<RobotsIO::Camera::Camera> camera_;

    cv::Mat rgb_;

    Eigen::MatrixXf depth_;

    Eigen::MatrixXd point_cloud_;

    Eigen::Transform<double, 3, Eigen::Affine> pose_;

    double time_stamp_rgb_;

    double time_stamp_depth_;

    bool is_time_stamp_rgb_ = false;

    bool is_time_stamp_depth_ = false;

    ROFT::CameraMeasurementType measure_type_;

    bool measurement_available_ = false;

    std::string log_name_ = "CameraMeasurements";
};

#endif /* ROFT_CAMERAMEASUREMENTS_H */
