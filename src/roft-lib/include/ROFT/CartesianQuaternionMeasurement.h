/*
 * Copyright (C) 2022 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#ifndef ROFT_CARTESIANQUATERNIONMEASUREMENT_H
#define ROFT_CARTESIANQUATERNIONMEASUREMENT_H

#include <Eigen/Dense>

#include <BayesFilters/MeasurementModel.h>

#include <ROFT/CameraMeasurement.h>

#include <RobotsIO/Utils/Transform.h>
#include <RobotsIO/Utils/SpatialVelocity.h>

#include <deque>
#include <memory>

namespace ROFT {
    class CartesianQuaternionMeasurement;
}


class ROFT::CartesianQuaternionMeasurement : public bfl::MeasurementModel
{
public:
    CartesianQuaternionMeasurement(std::shared_ptr<RobotsIO::Utils::Transform> pose_measurement, std::shared_ptr<RobotsIO::Utils::SpatialVelocity> velocity_measurement, const bool use_screw_velocity, const bool use_pose_measurement, const bool use_velocity_measurement, const Eigen::Ref<const Eigen::MatrixXd> sigma_position, const Eigen::Ref<const Eigen::MatrixXd> sigma_quaternion, const Eigen::Ref<const Eigen::MatrixXd> sigma_linear_velocity, const Eigen::Ref<const Eigen::MatrixXd> sigma_angular_velocity, const bool enable_log);

    CartesianQuaternionMeasurement(std::shared_ptr<RobotsIO::Utils::Transform> pose_measurement, std::shared_ptr<RobotsIO::Utils::SpatialVelocity> velocity_measurement, std::shared_ptr<ROFT::CameraMeasurement> camera_measurement, const bool use_screw_velocity, const bool use_pose_measurement, const bool use_velocity_measurement, const Eigen::Ref<const Eigen::MatrixXd> sigma_position, const Eigen::Ref<const Eigen::MatrixXd> sigma_quaternion, const Eigen::Ref<const Eigen::MatrixXd> sigma_linear_velocity, const Eigen::Ref<const Eigen::MatrixXd> sigma_angular_velocity, const bool wait_source_initialization, const bool enable_log);

    virtual ~CartesianQuaternionMeasurement();

    bool freeze(const bfl::Data& data = bfl::Data()) override;

    std::pair<bool, bfl::Data> measure(const bfl::Data& data = bfl::Data()) const override;

    std::pair<bool, bfl::Data> predictedMeasure(const Eigen::Ref<const Eigen::MatrixXd>& current_states) const override;

    std::pair<bool, bfl::Data> innovation(const bfl::Data& predicted_measurements, const bfl::Data& measurements) const override;

    std::pair<bool, Eigen::MatrixXd> getNoiseCovarianceMatrix() const override;

    bfl::VectorDescription getInputDescription() const override;

    bfl::VectorDescription getMeasurementDescription() const override;

    bool setProperty(const std::string& property) override;

    enum class MeasurementMode { Standard, RepeatOnlyVelocity, PopBufferedMeasurement };

protected:
    std::vector<std::string> log_file_names(const std::string& prefix_path, const std::string& prefix_name) override;

private:
    std::shared_ptr<RobotsIO::Utils::Transform> pose_measurement_;

    std::shared_ptr<RobotsIO::Utils::SpatialVelocity> velocity_measurement_;

    std::shared_ptr<ROFT::CameraMeasurement> camera_measurement_;

    bool is_velocity_measurement_degenerate_ = false;

    Eigen::MatrixXd measurement_;

    Eigen::MatrixXd noise_covariance_;

    Eigen::MatrixXd R_pose_;

    Eigen::MatrixXd R_velocity_;

    Eigen::MatrixXd R_pose_velocity_;

    Eigen::Vector3d last_linear_velocity_;

    Eigen::Vector3d last_angular_velocity_;

    Eigen::Transform<double, 3, Eigen::Affine>  last_pose_;

    std::deque<Eigen::VectorXd> buffer_velocities_;

    enum class MeasurementType { Pose, Velocity, PoseVelocity, None };

    MeasurementType measurement_type_ = MeasurementType::None;

    MeasurementMode measurement_mode_ = MeasurementMode::Standard;

    bfl::VectorDescription input_description_;

    bfl::VectorDescription measurement_description_;

    bool is_first_velocity_in_ = false;

    bool is_pose_ = false;

    bool use_screw_velocity_;

    bool use_pose_measurement_;

    bool use_velocity_measurement_;

    const bool wait_source_initialization_default_;

    bool wait_source_initialization_;

    int pose_frames_between_iterations_;

    bool enable_log_;

    const std::string log_name_ = "CartesianQuaternionMeasurement";
};

#endif /* ROFT_CARTESIANQUATERNIONMEASUREMENT_H */
