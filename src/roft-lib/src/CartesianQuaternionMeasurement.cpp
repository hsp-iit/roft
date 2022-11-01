/*
 * Copyright (C) 2022 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#include <ROFT/CartesianQuaternionMeasurement.h>

#include <BayesFilters/utils.h>

#include <chrono>
#include <thread>

using namespace Eigen;
using namespace ROFT;
using namespace bfl;
using namespace bfl::utils;
using namespace std::literals::chrono_literals;


CartesianQuaternionMeasurement::CartesianQuaternionMeasurement
(
    std::shared_ptr<RobotsIO::Utils::Transform> pose_measurement,
    std::shared_ptr<RobotsIO::Utils::SpatialVelocity> velocity_measurement,
    std::shared_ptr<ROFT::CameraMeasurement> camera_measurement,
    const bool use_screw_velocity,
    const bool use_pose_measurement,
    const bool use_velocity_measurement,
    const Ref<const MatrixXd> sigma_position,
    const Ref<const MatrixXd> sigma_quaternion,
    const Ref<const MatrixXd> sigma_linear_velocity,
    const Ref<const MatrixXd> sigma_angular_velocity,
    const bool wait_source_initialization,
    const bool enable_log
) :
    pose_measurement_(pose_measurement),
    velocity_measurement_(velocity_measurement),
    camera_measurement_(camera_measurement),
    use_screw_velocity_(use_screw_velocity),
    use_pose_measurement_(use_pose_measurement),
    use_velocity_measurement_(use_velocity_measurement),
    wait_source_initialization_(wait_source_initialization),
    pose_frames_between_iterations_(pose_measurement->get_frames_between_iterations()),
    enable_log_(enable_log)
{
    /* Cache covariance matrix for all the possible combinations .*/
    R_pose_ = MatrixXd::Zero(6, 6);
    R_pose_.block<3, 3>(0, 0) = sigma_position;
    R_pose_.block<3, 3>(3, 3) = sigma_quaternion;

    R_velocity_ = MatrixXd::Zero(6, 6);
    R_velocity_.block<3, 3>(0, 0) = sigma_linear_velocity;
    R_velocity_.block<3, 3>(3, 3) = sigma_angular_velocity;

    R_pose_velocity_ = MatrixXd::Zero(12, 12);
    R_pose_velocity_.block<6, 6>(0, 0) = R_velocity_;
    R_pose_velocity_.block<6, 6>(6, 6) = R_pose_;
}


CartesianQuaternionMeasurement::CartesianQuaternionMeasurement
(
    std::shared_ptr<RobotsIO::Utils::Transform> pose_measurement,
    std::shared_ptr<RobotsIO::Utils::SpatialVelocity> velocity_measurement,
    const bool use_screw_velocity,
    const bool use_pose_measurement,
    const bool use_velocity_measurement,
    const Ref<const MatrixXd> sigma_position,
    const Ref<const MatrixXd> sigma_quaternion,
    const Ref<const MatrixXd> sigma_linear_velocity,
    const Ref<const MatrixXd> sigma_angular_velocity,
    const bool enable_log
) :
    CartesianQuaternionMeasurement
    (
        pose_measurement, velocity_measurement, /* camera_measurement */ nullptr,
        use_screw_velocity, use_pose_measurement, use_velocity_measurement,
        sigma_position, sigma_quaternion, sigma_linear_velocity, sigma_angular_velocity,
        /* wait_source_initialization = */ false, enable_log
    )
{}


CartesianQuaternionMeasurement::~CartesianQuaternionMeasurement()
{}


bool CartesianQuaternionMeasurement::freeze(const Data& data)
{
    /* Extract the measurement mode. */
    MeasurementMode mode = bfl::any::any_cast<MeasurementMode>(data);

    if (mode == MeasurementMode::PopBufferedMeasurement)
    {

        if (pose_frames_between_iterations_ > 0)
        {
            while (buffer_velocities_.size() > pose_frames_between_iterations_ + 1)
                buffer_velocities_.pop_front();
        }

        /* We need to store the last velocity we received,
           to be used later on when we will receive the pose associated to the current frame. */
        if (buffer_velocities_.size() == 0)
        {
            buffer_velocities_.push_back(measurement_.col(0).head<6>());
            return false;
        }

        VectorXd buffered_velocity = buffer_velocities_.at(0);
        buffer_velocities_.pop_front();
        last_linear_velocity_ = buffered_velocity.head<3>();
        last_angular_velocity_ = buffered_velocity.tail<3>();

        if (is_pose_)
        {
            measurement_type_ = MeasurementType::PoseVelocity;
            input_description_ = VectorDescription(9, 1, 12, VectorDescription::CircularType::Quaternion);
            measurement_description_ = VectorDescription(9, 1, 0, VectorDescription::CircularType::Quaternion);

            measurement_.resize(13, 1);
            measurement_.col(0).head<3>() = last_linear_velocity_;
            measurement_.col(0).segment<3>(3) = last_angular_velocity_;
            measurement_.col(0).segment<3>(6) = last_pose_.translation();
            Quaterniond q(last_pose_.rotation());
            measurement_.col(0).tail<4>()(0) = q.w();
            measurement_.col(0).tail<4>()(1) = q.x();
            measurement_.col(0).tail<4>()(2) = q.y();
            measurement_.col(0).tail<4>()(3) = q.z();

            noise_covariance_ = R_pose_velocity_;

            /* Once consumed, the pose is not valid anymore. */
            is_pose_ = false;
        }
        else
        {
            measurement_type_ = MeasurementType::Velocity;
            input_description_ = VectorDescription(9, 1, 6, VectorDescription::CircularType::Quaternion);
            measurement_description_ = VectorDescription(6);

            measurement_.resize(6, 1);
            measurement_.col(0).head<3>() = last_linear_velocity_;
            measurement_.col(0).tail<3>() = last_angular_velocity_;

            noise_covariance_ = R_velocity_;
        }

        return true;
    }

    if (mode == MeasurementMode::RepeatOnlyVelocity)
    {
        if (is_first_velocity_in_)
        {
            /* Repeat the current measurement but excluding the pose part. */

            measurement_type_ = MeasurementType::Velocity;
            input_description_ = VectorDescription(9, 1, 6, VectorDescription::CircularType::Quaternion);
            measurement_description_ = VectorDescription(6);

            measurement_.resize(6, 1);
            measurement_.col(0).head<3>() = last_linear_velocity_;
            measurement_.col(0).tail<3>() = last_angular_velocity_;

            noise_covariance_ = R_velocity_;
        }

        return true;
    }

    if (use_velocity_measurement_ && velocity_measurement_->freeze(true))
    {
        is_first_velocity_in_ = true;
        is_velocity_measurement_degenerate_ = true;
        last_linear_velocity_ = velocity_measurement_->linear_velocity_origin();
        last_angular_velocity_ = velocity_measurement_->angular_velocity();

        if (use_screw_velocity_)
        {
            if (last_angular_velocity_.norm() > 1e-4)
            {
                last_linear_velocity_ = velocity_measurement_->linear_velocity_screw();
                is_velocity_measurement_degenerate_ = false;
            }
        }
    }

    /* Provide RGB input for Transform sources that might require it. */
    cv::Mat rgb;
    if (camera_measurement_ != nullptr)
    {
        bfl::Data camera_data;
        bool valid_data = false;
        std::tie(valid_data, camera_data) = camera_measurement_->measure();
        if (!valid_data)
        {
            std::cout << log_name_ << "::freeze. Warning: RGB from camera measurement is not available." << std::endl;
            return false;
        }
        std::tie(std::ignore, rgb, std::ignore) = bfl::any::any_cast<CameraMeasurement::CameraMeasurementTuple>(camera_data);
    }

    is_pose_ = false;
    if (use_pose_measurement_)
    {
        is_pose_ = pose_measurement_->freeze(false);

        if ((!rgb.empty()) && wait_source_initialization_)
        {
            std::size_t number_trials = 5;
            for (std::size_t i = 0; (i < number_trials) && (!is_pose_); i++)
            {
                pose_measurement_->set_rgb_image(rgb);

                is_pose_ = pose_measurement_->freeze(false);

                std::this_thread::sleep_for(500ms);
            }

            if (is_pose_)
                wait_source_initialization_ = false;
            else
                return false;
        }

        if (is_pose_)
            last_pose_ = pose_measurement_->transform();
        else if ((pose_frames_between_iterations_ < 0) && (pose_measurement_->transform_received()))
        {
            /* If the pose was received but it is invalid and
               the number of frames between each iteration is not known,
               the velocity buffer must be voided. */
            while (buffer_velocities_.size() > 1)
                buffer_velocities_.pop_front();
        }


        if ((!rgb.empty()) && (is_pose_ || pose_measurement_->transform_received()))
            pose_measurement_->set_rgb_image(rgb);
    }

    bool valid_freeze = true;
    if (is_first_velocity_in_ && is_pose_)
    {
        measurement_type_ = MeasurementType::PoseVelocity;
        input_description_ = VectorDescription(9, 1, 12, VectorDescription::CircularType::Quaternion);
        measurement_description_ = VectorDescription(9, 1, 0, VectorDescription::CircularType::Quaternion);

        measurement_.resize(13, 1);
        measurement_.col(0).head<3>() = last_linear_velocity_;
        measurement_.col(0).segment<3>(3) = last_angular_velocity_;
        measurement_.col(0).segment<3>(6) = last_pose_.translation();
        Quaterniond q(last_pose_.rotation());
        measurement_.col(0).tail<4>()(0) = q.w();
        measurement_.col(0).tail<4>()(1) = q.x();
        measurement_.col(0).tail<4>()(2) = q.y();
        measurement_.col(0).tail<4>()(3) = q.z();

        buffer_velocities_.push_back(measurement_.col(0).head<6>());

        noise_covariance_ = R_pose_velocity_;
    }
    else if (is_first_velocity_in_)
    {
        measurement_type_ = MeasurementType::Velocity;
        input_description_ = VectorDescription(9, 1, 6, VectorDescription::CircularType::Quaternion);
        measurement_description_ = VectorDescription(6);

        measurement_.resize(6, 1);
        measurement_.col(0).head<3>() = last_linear_velocity_;
        measurement_.col(0).tail<3>() = last_angular_velocity_;

        buffer_velocities_.push_back(measurement_.col(0));

        noise_covariance_ = R_velocity_;
    }
    else if (is_pose_)
    {
        measurement_type_ = MeasurementType::Pose;
        input_description_ = VectorDescription(9, 1, 6, VectorDescription::CircularType::Quaternion);
        measurement_description_ = VectorDescription(3, 1, 0, VectorDescription::CircularType::Quaternion);

        measurement_.resize(7, 1);
        measurement_.col(0).head<3>() = last_pose_.translation();
        Quaterniond q(last_pose_.rotation());
        measurement_.col(0).tail<4>()(0) = q.w();
        measurement_.col(0).tail<4>()(1) = q.x();
        measurement_.col(0).tail<4>()(2) = q.y();
        measurement_.col(0).tail<4>()(3) = q.z();

        noise_covariance_ = R_pose_;
    }
    else
    {
        measurement_type_ = MeasurementType::None;
        input_description_ = VectorDescription(0, 0, 0);
        measurement_description_ = VectorDescription(0, 0, 0);
        valid_freeze = false;
    }

    if (enable_log_)
    {
        VectorXd pose_vector(7);
        AngleAxisd pose_aa(last_pose_.rotation());
        pose_vector.head<3>() = last_pose_.translation();
        pose_vector.segment<3>(3) = pose_aa.axis();
        pose_vector(6) = pose_aa.angle();

        VectorXd velocity_vector(6);
        velocity_vector.head<3>() = last_linear_velocity_;
        velocity_vector.tail<3>() = last_angular_velocity_;

        logger(pose_vector.transpose(), velocity_vector.transpose());
    }

    return valid_freeze;
}


std::pair<bool, bfl::Data> CartesianQuaternionMeasurement::measure(const Data& data) const
{
    return std::make_pair(measurement_type_ != MeasurementType::None, measurement_);
}


std::pair<bool, bfl::Data> CartesianQuaternionMeasurement::predictedMeasure(const Ref<const MatrixXd>& current_states) const
{
    /* Warning. This a bfl::MeasurementModel, i.e. the most general form of measurement model.
       For this reason, it is assumed that noise_covariance_.rows() noise components are provided in cur_states.bottomRows(noise_covariance_.cols()). */
    const Ref<const MatrixXd> noise = current_states.bottomRows(noise_covariance_.rows());

    MatrixXd predicted_measure(measurement_description_.total_size(), current_states.cols());
    MatrixXd predicted_pose(7, current_states.cols());
    MatrixXd predicted_velocity(6, current_states.cols());

    for (std::size_t i = 0; i < current_states.cols(); i++)
    {
        if ((measurement_type_ == MeasurementType::PoseVelocity) || (measurement_type_ == MeasurementType::Pose))
        {
            predicted_pose.col(i) = current_states.col(i).segment<7>(6);
            if (measurement_type_ == MeasurementType::Pose)
                predicted_pose.col(i).head<3>() += noise.col(i).head<3>();
            else
                predicted_pose.col(i).head<3>() += noise.col(i).segment<3>(6);

            VectorXd q = predicted_pose.col(i).tail<4>();
            predicted_pose.col(i).tail<4>() = sum_quaternion_rotation_vector(q, noise.col(i).tail<3>());
        }

        if ((measurement_type_ == MeasurementType::PoseVelocity) || (measurement_type_ == MeasurementType::Velocity))
        {
            const Ref<const Vector3d> p = current_states.col(i).segment<3>(6);
            const Ref<const Vector3d> v_object = current_states.col(i).head<3>();
            const Ref<const Vector3d> w = current_states.col(i).segment<3>(3);

            predicted_velocity.col(i).head<3>() = v_object;
            if (use_screw_velocity_)
            {
                if (is_velocity_measurement_degenerate_)
                {
                    /* In this case, velocity_measurement_->linear_velocity_screw() == velocity_measurement_->linear_velocity_origin(). */
                    predicted_velocity.col(i).head<3>() += w.cross(-p);
                }
                else
                {
                    const double norm_w = w.norm();
                    if (norm_w > 1e-4)
                        predicted_velocity.col(i).head<3>() += w.cross(w.cross(v_object)) / std::pow(norm_w, 2.0);
                    else
                    {
                        std::cout << log_name_ + "::predictedMeasure. Error: one of the sigma point has degenerate velocity (context: use_screw_velocity == true, is_velocity_measurement_degenerate == false, sigma_angular_velocity.norm() == " + std::to_string(norm_w) + ", last_angular_velocity.norm() == " + std::to_string(last_angular_velocity_.norm()) + "." << std::endl;
                        std::cout << log_name_ + "::predictedMeasure. Information: the measured angular velocity will be used for that sigma point." << std::endl;

                        predicted_velocity.col(i).head<3>() += last_angular_velocity_.cross(last_angular_velocity_.cross(v_object)) / std::pow(last_angular_velocity_.norm(), 2.0);
                    }
                }
            }
            else
                predicted_velocity.col(i).head<3>() += w.cross(-p);

            predicted_velocity.col(i).tail<3>() = w;

            predicted_velocity.col(i) += noise.col(i).head<6>();
        }
    }

    if (measurement_type_ == MeasurementType::PoseVelocity)
    {
        // predicted_pose.swap(predicted_measure.bottomRows<7>());
        // predicted_velocity.swap(predicted_measure.topRows<6>());
        predicted_measure.bottomRows<7>() = predicted_pose;
        predicted_measure.topRows<6>() = predicted_velocity;
    }
    else if (measurement_type_ == MeasurementType::Pose)
        // predicted_pose.swap(predicted_measure);
        predicted_measure = predicted_pose;
    else if (measurement_type_ == MeasurementType::Velocity)
        // predicted_velocity.swap(predicted_measure);
        predicted_measure = predicted_velocity;

    return std::make_pair(measurement_type_ != MeasurementType::None, predicted_measure);
}


std::pair<bool, bfl::Data> CartesianQuaternionMeasurement::innovation(const bfl::Data& predicted_measurements, const bfl::Data& measurements) const
{
    MatrixXd predicted = any::any_cast<MatrixXd>(predicted_measurements);

    MatrixXd innovation_position;
    MatrixXd innovation_quaternion;
    MatrixXd innovation_velocity;

    std::size_t innovation_size = 0;

    if ((measurement_type_ == MeasurementType::PoseVelocity) || (measurement_type_ == MeasurementType::Pose))
    {
        innovation_size += 6;
        if (measurement_type_ == MeasurementType::Pose)
            innovation_position = -(predicted.topRows<3>().colwise() - measurement_.col(0).head<3>());
        else
            innovation_position = -(predicted.middleRows<3>(6).colwise() - measurement_.col(0).segment<3>(6));

        innovation_quaternion.resize(3, predicted.cols());
        for (std::size_t i = 0; i < predicted.cols(); i++)
            innovation_quaternion.col(i) = diff_quaternion(measurement_.col(0).tail<4>(), predicted.col(i).tail<4>());
    }

    if ((measurement_type_ == MeasurementType::PoseVelocity) || (measurement_type_ == MeasurementType::Velocity))
    {
        innovation_size += 6;
        innovation_velocity = -(predicted.topRows<6>().colwise() - measurement_.col(0).head<6>());
    }

    MatrixXd innovation(innovation_size, predicted.cols());
    if (measurement_type_ == MeasurementType::PoseVelocity)
    {
        // innovation_position.swap(innovation.middleRows<3>(6));
        // innovation_quaternion.swap(innovation.bottomRows<3>());
        // innovation_velocity.swap(innovation.topRows<6>());
        innovation.middleRows<3>(6) = innovation_position;
        innovation.bottomRows<3>() = innovation_quaternion;
        innovation.topRows<6>() = innovation_velocity;
    }
    else if (measurement_type_ == MeasurementType::Pose)
    {
        // innovation_position.swap(innovation.topRows<3>());
        // innovation_quaternion.swap(innovation.bottomRows<3>());
        innovation.topRows<3>() = innovation_position;
        innovation.bottomRows<3>() = innovation_quaternion;
    }
    else if (measurement_type_ == MeasurementType::Velocity)
        // innovation_velocity.swap(innovation);
        innovation = innovation_velocity;

    return std::make_pair(true, std::move(innovation));
}


std::pair<bool, MatrixXd> CartesianQuaternionMeasurement::getNoiseCovarianceMatrix() const
{
    return std::make_pair(measurement_type_ != MeasurementType::None, noise_covariance_);
}


VectorDescription CartesianQuaternionMeasurement::getInputDescription() const
{
    return input_description_;
}


VectorDescription CartesianQuaternionMeasurement::getMeasurementDescription() const
{
    return measurement_description_;
}


std::vector<std::string> CartesianQuaternionMeasurement::log_file_names(const std::string& prefix_path, const std::string& prefix_name)
{
    return  {prefix_path + "/" + prefix_name + "pose_measurements",
             prefix_path + "/" + prefix_name + "velocity_measurements"};
}
