/*
 * Copyright (C) 2022 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#include <ROFT/CartesianQuaternionModel.h>

#include <Eigen/Dense>

using namespace Eigen;
using namespace ROFT;
using namespace bfl;


CartesianQuaternionModel::CartesianQuaternionModel
(
    const Eigen::Ref<const Eigen::MatrixXd> psd_linear_acceleration,
    const Eigen::Ref<const Eigen::MatrixXd> sigma_angular_velocity,
    const double sample_time
) :
    psd_linear_acceleration_(psd_linear_acceleration),
    sigma_angular_velocity_(sigma_angular_velocity),
    sample_time_(sample_time)
{
    /* Initialize the noise covariance matrix. */
    Q_.resize(9, 9);
    Q_ = MatrixXd::Zero(9, 9);

    /* Evaluate the noise matrix Q_ using a default sampling time. */
    evaluate_noise_covariance_matrix(sample_time_);

    /* Set the state description: 9 linear components (x_dot, y_dot, z_dot, w_x, w_y, w_z, x, y, z) and 1 quaternion. */
    state_description_ = VectorDescription(9, 1, 0, VectorDescription::CircularType::Quaternion);

    /* Set the input description: as the state description plus noise components for x_dot, y_dot, z_dot, w_x, w_y, w_z, x, y, z. */
    input_description_ = VectorDescription(9, 1, 9, VectorDescription::CircularType::Quaternion);
}


CartesianQuaternionModel::~CartesianQuaternionModel()
{ }


void CartesianQuaternionModel::propagate(const Eigen::Ref<const Eigen::MatrixXd>& cur_states, Eigen::Ref<Eigen::MatrixXd> mot_states)
{
    /* Same implementation as CartesianQuaternionModel::motion() but noise-free. */

    /* Except for the quaternionic part, the update can be done using matrix operations. */

    /* Set previous state. */
    mot_states.topRows<9>() = cur_states.topRows<9>();

    /* Add integral of linear velocity to the Cartesian position. */
    mot_states.middleRows<3>(6) += cur_states.topRows<3>() * sample_time_;

    /* Cycle over all the components for the quaternionic part. */
    for (std::size_t i = 0; i < cur_states.cols(); i++)
    {
        /* Take the angular velocity. */
        const Ref<const Vector3d> w = cur_states.col(i).segment<3>(3);

        /* Angular velocity norm.

         std::numeric_limits<double>::epsilon() used to extend by continuity sin(x)/x when x-> 0. */
        double norm_w = w.norm() + std::numeric_limits<double>::epsilon();

        /* Left-quaternion-product matrix of the augmented angular velocity quaternion *written in the inertial reference frame*. */
        Matrix4d skew_w = Matrix4d::Zero();
        skew_w.block<1, 3>(0, 1) = -1.0 * w.transpose();
        skew_w.block<3, 1>(1, 0) = w;
        /* The following should have a reversed sign if the angular velocity was *written in the body reference frame*. */
        skew_w(1, 2) = -w(2);
        skew_w(1, 3) = w(1);
        skew_w(2, 1) = w(2);
        skew_w(2, 3) = -w(0);
        skew_w(3, 1) = -w(1);
        skew_w(3, 2) = w(0);

        mot_states.col(i).tail<4>() = (std::cos(norm_w * sample_time_ / 2.0) * Matrix4d::Identity() + sin(norm_w * sample_time_ / 2.0) / norm_w * skew_w) * cur_states.col(i).segment<4>(9);
    }
}


void CartesianQuaternionModel::motion(const Eigen::Ref<const Eigen::MatrixXd>& cur_states, Eigen::Ref<Eigen::MatrixXd> mot_states)
{
    /* Warning. This a bfl::StateModel, i.e. the most general form of state model.
       For this reason, it is assumed that Q_.rows() noise components are provided in cur_states.bottomRows(Q_.cols()). */

    /* Except for the quaternionic part, the update can be done using matrix operations. */

    /* Set previous state and noise. */
    mot_states.topRows<9>() = cur_states.topRows<9>() + cur_states.bottomRows<9>();

    /* Add integral of linear velocity to the Cartesian position. */
    mot_states.middleRows<3>(6) += cur_states.topRows<3>() * sample_time_;

    /* Cycle over all the components for the quaternionic part. */
    for (std::size_t i = 0; i < cur_states.cols(); i++)
    {
        /* Take the previous angular velocity without the current noise sample. */
        const Ref<const Vector3d> w = cur_states.col(i).segment<3>(3);

        /* Angular velocity norm.

         std::numeric_limits<double>::epsilon() used to extend by continuity sin(x)/x when x-> 0. */
        double norm_w = w.norm() + std::numeric_limits<double>::epsilon();

        /* Left-quaternion-product matrix of the augmented angular velocity quaternion *written in the inertial reference frame*. */
        Matrix4d skew_w = Matrix4d::Zero();
        skew_w.block<1, 3>(0, 1) = -1.0 * w.transpose();
        skew_w.block<3, 1>(1, 0) = w;
        /* The following should have a reversed sign if the angular velocity was *written in the body reference frame*. */
        skew_w(1, 2) = -w(2);
        skew_w(1, 3) = w(1);
        skew_w(2, 1) = w(2);
        skew_w(2, 3) = -w(0);
        skew_w(3, 1) = -w(1);
        skew_w(3, 2) = w(0);

        mot_states.col(i).tail<4>() = (std::cos(norm_w * sample_time_ / 2.0) * Matrix4d::Identity() + sin(norm_w * sample_time_ / 2.0) / norm_w * skew_w) * cur_states.col(i).segment<4>(9);
    }
}


void CartesianQuaternionModel::evaluate_noise_covariance_matrix(const double& T)
{
    /* Compose noise covariance matrix for the linear velocity part. */
    Q_.block<3, 3>(0, 0) = psd_linear_acceleration_ * T;

    /* Compose noise covariance matrix for the angular velocity part. */
    Q_.block<3, 3>(3, 3) = sigma_angular_velocity_;

    /* Compose noise covariance matrix for the Cartesian position part. */
    Q_.block<3, 3>(6, 6) = psd_linear_acceleration_ * (std::pow(T, 3.0) / 3.0);

    /* Compose noise cross-covariance matrix. */
    Q_.block<3, 3>(0, 6) = psd_linear_acceleration_ * (std::pow(T, 2.0) / 2.0);
    Q_.block<3, 3>(6, 0) = psd_linear_acceleration_ * (std::pow(T, 2.0) / 2.0);
}


VectorDescription CartesianQuaternionModel::getInputDescription() const
{
    return input_description_;
}


VectorDescription CartesianQuaternionModel::getStateDescription() const
{
    /* 9 linear components (x_dot, y_dot, z_dot, w_x, w_y, w_z, x, y, z) and 1 quaternion. */
    return state_description_;
}


Eigen::MatrixXd CartesianQuaternionModel::getNoiseCovarianceMatrix() const
{
    return Q_;
}


void CartesianQuaternionModel::setSamplingTime(const double& sample_time)
{
   sample_time_ = sample_time;
   evaluate_noise_covariance_matrix(sample_time_);
}


bool CartesianQuaternionModel::setProperty(const std::string& property)
{
    return false;
}
