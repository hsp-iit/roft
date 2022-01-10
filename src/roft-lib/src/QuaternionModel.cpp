/*
 * Copyright (C) 2022 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#include <ROFT/QuaternionModel.h>

#include <Eigen/Dense>

using namespace Eigen;
using namespace ROFT;
using namespace bfl;


QuaternionModel::QuaternionModel(const Eigen::Ref<const Eigen::MatrixXd> sigma_angular_velocity, const double sample_time) :
    sigma_angular_velocity_(sigma_angular_velocity),
    sample_time_(sample_time)
{
    /* Initialize the noise covariance matrix. */
    Q_.resize(3, 3);
    Q_ = sigma_angular_velocity;

    /* Set the state description: 3 linear components (w_x, w_y, w_z) and 1 quaternion. */
    state_description_ = VectorDescription(3, 1, 0, VectorDescription::CircularType::Quaternion);

    /* Set the input description: as the state description plus noise components for w_x, w_y, w_z. */
    input_description_ = VectorDescription(3, 1, 3, VectorDescription::CircularType::Quaternion);
}


QuaternionModel::~QuaternionModel()
{ }


void QuaternionModel::motion(const Ref<const MatrixXd>& current, Ref<MatrixXd> propagated)
{
    /* Warning. This class is bfl::StateModel, i.e. the most general form of state model.
       For this reason, it is assumed that Q_.rows() = 3 noise components are provided in current.bottomRows(Q_.cols()). */

    propagated.topRows<3>() = current.topRows<3>() + current.bottomRows<3>();
    propagated.bottomRows<4>() = propagate_quaternion(current.middleRows<4>(3), current.topRows<3>());

}

void QuaternionModel::propagate(const Ref<const MatrixXd>& current, Ref<MatrixXd> propagated)
{
    /* Same implementation as QuaternionModel::motion() but noise-free. */

    propagated.topRows<3>() = current.topRows<3>();
    propagated.bottomRows<4>() = propagate_quaternion(current.middleRows<4>(3), current.topRows<3>());
}


VectorDescription QuaternionModel::getInputDescription() const
{
    return input_description_;
}


VectorDescription QuaternionModel::getStateDescription() const
{
    return state_description_;
}


Eigen::MatrixXd QuaternionModel::getNoiseCovarianceMatrix() const
{
    return Q_;
}


bool QuaternionModel::setProperty(const std::string& property)
{
    return false;
}


void QuaternionModel::setSamplingTime(const double& sample_time)
{
    sample_time_ = sample_time;
}


MatrixXd QuaternionModel::propagate_quaternion(const Ref<const MatrixXd>& current_quaternion, const Ref<const MatrixXd>& angular_velocities)
{
    MatrixXd propagated(4, current_quaternion.cols());

    for (std::size_t i = 0; i < current_quaternion.cols(); i++)
    {
        /* Take the angular velocity. */
        const Ref<const Vector3d> w = angular_velocities.col(i);

        /* Angular velocity norm.

         std::numeric_limits<double>::min() used to properly evaluate sin(x)/x when x-> 0. */
        double norm_w = w.norm() + std::numeric_limits<double>::min();

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

        propagated.col(i) = (std::cos(norm_w * sample_time_ / 2.0) * Matrix4d::Identity() + sin(norm_w * sample_time_ / 2.0) / norm_w * skew_w) * current_quaternion.col(i);
    }

    return propagated;
}
