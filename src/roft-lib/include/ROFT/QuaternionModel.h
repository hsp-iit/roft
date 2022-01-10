/*
 * Copyright (C) 2022 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#ifndef ROFT_QUATERNIONMODEL_H
#define ROFT_QUATERNIONMODEL_H

#include <BayesFilters/StateModel.h>
#include <BayesFilters/VectorDescription.h>

#include <Eigen/Dense>

#include <chrono>

namespace ROFT {
    class QuaternionModel;
}


class ROFT::QuaternionModel : public bfl::StateModel
{
public:
    QuaternionModel(const Eigen::Ref<const Eigen::MatrixXd> sigma_angular_velocity, const double sample_time);

    virtual ~QuaternionModel();

    virtual void propagate(const Eigen::Ref<const Eigen::MatrixXd>& cur_states, Eigen::Ref<Eigen::MatrixXd> mot_states) override;

    virtual void motion(const Eigen::Ref<const Eigen::MatrixXd>& cur_states, Eigen::Ref<Eigen::MatrixXd> mot_states) override;

    Eigen::MatrixXd getNoiseCovarianceMatrix() const override;

    bfl::VectorDescription getInputDescription() const override;

    bfl::VectorDescription getStateDescription() const override;

    bool setProperty(const std::string& property);

    void setSamplingTime(const double& sample_time) override;

private:
    Eigen::MatrixXd propagate_quaternion(const Eigen::Ref<const Eigen::MatrixXd>& current_quaternion, const Eigen::Ref<const Eigen::MatrixXd>& angular_velocities);

    /**
     * Input and state descriptions.
     */
    bfl::VectorDescription input_description_;

    bfl::VectorDescription state_description_;

    /**
     * Noise covariance matrix.
     */
    Eigen::MatrixXd Q_;

    /**
     * Variance for the angular velocity part
     */
    Eigen::MatrixXd sigma_angular_velocity_;

    double sample_time_;
};

#endif /* ROFT_QUATERNIONMODEL_H */
