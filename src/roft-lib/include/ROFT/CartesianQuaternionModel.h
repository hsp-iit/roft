/*
 * Copyright (C) 2022 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#ifndef ROFT_CARTESIANQUATERNIONMODEL_H
#define ROFT_CARTESIANQUATERNIONMODEL_H

#include <BayesFilters/StateModel.h>
#include <BayesFilters/VectorDescription.h>

#include <Eigen/Dense>

#include <chrono>

namespace ROFT {
    class CartesianQuaternionModel;
}


class ROFT::CartesianQuaternionModel : public bfl::StateModel
{
public:
    CartesianQuaternionModel(const Eigen::Ref<const Eigen::MatrixXd> psd_linear_acceleration, const Eigen::Ref<const Eigen::MatrixXd> sigma_angular_velocity, const double sample_time);

    virtual ~CartesianQuaternionModel();

    virtual void propagate(const Eigen::Ref<const Eigen::MatrixXd>& cur_states, Eigen::Ref<Eigen::MatrixXd> mot_states) override;

    virtual void motion(const Eigen::Ref<const Eigen::MatrixXd>& cur_states, Eigen::Ref<Eigen::MatrixXd> mot_states) override;

    bool setSamplingTime(const double& sample_time) override;

    bool setProperty(const std::string& property) override;

    Eigen::MatrixXd getNoiseCovarianceMatrix() override;

    bfl::VectorDescription getInputDescription() override;

    bfl::VectorDescription getStateDescription() override;

private:
    void evaluate_noise_covariance_matrix(const double& T);

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
     * Squared power spectral density for the linear acceleration part
     */
    Eigen::MatrixXd psd_linear_acceleration_;

    /**
     * Variance for the angular velocity part
     */
    Eigen::MatrixXd sigma_angular_velocity_;

    /**
     * Sample time estimation
     */
    std::chrono::steady_clock::time_point last_time_;

    double sample_time_;

    bool last_time_set_ = false;
};

#endif /* ROFT_CARTESIANQUATERNIONMODEL_H */
