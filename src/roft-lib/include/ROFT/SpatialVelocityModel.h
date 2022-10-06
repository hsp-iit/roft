/*
 * Copyright (C) 2022 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#ifndef ROFT_SPATIALVELOCITYMODEL_H
#define ROFT_SPATIALVELOCITYMODEL_H

#include <BayesFilters/LinearStateModel.h>

#include <Eigen/Dense>

namespace ROFT {
    class SpatialVelocityModel;
}


class ROFT::SpatialVelocityModel : public bfl::LinearStateModel
{
public:
    SpatialVelocityModel(const Eigen::Ref<const Eigen::MatrixXd> sigma_v, const Eigen::Ref<const Eigen::MatrixXd> sigma_w);

    virtual ~SpatialVelocityModel();

    Eigen::MatrixXd getStateTransitionMatrix() override;

    Eigen::MatrixXd getNoiseCovarianceMatrix() override;

    bfl::VectorDescription getInputDescription() override;

    bfl::VectorDescription getStateDescription() override;

    bool setProperty(const std::string& property) override;

protected:
   /**
     * Input and state descriptions.
     */
    bfl::VectorDescription input_description_;

    bfl::VectorDescription state_description_;

    /**
     * State transition matrix.
     */
    Eigen::MatrixXd F_;

    /**
     * Noise covariance matrix.
     */
    Eigen::MatrixXd Q_;

    const std::string log_name_ = "SpatialVelocityModel";
};

#endif /* ROFT_SPATIALVELOCITYMODEL_H */
