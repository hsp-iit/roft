/*
 * Copyright (C) 2022 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#include <ROFT/SpatialVelocityModel.h>

using namespace Eigen;
using namespace ROFT;
using namespace bfl;


SpatialVelocityModel::SpatialVelocityModel(const Ref<const MatrixXd> sigma_v, const Ref<const MatrixXd> sigma_w)
{
    F_ = MatrixXd::Identity(6, 6);

    Q_.resize(6, 6);
    Q_ = MatrixXd::Zero(6, 6);

    Q_.block<3, 3>(0, 0) = sigma_v;
    Q_.block<3, 3>(3, 3) = sigma_w;

    input_description_ = VectorDescription(6, 0, 6);
    state_description_ = input_description_.noiseless_description();
}


SpatialVelocityModel::~SpatialVelocityModel()
{}


Eigen::MatrixXd SpatialVelocityModel::getStateTransitionMatrix()
{
    return F_;
}


Eigen::MatrixXd SpatialVelocityModel::getNoiseCovarianceMatrix()
{
    return Q_;
}


VectorDescription SpatialVelocityModel::getInputDescription()
{
    return input_description_;
}


VectorDescription SpatialVelocityModel::getStateDescription()
{
    return state_description_;
}


bool SpatialVelocityModel::setProperty(const std::string& property)
{
    return true;
}
