/*
 * Copyright (C) 2022 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#ifndef ROFT_SKFCORRECTION_H
#define ROFT_SKFCORRECTION_H

#include <BayesFilters/AdditiveMeasurementModel.h>
#include <BayesFilters/GaussianCorrection.h>
#include <BayesFilters/GaussianMixture.h>
#include <BayesFilters/LinearMeasurementModel.h>

#include <Eigen/Dense>

namespace ROFT {
    class SKFCorrection;
}


class ROFT::SKFCorrection : public bfl::GaussianCorrection
{
public:
    SKFCorrection(std::unique_ptr<bfl::LinearMeasurementModel> measurement_model, const std::size_t measurement_sub_size, const bool use_laplacian_reweighting = false);

    virtual ~SKFCorrection();

    bfl::MeasurementModel& getMeasurementModel() override;

protected:
    void correctStep(const bfl::GaussianMixture& pred_state, bfl::GaussianMixture& corr_state) override;

private:
    std::unique_ptr<bfl::LinearMeasurementModel> measurement_model_;

    /* Eigen::MatrixXd measurement_; */

    /* Eigen::MatrixXd propagated_sigma_points_; */

    /* Eigen::MatrixXd innovations_; */

    /**
     * Unscented transform weight.
     */
    std::size_t measurement_sub_size_;

    bool use_laplacian_reweighting_;

    const std::string log_name_ = "SKFCorrection";
};

#endif /* SKFCORRECTION_H */
