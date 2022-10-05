/*
 * Copyright (C) 2016-2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * BSD 3-Clause license. See the accompanying LICENSE file for details.
 */

#ifndef ROFT_UKFCORRECTION_H
#define ROFT_UKFCORRECTION_H

#include <BayesFilters/AdditiveMeasurementModel.h>
#include <BayesFilters/GaussianCorrection.h>
#include <BayesFilters/GaussianMixture.h>
#include <BayesFilters/MeasurementModel.h>
#include <BayesFilters/sigma_point.h>

#include <Eigen/Dense>

namespace ROFT {
    class UKFCorrection;
}


class ROFT::UKFCorrection : public bfl::GaussianCorrection
{
public:
    UKFCorrection(std::unique_ptr<bfl::MeasurementModel> meas_model, const double alpha, const double beta, const double kappa) noexcept;

    UKFCorrection(UKFCorrection&& ukf_prediction) noexcept;

    virtual ~UKFCorrection() noexcept = default;

    bfl::MeasurementModel& getMeasurementModel() override;

    std::pair<bool, Eigen::VectorXd> getLikelihood() override;

protected:
    void correctStep(const bfl::GaussianMixture& pred_state, bfl::GaussianMixture& corr_state) override;

    std::unique_ptr<bfl::MeasurementModel> measurement_model_;

    /**
     * Unscented transform weight.
     */
    bfl::sigma_point::UTWeight ut_weight_;

    const double ut_alpha_;

    const double ut_beta_;

    const double ut_kappa_;
};

#endif /* ROFT_UKFCORRECTION_H */
