/*
 * Copyright (C) 2016-2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * BSD 3-Clause license. See the accompanying LICENSE file for details.
 */

#include <ROFT/UKFCorrection.h>
#include <BayesFilters/utils.h>

using namespace bfl;
using namespace bfl::sigma_point;
using namespace bfl::utils;
using namespace Eigen;
using namespace ROFT;


UKFCorrection::UKFCorrection
(
    std::unique_ptr<MeasurementModel> measurement_model,
    const double alpha,
    const double beta,
    const double kappa
) noexcept :
    measurement_model_(std::move(measurement_model)),
    ut_weight_(measurement_model_->getInputDescription(), alpha, beta, kappa),
    ut_alpha_(alpha),
    ut_beta_(beta),
    ut_kappa_(kappa)
{ }


UKFCorrection::UKFCorrection(UKFCorrection&& ukf_correction) noexcept :
    measurement_model_(std::move(ukf_correction.measurement_model_)),
    ut_weight_(ukf_correction.ut_weight_),
    ut_alpha_(ukf_correction.ut_alpha_),
    ut_beta_(ukf_correction.ut_beta_),
    ut_kappa_(ukf_correction.ut_kappa_)
{ }


MeasurementModel& UKFCorrection::getMeasurementModel()
{
    return *measurement_model_;
}


std::pair<bool, VectorXd> UKFCorrection::getLikelihood()
{
    throw(std::runtime_error("Error: ROFT::UKFCorrection::getLikelihood() is not implemented."));
}


void UKFCorrection::correctStep(const GaussianMixture& pred_state, GaussianMixture& corr_state)
{
    /* Pick the correct measurement model. */
    MeasurementModel& model = getMeasurementModel();

    /* Get the current measurement if available. */
    bool valid_measurement;
    Data measurement;
    std::tie(valid_measurement, measurement) = model.measure();

    if (!valid_measurement)
    {
        corr_state = pred_state;
        return;
    }

    /* Extract measurement size. */
    std::size_t meas_size = model.getMeasurementDescription().total_size();

    /* Evaluate the joint state-measurement statistics, if possible. */
    bool valid = false;
    MatrixXd Pxy;

    /* Augment the previous state using measurement noise statistics. */
    GaussianMixture pred_state_augmented = pred_state;
    MatrixXd noise_covariance_matrix;
    std::tie(std::ignore, noise_covariance_matrix) = model.getNoiseCovarianceMatrix();
    pred_state_augmented.augmentWithNoise(noise_covariance_matrix);

    /* Update the UT weights. */
    ut_weight_ = UTWeight(measurement_model_->getInputDescription(), ut_alpha_, ut_beta_, ut_kappa_);

    /* Predict the measurements. */
    GaussianMixture predicted_meas;
    std::tie(valid, predicted_meas, Pxy) = sigma_point::unscented_transform(pred_state_augmented, ut_weight_, *measurement_model_);
    if (!valid)
    {
        corr_state = pred_state;
        return;
    }

    /* Evaluate the innovation if possible. */
    bool valid_innovation;
    Data innovation;
    /* This temporary is required since some MeasurementModel::innovation methods may try to cast from
       const Ref<const MatrixXd> to MatrixXd resulting in a bfl::any::bad_any_cast.

       Hopefully, using std::move, it is possible to steal the memory from predicted_meas.mean(). */
    MatrixXd y_p = std::move(predicted_meas.mean());
    std::tie(valid_innovation, innovation) = model.innovation(y_p, measurement);

    if (!valid_innovation)
    {
        corr_state = pred_state;
        return;
    }

    /* Cast innovations once for all. */
    MatrixXd innovations = any::any_cast<MatrixXd&&>(std::move(innovation));

    /* Our mixture only has one component. */

    /* Evaluate the Kalman Gain
       K = Pxy * (Py)^{-1} */
    MatrixXd K = Pxy * predicted_meas.covariance().inverse();

    /* Evaluate the K * innovation product once for all. */
    MatrixXd K_innovations = K * innovations;

    /* Evaluate the filtered mean.
       x_{k}+ = x{k}- + K * innovation */
    corr_state.mean().topRows(corr_state.dim_linear) = pred_state.mean().topRows(corr_state.dim_linear) + K_innovations.topRows(corr_state.dim_linear);

    /*  Sum the mean predicted quaternion with rotation vectors resulting from the product between K and the innovations.*/
    corr_state.mean().middleRows(corr_state.dim_linear, 4) = sum_quaternion_rotation_vector(pred_state.mean().middleRows(corr_state.dim_linear, 4), K_innovations.middleRows(corr_state.dim_linear, 3));

    /* Evaluate the filtered covariance
       P_{k}+ = P_{k}- - K * Py * K' */
    corr_state.covariance() = pred_state.covariance() - K * predicted_meas.covariance() * K.transpose();
}
