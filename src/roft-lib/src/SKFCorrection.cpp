/*
 * Copyright (C) 2022 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#include <ROFT/SKFCorrection.h>

using namespace bfl;
using namespace Eigen;
using namespace ROFT;


SKFCorrection::SKFCorrection
(
    std::unique_ptr<LinearMeasurementModel> measurement_model,
    const size_t measurement_sub_size,
    const bool use_laplacian_reweighting
) :
    measurement_model_(std::move(measurement_model)),
    measurement_sub_size_(measurement_sub_size),
    use_laplacian_reweighting_(use_laplacian_reweighting)
{ }


SKFCorrection::~SKFCorrection()
{ }


MeasurementModel& SKFCorrection::getMeasurementModel()
{
    return *measurement_model_;
}


void SKFCorrection::correctStep(const GaussianMixture& pred_state, GaussianMixture& corr_state)
{
    /* WARNING: this implementation assumes that pred_state.components == 1. */

    /* Evaluate predicted measure. */
    Data prediction;
    bool valid_prediction;
    std::tie(valid_prediction, prediction) = measurement_model_->predictedMeasure(pred_state.mean());

    if (!valid_prediction)
    {
        std::cout << log_name_ << ":correctStep. Error: cannot predicted measurement." << std::endl;
        corr_state = pred_state;
        return;
    }

    /* Get measurement. */
    Data measurement;
    bool valid_measurement;
    std::tie(valid_measurement, measurement) = measurement_model_->measure();

    /* Check if the size of the measurement is not zero and compatible with measurement_sub_size. */
    VectorDescription description = measurement_model_->getMeasurementDescription();
    std::size_t meas_size = description.linear_size;
    valid_measurement &= (meas_size != 0);
    valid_measurement &= ((meas_size % measurement_sub_size_ ) == 0);

    if (!valid_measurement)
    {
        std::cout << log_name_ << ":correctStep. Error: measurement is empty or size not a multiple of " << measurement_sub_size_ << std::endl;
        corr_state = pred_state;
        return;
    }

    /* Evaluate the innovation. */
    bool valid_innovation;
    Data innovation;
    std::tie(valid_innovation, innovation) = measurement_model_->innovation(prediction, measurement);

    if (!valid_innovation)
    {
        std::cout << log_name_ << ":correctStep. Error: cannot evaluate innovation " << std::endl;
        corr_state = pred_state;
        return;
    }

    /* Cast innovations once for all. */
    MatrixXd innovations = any::any_cast<MatrixXd&&>(std::move(innovation));

    /* Cast measurement once for all. */
    MatrixXd measure = any::any_cast<MatrixXd&&>(std::move(measurement));

    /* Fit innovations to a Laplacian in order to weight measurements. */
    VectorXd likelihoods;
    if (use_laplacian_reweighting_)
    {
        MatrixXd innovation_vector = Map<MatrixXd>(innovations.col(0).head(innovations.rows()).data(), innovations.rows() / measurement_sub_size_, measurement_sub_size_);
        VectorXd norms = innovation_vector.rowwise().norm();
        std::sort(norms.data(), norms.data() + norms.size());

        VectorXd laplacian_mi(1);
        laplacian_mi(0) = norms(norms.size() / 2);
        if ((norms.size() % 2) == 0)
            laplacian_mi(0) = 0.5 * (norms(norms.size() / 2 - 1) + norms(norms.size() / 2));

        double laplacian_b = ((norms.rowwise() - laplacian_mi.transpose()).array().abs().matrix()).sum() / norms.size();

        likelihoods = VectorXd::Ones(norms.size());

        if (laplacian_b > 1e-4)
        {
            for (std::size_t j = 0; j < norms.size(); j++)
            {
                /* Avoid zero likelihoods, as these must be invertible.*/
                likelihoods(j) = std::max(1.0 / (2 * laplacian_b) * std::exp(-std::abs(innovations.col(0).segment(j * measurement_sub_size_, measurement_sub_size_).norm() - laplacian_mi(0)) / laplacian_b), 1e-6);
            }

            likelihoods /= likelihoods.maxCoeff();
        }
    }

    /* Get noise covariance matrix. */
    MatrixXd R;
    std::tie(std::ignore, R) = measurement_model_->getNoiseCovarianceMatrix();

    /* Get measurement matrix. */
    MatrixXd H = measurement_model_->getMeasurementMatrix();

    VectorXd mean_j = pred_state.mean();
    MatrixXd covariance_j = pred_state.covariance();

    /* Perform sequential correction. */
    for (size_t j = 0; j < int(meas_size / measurement_sub_size_); j++)
    {
        // Evaluate the measurement covariance matrix
        //    Py = H * Px * H' + R
        MatrixXd R_j = R;
        if (use_laplacian_reweighting_)
            R_j /= likelihoods(j);
        MatrixXd Py = H.middleRows(measurement_sub_size_ * j, measurement_sub_size_) * covariance_j * H.middleRows(measurement_sub_size_  * j, measurement_sub_size_).transpose() + R_j;

        // Evaluate the Kalman Gain
        //    K = Px * H' * (Py)^{-1}
        MatrixXd K = covariance_j * H.middleRows(measurement_sub_size_ * j, measurement_sub_size_).transpose() * Py.inverse();

        // Evaluate the filtered mean
        //    x_{k}+ = x{k}- + K * (y - y_predicted)
        mean_j += K * (measure.col(0).segment(measurement_sub_size_ * j, measurement_sub_size_) - H.middleRows(measurement_sub_size_ * j, measurement_sub_size_) * mean_j);

        // Evaluate the filtered covariance
        //    P_{k}+ = P_{k}- - K * Py * K'
        covariance_j = (MatrixXd::Identity(6, 6) - K * H.middleRows(measurement_sub_size_ * j, measurement_sub_size_)) * covariance_j;
    }

    corr_state.mean() = mean_j;
    corr_state.covariance() = covariance_j;
}
