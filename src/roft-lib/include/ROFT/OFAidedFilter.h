/*
 * Copyright (C) 2022 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#ifndef ROFT_OFAIDEDFILTER_H
#define ROFT_OFAIDEDFILTER_H

#include <BayesFilters/FilteringAlgorithm.h>
#include <BayesFilters/Gaussian.h>
#include <BayesFilters/GaussianCorrection.h>
#include <BayesFilters/GaussianPrediction.h>

#include <Eigen/Dense>

#include <ROFT/CameraMeasurement.h>
#include <ROFT/ImageOpticalFlowMeasurement.hpp>
#include <ROFT/ImageOpticalFlowSource.h>
#include <ROFT/ImageSegmentationMeasurement.h>
#include <ROFT/ImageSegmentationSource.h>
#include <ROFT/ModelParameters.h>

#include <RobotsIO/Camera/Camera.h>
#include <RobotsIO/Utils/ProbeContainer.h>
#include <RobotsIO/Utils/SpatialVelocityBuffer.h>
#include <RobotsIO/Utils/Transform.h>

#include <SuperimposeMesh/SICAD.h>

#include <opencv2/opencv.hpp>

namespace ROFT {
    class OFAidedFilter;
}


class ROFT::OFAidedFilter : public bfl::FilteringAlgorithm,
                           public RobotsIO::Utils::ProbeContainer
{
public:
    OFAidedFilter
    (
        std::shared_ptr<ROFT::CameraMeasurement> camera_measurement,
        std::shared_ptr<ROFT::ImageSegmentationSource> segmentation_source,
        std::shared_ptr<ROFT::ImageOpticalFlowSource> flow_source,
        std::shared_ptr<RobotsIO::Utils::Transform> pose_measurement,
        const ModelParameters& model_parameters,
        const Eigen::Ref<const Eigen::VectorXd>& initial_condition_p,
        const Eigen::Ref<const Eigen::VectorXd>& initial_covariance_p,
        const Eigen::Ref<const Eigen::VectorXd>& model_covariance_p,
        const Eigen::Ref<const Eigen::VectorXd>& measurement_covariance_p,
        const Eigen::Ref<const Eigen::VectorXd> initial_condition_v,
        const Eigen::Ref<const Eigen::VectorXd> initial_covariance_v,
        const Eigen::Ref<const Eigen::VectorXd> model_covariance_v,
        const Eigen::Ref<const Eigen::VectorXd> measurement_covariance_v,
        const double& ut_alpha,
        const double& ut_beta,
        const double& ut_kappa,
        const double& sample_time,
        const bool pose_meas,
        const bool pose_resync,
        const bool pose_outlier_rejection,
        const bool pose_outlier_rejection_gain,
        const bool velocity_meas,
        const bool flow_weighting,
        const bool flow_aided_segmentation,
        const double& maximum_depth,
        const double& subsampling_radius,
        const bool enable_log,
        const std::string& log_path,
        const std::string& log_prefix
    );

    virtual ~OFAidedFilter();

    bool runCondition() override;

    bool initialization() override;

    bool skip(const std::string& what_step, const bool status) override;

protected:
    std::vector<std::string> log_file_names(const std::string& prefix_path, const std::string& prefix_name) override;

    void filteringStep() override;

private:
    void start_time_count();

    double stop_time_count();

    std::pair<bool, bfl::Gaussian> pick_best_alternative(const std::vector<bfl::Gaussian>& alternatives, const bool use_buffered_features);

    bool buffer_outlier_rejection_features();

    bfl::Gaussian correct_outlier_rejection(const bfl::Gaussian& prediction, const bool use_buffered_features);

    /* Initial conditions for flow aided velocity estimation. */

    Eigen::VectorXd v_v_0_;

    Eigen::VectorXd v_w_0_;

    Eigen::MatrixXd v_covariance_0_;

    /* Initial conditions for pose tracking .*/

    Eigen::VectorXd p_v_0_;

    Eigen::VectorXd p_w_0_;

    Eigen::VectorXd p_x_0_;

    Eigen::VectorXd p_q_0_;

    Eigen::MatrixXd p_covariance_0_;

    /* Measurement models. */

    std::shared_ptr<ROFT::CameraMeasurement> camera_;

    std::shared_ptr<ROFT::ImageSegmentationMeasurement> segmentation_;

    std::shared_ptr<RobotsIO::Utils::SpatialVelocityBuffer> velocity_;

    const double maximum_depth_;

    /* Prediction and correction. */

    std::unique_ptr<bfl::GaussianPrediction> p_prediction_;

    std::unique_ptr<bfl::GaussianCorrection> p_correction_;

    std::unique_ptr<bfl::GaussianPrediction> v_prediction_;

    std::unique_ptr<bfl::GaussianCorrection> v_correction_;

    /* Beliefs. */

    bfl::Gaussian p_pred_belief_;

    bfl::Gaussian p_corr_belief_;

    bfl::Gaussian v_pred_belief_;

    bfl::Gaussian v_corr_belief_;

    /* Depth measurement and rendering for outlier rejection .*/

    RobotsIO::Camera::CameraParameters camera_parameters_;

    std::unique_ptr<SICAD> renderer_;

    const bool outlier_rejection_;

    const double outlier_rejection_gain_;

    int divider_ = 4;

    /* Pose re-synchronization related. */

    bfl::Gaussian buffered_belief_;

    Eigen::MatrixXf buffered_depth_;

    cv::Mat buffered_segmentation_;

    const bool pose_resync_;

    bool outlier_rejection_features_initialized_ = false;

    /* Timing. */

    std::chrono::steady_clock::time_point std_time_0_;

    double last_camera_stamp_ = -1;

    const double sample_time_;

    ImageOpticalFlowMeasurement<cv::Vec2f>* flow_meas_alt_f_ = nullptr;
    ImageOpticalFlowMeasurement<cv::Vec2s>* flow_meas_alt_s_ = nullptr;

    /* Segmentation debugging. */
    std::shared_ptr<ROFT::ImageSegmentationSource> segmentation_debug_;

    cv::Mat last_mask_;

    /* Log name. */

    const std::string log_name_ = "OFAidedFilter";
};

#endif /* ROFT_OFAIDEDFILTER_H */
