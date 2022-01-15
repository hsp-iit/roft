/*
 * Copyright (C) 2022 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#include <ROFT/Filter.h>

#include <BayesFilters/AdditiveMeasurementModel.h>
#include <BayesFilters/KFPrediction.h>
#include <BayesFilters/LinearStateModel.h>
#include <BayesFilters/UKFPrediction.h>
#include <BayesFilters/UKFCorrection.h>

#include <ROFT/CartesianQuaternionMeasurement.h>
#include <ROFT/CartesianQuaternionModel.h>
#include <ROFT/ImageSegmentationOFAidedSource.hpp>
#include <ROFT/MeshResource.h>
#include <ROFT/SpatialVelocityModel.h>
#include <ROFT/SKFCorrection.h>

#include <RobotsIO/Camera/CameraParameters.h>

using namespace Eigen;
using namespace ROFT;
using namespace RobotsIO::Camera;
using namespace RobotsIO::Utils;
using namespace bfl;


Filter::Filter
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
) :
    p_pred_belief_(9, 1, true),
    p_corr_belief_(9, 1, true),
    v_pred_belief_(6, 0, false),
    v_corr_belief_(6, 0, false),
    camera_(camera_measurement),
    sample_time_(sample_time),
    maximum_depth_(maximum_depth),
    outlier_rejection_(pose_outlier_rejection),
    outlier_rejection_gain_(pose_outlier_rejection_gain),
    pose_resync_(pose_resync)
{
    /* Extract initial conditions. */
    p_v_0_ = initial_condition_p.head<3>();
    p_w_0_ = initial_condition_p.segment<3>(3);
    p_x_0_ = initial_condition_p.segment<3>(6);
    p_q_0_ = initial_condition_p.tail<4>();

    v_v_0_ = initial_condition_v.head<3>();
    v_w_0_ = initial_condition_v.tail<3>();

    p_covariance_0_ = initial_covariance_p.asDiagonal();

    v_covariance_0_ = initial_covariance_v.asDiagonal();

    /* Extract model power spectral densities and variances .*/
    MatrixXd model_sigma_angular_velocity = model_covariance_p.head<3>().asDiagonal();
    MatrixXd model_psd_linear_acceleration = model_covariance_p.tail<3>().asDiagonal();

    MatrixXd v_cov_v = model_covariance_v.head<3>().asDiagonal();
    MatrixXd v_cov_w = model_covariance_v.tail<3>().asDiagonal();

    /* Extract measurement covariance .*/
    MatrixXd meas_sigma_linear_velocity = measurement_covariance_p.head<3>().asDiagonal();
    MatrixXd meas_sigma_angular_velocity = measurement_covariance_p.segment<3>(3).asDiagonal();
    MatrixXd meas_sigma_position = measurement_covariance_p.segment<3>(6).asDiagonal();
    MatrixXd meas_sigma_quaternion = measurement_covariance_p.tail<3>().asDiagonal();

    MatrixXd v_cov_measurement = measurement_covariance_v.asDiagonal();

    /* Kinematic model. */
    auto p_kinematic_model = std::unique_ptr<CartesianQuaternionModel>
    (
        new CartesianQuaternionModel(model_psd_linear_acceleration, model_sigma_angular_velocity, sample_time)
    );

    auto v_kinematic_model = std::unique_ptr<SpatialVelocityModel>
    (
        new SpatialVelocityModel(v_cov_v, v_cov_w)
    );

    /* Save pointer to original segmentation source debugging purposes. */
    segmentation_debug_ = segmentation_source;

    /* Segmentation measurement. */
    if (flow_aided_segmentation)
    {
        if (flow_source->get_matrix_type() == CV_32FC2)
            segmentation_ = std::make_shared<ImageSegmentationMeasurement>(std::make_shared<ImageSegmentationOFAidedSource<cv::Vec2f>>(segmentation_source, flow_source, camera_, 3, true));
        else if (flow_source->get_matrix_type() == CV_16SC2)
            segmentation_ = std::make_shared<ImageSegmentationMeasurement>(std::make_shared<ImageSegmentationOFAidedSource<cv::Vec2s>>(segmentation_source, flow_source, camera_, 3, true));      }
    else
        segmentation_ = std::make_shared<ImageSegmentationMeasurement>(segmentation_source);

    /* Flow measurement. */
    std::unique_ptr<LinearMeasurementModel> flow;
    if (flow_source->get_matrix_type() == CV_32FC2)
    {
        auto tmp_flow = std::unique_ptr<ImageOpticalFlowMeasurement<cv::Vec2f>>
        (
            new ImageOpticalFlowMeasurement<cv::Vec2f>(flow_source, camera_, segmentation_, subsampling_radius, maximum_depth_, v_cov_measurement, false)
        );
        flow_meas_alt_f_ = tmp_flow.get();
        flow = std::move(tmp_flow);
    }
    else if (flow_source->get_matrix_type() == CV_16SC2)
    {
        auto tmp_flow = std::unique_ptr<ImageOpticalFlowMeasurement<cv::Vec2s>>
        (
            new ImageOpticalFlowMeasurement<cv::Vec2s>(flow_source, camera_, segmentation_, subsampling_radius, maximum_depth_, v_cov_measurement, false)
        );
        flow_meas_alt_s_ = tmp_flow.get();
        flow = std::move(tmp_flow);
    }

    /* Velocity measurement. */
    velocity_ = std::make_shared<SpatialVelocityBuffer>();

    /* Pose/Velocity measurement. */
    auto measurement_model = std::unique_ptr<CartesianQuaternionMeasurement>
    (
        new CartesianQuaternionMeasurement(pose_measurement, velocity_, false, pose_meas, velocity_meas, meas_sigma_position, meas_sigma_quaternion, meas_sigma_linear_velocity, meas_sigma_angular_velocity, enable_log)
    );
    if (enable_log)
        measurement_model->enable_log(log_path, log_prefix);

    /* Prediction. */
    p_prediction_ = std::unique_ptr<UKFPrediction>
    (
        new UKFPrediction(std::move(p_kinematic_model), ut_alpha, ut_beta, ut_kappa)
    );

    v_prediction_ = std::unique_ptr<KFPrediction>
    (
        new KFPrediction(std::move(v_kinematic_model))
    );

    /* Correction. */
    p_correction_ = std::unique_ptr<UKFCorrection>
    (
        new UKFCorrection(std::move(measurement_model), ut_alpha, ut_beta, ut_kappa, true)
    );

    v_correction_ = std::unique_ptr<SKFCorrection>
    (
        new SKFCorrection(std::move(flow), 2, flow_weighting)
    );

    /* Depth rendering for outlier rejection. */
    std::tie(std::ignore, camera_parameters_) = camera_->camera_parameters();
    MeshResource mesh_resource(model_parameters);
    std::istringstream mesh_resource_stream(mesh_resource.as_string());
    SICAD::ModelStreamContainer model;
    model["object"] = &mesh_resource_stream;
    std::size_t desired_images = 2;
    divider_ = 4;
    if (camera_parameters_.width() == 640)
        divider_ = 2;
    renderer_ = std::unique_ptr<SICAD>
    (
        new SICAD(model, camera_parameters_.width() / divider_, camera_parameters_.height() / divider_, camera_parameters_.fx() / divider_, camera_parameters_.fy() / divider_, camera_parameters_.cx() / divider_, camera_parameters_.cy() / divider_, desired_images)
    );
    renderer_->setOglToCam({1.0, 0.0, 0.0, static_cast<float>(M_PI)});
    if (renderer_->getTilesNumber() != desired_images)
        throw(std::runtime_error(log_name_ + "::ctor. Depth rendering cannot provide desired_images = " + std::to_string(desired_images) + " images per iteration."));
}


Filter::~Filter()
{
    disable_log();
}


bool Filter::runCondition()
{
    return true;
}


bool Filter::initialization()
{
    /* Initialize Gaussian belief. */
    v_corr_belief_.mean().head<3>() = v_v_0_;
    v_corr_belief_.mean().tail<3>() = v_w_0_;

    v_corr_belief_.covariance() = v_covariance_0_;

    p_corr_belief_.mean().head<3>() = p_v_0_;
    p_corr_belief_.mean().segment<3>(3) = p_w_0_;
    p_corr_belief_.mean().segment<3>(6) = p_x_0_;
    p_corr_belief_.mean().tail<4>() = p_q_0_;

    p_corr_belief_.covariance() = p_covariance_0_;

    buffered_belief_ = p_corr_belief_;

    /* Reset segmentation. */
    segmentation_->reset();

    return true;
}


bool Filter::skip(const std::string& what_step, const bool status)
{
    /* Not implemented. */
    return false;
}


std::vector<std::string> Filter::log_file_names(const std::string& prefix_path, const std::string& prefix_name)
{
    return  {prefix_path + "/" + prefix_name + "pose_estimate",
             prefix_path + "/" + prefix_name + "velocity_estimate",
             prefix_path + "/" + prefix_name + "execution_times"};
}


void Filter::filteringStep()
{
    bool data_in;

    auto time0 = std::chrono::steady_clock::now();
    /* Freeze camera. */
    if (!(data_in = camera_->freeze(CameraMeasurementType::RGBD)))
    {
        std::cout << log_name_ + "::filteringStep. Error: cannot continue without a continuous depth stream" << std::endl;
        teardown();
        return;
    }
    double rgbd_load_time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - time0).count();

    /* Start time count here excluding RGBD loading time. */
    start_time_count();

    /* Evaluate difference between current and previous image time stamp. */
    double elapsed_time = sample_time_;
    double camera_stamp;
    std::tie(std::ignore, camera_stamp) = camera_->camera_time_stamp_rgb();
    if (last_camera_stamp_ != -1)
        elapsed_time = camera_stamp - last_camera_stamp_;
    last_camera_stamp_ = camera_stamp;
    p_prediction_->getStateModel().setSamplingTime(elapsed_time);

    /* Ask for flow source stepping beforehand
       This enables optical flow aided segmentation sources to work in case. */
    v_correction_->getMeasurementModel().freeze(std::make_pair(ImageOpticalFlowMeasurementBase::FreezeType::OnlyStepSource, elapsed_time));

    /* Freeze segmentation. */
    data_in &= segmentation_->freeze();

    /* Freeze flow. */
    data_in &= v_correction_->getMeasurementModel().freeze(std::make_pair(ImageOpticalFlowMeasurementBase::FreezeType::ExceptStepSource, elapsed_time));

    if (data_in)
    {
        /* UKF velocity filtering. */
        Gaussian v_corr_belief_copy = v_corr_belief_;
        v_prediction_->predict(v_corr_belief_, v_pred_belief_);
        v_correction_->correct(v_pred_belief_, v_corr_belief_);
        if (!v_correction_->getMeasurementModel().setProperty("check_observability"))
        {
            std::cout << "The state is unobservable. Skipping this correction." << std::endl;
            v_corr_belief_ = v_corr_belief_copy;
        }
    }

    /* Populate velocity buffer for subsequent filtering steps. */
    velocity_->set_twist(v_corr_belief_.mean().head<3>(), v_corr_belief_.mean().tail<3>());

    /* Get refined segmentation. */
    cv::Mat segmentation_image;
    bfl::Data segmentation_data;
    std::tie(std::ignore, segmentation_data) = segmentation_->measure();
    std::tie(std::ignore, segmentation_image) = any::any_cast<std::pair<bool, cv::Mat>>(segmentation_data);

    if (pose_resync_)
    {
        if (!outlier_rejection_features_initialized_)
        {
            if (!buffer_outlier_rejection_features())
                throw(std::runtime_error(log_name_ + "::filteringStep(). Error: cannot initialize the outlier rejection features."));

            outlier_rejection_features_initialized_ = true;
        }
    }

    /* UKF pose filtering. */
    p_prediction_->predict(p_corr_belief_, p_pred_belief_);

    if (p_correction_->getMeasurementModel().freeze(CartesianQuaternionMeasurement::MeasurementMode::Standard))
    {
        if (p_correction_->getMeasurementModel().getMeasurementDescription().total_size == 13)
        {
            if (pose_resync_)
            {
                /* Copy last buffered belief. */
                Gaussian buffered_belief_copy = buffered_belief_;

                /* Store the corrected belief for the next pose re-sync. */
                buffered_belief_ = p_corr_belief_;

                /* Reset to the buffered belief. */
                p_corr_belief_ = buffered_belief_copy;

                while(p_correction_->getMeasurementModel().freeze(CartesianQuaternionMeasurement::MeasurementMode::PopBufferedMeasurement))
                {
                    p_prediction_->predict(p_corr_belief_, p_pred_belief_);

                    if (outlier_rejection_ && p_correction_->getMeasurementModel().getMeasurementDescription().total_size == 13)
                        p_corr_belief_ = correct_outlier_rejection(p_pred_belief_, true);
                    else
                        p_correction_->correct(p_pred_belief_, p_corr_belief_);
                }

                /* Store the outlier rejection features for the next pose re-sync. */
                buffer_outlier_rejection_features();
            }
            else
            {
                if (outlier_rejection_)
                    p_corr_belief_ = correct_outlier_rejection(p_pred_belief_, false);
                else
                    p_correction_->correct(p_pred_belief_, p_corr_belief_);
            }
        }
        else
            p_correction_->correct(p_pred_belief_, p_corr_belief_);
    }
    else
        p_corr_belief_ = p_pred_belief_;

    /* Stop time count here, as the rest is for debugging purposes only. */
    double exec_time = stop_time_count();

    /* Remove time required to load data from disk, if any. */
    double load_time = 0.0;
    if (flow_meas_alt_f_ != nullptr)
    {
        load_time = flow_meas_alt_f_->get_data_loading_time();
        flow_meas_alt_f_->reset_data_loading_time();
    }
    else
    {
        load_time = flow_meas_alt_s_->get_data_loading_time();
        flow_meas_alt_s_->reset_data_loading_time();
    }
    exec_time -= load_time;

    VectorXd p_mean(13);
    p_mean.head<6>() = p_corr_belief_.mean().head<6>();
    p_mean.segment<3>(6) = p_corr_belief_.mean().segment<3>(6);
    Quaterniond quaternion(p_corr_belief_.mean(9), p_corr_belief_.mean(10), p_corr_belief_.mean(11), p_corr_belief_.mean(12));
    AngleAxisd angle_axis(quaternion);
    p_mean.segment<3>(9) = angle_axis.axis();
    p_mean(12) = angle_axis.angle();

    VectorXd v_mean = v_corr_belief_.mean();

    if (is_probe("output_pose"))
    {
        VectorXd pose = p_mean.tail<7>();
        get_probe("output_pose").set_data(pose);
    }

    if (is_probe("output_velocity"))
        get_probe("output_velocity").set_data(v_mean);

    /* Debug segmentation masks. */
    if (is_probe("output_segmentation") && is_probe("output_segmentation_refined"))
    {
        bool valid = false;


        /* Get current RGB input. */
        bfl::Data camera_data;
        std::tie(valid, camera_data) = camera_->measure();
        cv::Mat rgb;
        cv::Mat rgb_refined;
        std::tie(std::ignore, rgb, std::ignore) = bfl::any::any_cast<CameraMeasurement::CameraMeasurementTuple>(camera_data);
        rgb.copyTo(rgb_refined);

        /* Get current segmentation from the segmentation source. */
        cv::Mat mask;
        std::tie(valid, mask) = segmentation_debug_->segmentation(false);

        /* Draw synchronized segmentation stored in variable segmentation_image. */
        cv::Mat mask_0 = rgb.clone();
        mask_0.setTo(cv::Scalar(0, 255, 0), segmentation_image);

        double alpha = 0.8;
        cv::addWeighted(mask_0, alpha, rgb_refined, 1 - alpha, 0, rgb_refined);

        /* Draw segmentation from the segmentation source. */
        std::vector<std::vector<cv::Point>> contours;

        if (valid)
        {
            cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
            last_mask_ = mask;
        }
        else
            cv::findContours(last_mask_, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        cv::drawContours(rgb, contours, -1, cv::Scalar(0, 0, 255), 4);

        /* Send data to the probe. */
        get_probe("output_segmentation").set_data(rgb);
        get_probe("output_segmentation_refined").set_data(rgb_refined);
    }

    /* Log on file. */
    VectorXd execution_time(2);
    execution_time(0) = exec_time;
    execution_time(1) = load_time + rgbd_load_time;
    logger(p_mean.transpose(), v_mean.transpose(), execution_time.transpose());
}


void Filter::start_time_count()
{
    std_time_0_ = std::chrono::steady_clock::now();
}


double Filter::stop_time_count()
{
    return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - std_time_0_).count();
}


std::pair<bool, bfl::Gaussian> Filter::pick_best_alternative(const std::vector<bfl::Gaussian>& alternatives, const bool use_buffered_features)
{
    auto failed = std::make_pair(false, Gaussian());

    MatrixXf depth;
    cv::Mat segmentation;

    if (use_buffered_features)
    {
        depth = buffered_depth_;
        segmentation = buffered_segmentation_;
    }
    else
    {
        /* Measure depth. */
        bfl::Data camera_data;
        bool valid_data = false;
        std::tie(valid_data, camera_data) = camera_->measure();
        if (!valid_data)
            return failed;

        std::tie(std::ignore, std::ignore, depth) = any::any_cast<std::tuple<Eigen::Transform<double, 3, Affine>, cv::Mat, MatrixXf>>(camera_data);

        /* Measure segmentation. */
        bfl::Data segmentation_data;
        bool valid_segmentation = false;
        std::tie(valid_segmentation, segmentation_data) = segmentation_->measure();
        if (!valid_segmentation)
            return failed;

        std::tie(std::ignore, segmentation) = any::any_cast<std::pair<bool, cv::Mat>>(segmentation_data);
    }

    /* Find non zero coordinates within the segmentation. */
    cv::Mat coordinates;
    cv::findNonZero(segmentation, coordinates);

    /* Render depth. */
    cv::Mat placeholder;
    cv::Mat rendered_depth;

    double cam_x [4] = {0.0, 0.0, 0.0};
    double cam_o [4] = {1.0, 0.0, 0.0, 0.0};

    std::vector<SICAD::ModelPoseContainer> poses;
    for (std::size_t i = 0; i < alternatives.size(); i++)
    {
        const Ref<const VectorXd> alternative = alternatives.at(i).mean();
        SICAD::ModelPose pose;
        pose.push_back(alternative(6));
        pose.push_back(alternative(7));
        pose.push_back(alternative(8));

        AngleAxisd angle_axis(Quaterniond(alternative(9), alternative(10), alternative(11), alternative(12)));
        Vector3d axis = angle_axis.axis();
        pose.push_back(axis(0));
        pose.push_back(axis(1));
        pose.push_back(axis(2));
        pose.push_back(angle_axis.angle());

        SICAD::ModelPoseContainer container;
        container.emplace("object", pose);
        poses.push_back(container);
    }

    bool render_outcome = renderer_->superimpose(poses, cam_x, cam_o, placeholder, rendered_depth);

    cv::Mat mask_float;
    cv::Mat mask;
    cv::threshold(rendered_depth, mask_float, 0.001, 255, cv::THRESH_BINARY);
    mask_float.convertTo(mask, CV_8UC1);

    if (render_outcome)
    {
        VectorXd likelihoods(alternatives.size());
        std::size_t q = 0;
        for (std::size_t i = 0; i < rendered_depth.rows; i += camera_parameters_.height() / divider_)
        {
            for (std::size_t j = 0; j < rendered_depth.cols; j+= camera_parameters_.width() / divider_)
            {
                /* Extract the depth relative to one of the alternatives. */
                cv::Mat depth_alternative = cv::Mat(rendered_depth, cv::Rect(j, i, camera_parameters_.width() / divider_, camera_parameters_.height() / divider_));

                cv::Mat mask_alternative = cv::Mat(mask, cv::Rect(j, i, camera_parameters_.width() / divider_, camera_parameters_.height() / divider_));
                int pixel_in_alternative = cv::countNonZero(mask_alternative);

                /* Evaluate the depth error on the segmentation mask .*/
                double error = 0.0;
                std::size_t samples = 0;
                for (std::size_t k = 0; k < coordinates.total(); k+=2)
                {
                    const int& u = coordinates.at<cv::Point>(k).x;
                    const int& v = coordinates.at<cv::Point>(k).y;

                    if ((depth(v, u) > 0) && (depth(v, u) < 2.0) && (depth_alternative.at<float>(v / divider_, u / divider_) != 0.0))
                    {
                        error += std::abs(depth(v, u) - depth_alternative.at<float>(v / divider_, u / divider_));
                        samples++;
                    }
                }
                error /= samples;

                if (samples == 0)
                {
                    // This might happen if either the depth or the rendered depth are invalid within the current segmentation mask
                    // In this case we assign the biggest number possible here
                    likelihoods(q) = std::numeric_limits<double>::max();
                }
                else
                    likelihoods(q) = error / outlier_rejection_gain_;
                q++;
            }
        }

        std::size_t selected = 0;
        if (likelihoods(0) > 2.0 * likelihoods(1))
            selected = 1;

        // {
        //     std::cout << mask.cols << " " << mask.rows << std::endl;
        //     std::size_t width = camera_parameters_.width() / divider_;
        //     std::size_t height = camera_parameters_.height() / divider_;
        //     std::size_t x = 0;
        //     std::size_t y = 0;
        //     if (selected == 1)
        //         x = width;

        //     cv::Mat mask_best = cv::Mat(mask, cv::Rect(x, y, width, height));

        //     std::vector<std::vector<cv::Point>> contours;
        //     findContours(mask_best, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        //     cv::cvtColor(mask, mask, cv::COLOR_GRAY2BGR);
        //     for (std::size_t i = 0; i < contours.size(); i++)
        //         cv::drawContours(cv::Mat(mask, cv::Rect(x, y, width, height)), contours, i, cv::Scalar(0, 0, 255), 4);

        //     std::vector<std::vector<cv::Point>> contours_segmentation;
        //     cv::resize(segmentation, segmentation, cv::Size(width, height));
        //     findContours(segmentation, contours_segmentation, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        //     for (std::size_t i = 0; i < rendered_depth.rows; i += height)
        //         for (std::size_t j = 0; j < rendered_depth.cols; j+= width)
        //             for (std::size_t k = 0; k < contours_segmentation.size(); k++)
        //                 cv::drawContours(cv::Mat(mask, cv::Rect(j, i, width, height)), contours_segmentation, k, cv::Scalar(0, 255, 0), 4);

        //     std::cout << dbg_cnt << ", " << likelihoods.transpose() << std::endl << std::endl;
        //     cv::imwrite("./mask_" + std::to_string(dbg_cnt) + ".png", mask);

        //     dbg_cnt++;
        // }

        return std::make_pair(true, alternatives.at(selected));
    }

    return failed;
}


bool Filter::buffer_outlier_rejection_features()
{
    /* Measure depth. */
    bfl::Data camera_data;
    bool valid_data = false;
    std::tie(valid_data, camera_data) = camera_->measure();
    if (!valid_data)
        return false;

    std::tie(std::ignore, std::ignore, buffered_depth_) = any::any_cast<std::tuple<Eigen::Transform<double, 3, Affine>, cv::Mat, MatrixXf>>(camera_data);

    /* Measure segmentation. */
    bfl::Data segmentation_data;
    bool valid_segmentation = false;
    std::tie(valid_segmentation, segmentation_data) = segmentation_->measure();
    if (!valid_segmentation)
        return false;

    cv::Mat segmentation;
    std::tie(std::ignore, buffered_segmentation_) = any::any_cast<std::pair<bool, cv::Mat>>(segmentation_data);

    return true;
}


Gaussian Filter::correct_outlier_rejection(const Gaussian& prediction, const bool use_buffered_features)
{
    Gaussian correction = prediction;
    Gaussian corr_belief_v = prediction;
    Gaussian corr_belief_p_v = prediction;

    /* Standard correction. */
    p_correction_->correct(prediction, corr_belief_p_v);

    /* Correction with velocity only. */
    p_correction_->getMeasurementModel().freeze(CartesianQuaternionMeasurement::MeasurementMode::RepeatOnlyVelocity);
    p_correction_->correct(prediction, corr_belief_v);

    std::vector<Gaussian> alternatives;
    alternatives.push_back(corr_belief_p_v);
    alternatives.push_back(corr_belief_v);

    bool valid_alternative_selection;
    Gaussian best_alternative;
    std::tie(valid_alternative_selection, best_alternative) = pick_best_alternative(alternatives, use_buffered_features);

    if (valid_alternative_selection)
        correction = best_alternative;
    else
        correction = corr_belief_p_v;

    return correction;
}
