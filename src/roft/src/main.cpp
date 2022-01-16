/*
 * Copyright (C) 2022 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#include <ConfigParser.h>

#include <BayesFilters/FilteringAlgorithm.h>

#include <ROFT/CameraMeasurement.h>
#include <ROFT/DatasetImageSegmentation.h>
#include <ROFT/DatasetImageSegmentationDelayed.h>
#include <ROFT/DatasetImageOpticalFlow.h>
#ifdef HAS_NVOF
#include <ROFT/ImageOpticalFlowNVOF.h>
#endif
#include <ROFT/ImageOpticalFlowSource.h>
#include <ROFT/ImageSegmentationSource.h>
#include <ROFT/ModelParameters.h>
#include <ROFT/Filter.h>

#include <RobotsIO/Camera/Camera.h>
#include <RobotsIO/Camera/DatasetCamera.h>
#include <RobotsIO/Camera/CameraParameters.h>
#include <RobotsIO/Utils/DatasetTransform.h>
#include <RobotsIO/Utils/DatasetTransformDelayed.h>
#include <RobotsIO/Utils/ImageFileProbe.h>
#include <RobotsIO/Utils/Parameters.h>

using namespace bfl;
using namespace Eigen;
using namespace ROFT;
using namespace RobotsIO::Camera;
using namespace RobotsIO::Utils;


int main(int argc, char** argv)
{
    ConfigParser conf(argc, argv);

    double sample_time; conf("sample_time", sample_time);

    /* Camera parameters. */
    int camera_width;                   conf("camera_dataset.width", camera_width);
    int camera_height;                  conf("camera_dataset.height", camera_height);
    double camera_fx;                   conf("camera_dataset.fx", camera_fx);
    double camera_fy;                   conf("camera_dataset.fy", camera_fy);
    double camera_cx;                   conf("camera_dataset.cx", camera_cx);
    double camera_cy;                   conf("camera_dataset.cy", camera_cy);
    std::string camera_path;            conf("camera_dataset.path", camera_path);
    std::string camera_data_prefix;     conf("camera_dataset.data_prefix", camera_data_prefix);
    std::string camera_rgb_prefix;      conf("camera_dataset.rgb_prefix", camera_rgb_prefix);
    std::string camera_depth_prefix;    conf("camera_dataset.depth_prefix", camera_depth_prefix);
    std::string camera_data_format;     conf("camera_dataset.data_format", camera_data_format);
    std::string camera_rgb_format;      conf("camera_dataset.rgb_format", camera_rgb_format);
    std::string camera_depth_format;    conf("camera_dataset.depth_format", camera_depth_format);
    int camera_heading_zeros;           conf("camera_dataset.heading_zeros", camera_heading_zeros);
    int camera_index_offset;            conf("camera_dataset.index_offset", camera_index_offset);

    /* Initial conditions. */
    VectorXd p_v_0;                     conf("initial_condition.pose.v", p_v_0);
    VectorXd p_w_0;                     conf("initial_condition.pose.w", p_w_0);
    VectorXd p_x_0;                     conf("initial_condition.pose.x", p_x_0);
    VectorXd p_axis_angle_0;            conf("initial_condition.pose.axis_angle", p_axis_angle_0);
    VectorXd p_cov_v_0;                 conf("initial_condition.pose.cov_v", p_cov_v_0);
    VectorXd p_cov_w_0;                 conf("initial_condition.pose.cov_w", p_cov_w_0);
    VectorXd p_cov_x_0;                 conf("initial_condition.pose.cov_x", p_cov_x_0);
    VectorXd p_cov_q_0;                 conf("initial_condition.pose.cov_q", p_cov_q_0);
    VectorXd v_v_0;                     conf("initial_condition.velocity.v", v_v_0);
    VectorXd v_w_0;                     conf("initial_condition.velocity.w", v_w_0);
    VectorXd v_cov_v_0;                 conf("initial_condition.velocity.cov_v", v_cov_v_0);
    VectorXd v_cov_w_0;                 conf("initial_condition.velocity.cov_w", v_cov_w_0);

    /* Kinematic model. */
    VectorXd psd_lin_acc;               conf("kinematic_model.pose.sigma_linear", psd_lin_acc);
    VectorXd sigma_ang_vel;             conf("kinematic_model.pose.sigma_angular", sigma_ang_vel);
    VectorXd kin_q_v;                   conf("kinematic_model.velocity.sigma_linear", kin_q_v);
    VectorXd kin_q_w;                   conf("kinematic_model.velocity.sigma_angular", kin_q_w);

    /* Log parameters. */
    bool enable_log;                    conf("log.enable", enable_log);
    bool enable_log_segmentation;       conf("log.enable_segmentation", enable_log_segmentation);
    std::string log_path;               conf("log.path", log_path);
    if (enable_log && log_path == "")
    {
        std::cout << "Invalid log path. Disabling log." << std::endl;

        enable_log = false;
    }

    /* Measurement parameters. */
    VectorXd p_meas_cov_v;              conf("measurement_model.pose.cov_v", p_meas_cov_v);
    VectorXd p_meas_cov_w;              conf("measurement_model.pose.cov_w", p_meas_cov_w);
    VectorXd p_meas_cov_x;              conf("measurement_model.pose.cov_x", p_meas_cov_x);
    VectorXd p_meas_cov_q;              conf("measurement_model.pose.cov_q", p_meas_cov_q);
    VectorXd v_meas_cov_flow;           conf("measurement_model.velocity.cov_flow", v_meas_cov_flow);
    double depth_maximum;               conf("measurement_model.velocity.depth_maximum", depth_maximum);
    double subsampling_radius;          conf("measurement_model.velocity.subsampling_radius", subsampling_radius);
    bool flow_weighting;                conf("measurement_model.velocity.weight_flow", flow_weighting);

    bool use_pose_measurement;          conf("measurement_model.use_pose", use_pose_measurement);
    bool use_pose_resync;               conf("measurement_model.use_pose_resync", use_pose_resync);
    bool use_velocity_measurement;      conf("measurement_model.use_velocity", use_velocity_measurement);

    /* Model parameters. */
    std::string model_name;             conf("model.name", model_name);
    bool model_use_internal_db;         conf("model.use_internal_db", model_use_internal_db);
    std::string model_internal_db_name; conf("model.internal_db_name", model_internal_db_name);
    std::string model_external_path;    conf("model.external_path", model_external_path);

    /* Optical flow dataset parameters. */
    std::string optical_flow_path;      conf("optical_flow_dataset.path", optical_flow_path);
    std::string optical_flow_set;       conf("optical_flow_dataset.set", optical_flow_set);
    int optical_flow_heading_zeros;     conf("optical_flow_dataset.heading_zeros", optical_flow_heading_zeros);
    int optical_flow_index_offset;      conf("optical_flow_dataset.index_offset", optical_flow_index_offset);

    /* Outlier detection parameters. */
    bool outlier_rejection_enable;      conf("outlier_rejection.enable", outlier_rejection_enable);
    double outlier_rejection_gain;      conf("outlier_rejection.gain", outlier_rejection_gain);

    /* Pose dataset parameters .*/
    std::string pose_path;              conf("pose_dataset.path", pose_path);
    int pose_skip_rows;                 conf("pose_dataset.skip_rows", pose_skip_rows);
    int pose_skip_cols;                 conf("pose_dataset.skip_cols", pose_skip_cols);
    bool pose_fps_reduction;            conf("pose_dataset.fps_reduction", pose_fps_reduction);
    bool pose_delay;                    conf("pose_dataset.delay", pose_delay);
    double pose_original_fps;           conf("pose_dataset.original_fps", pose_original_fps);
    double pose_desired_fps;            conf("pose_dataset.desired_fps", pose_desired_fps);

    /* Segmentation dataset parameters. */
    std::string segmentation_path;      conf("segmentation_dataset.path", segmentation_path);
    std::string segmentation_format;    conf("segmentation_dataset.format", segmentation_format);
    std::string segmentation_set;       conf("segmentation_dataset.set", segmentation_set);
    int segmentation_heading_zeros;     conf("segmentation_dataset.heading_zeros", segmentation_heading_zeros);
    int segmentation_index_offset;      conf("segmentation_dataset.index_offset", segmentation_index_offset);
    double segmentation_original_fps;   conf("segmentation_dataset.original_fps", segmentation_original_fps);
    double segmentation_desired_fps;    conf("segmentation_dataset.desired_fps", segmentation_desired_fps);
    bool segmentation_fps_reduction;    conf("segmentation_dataset.fps_reduction", segmentation_fps_reduction);
    bool segmentation_delay;            conf("segmentation_dataset.delay", segmentation_delay);
    bool flow_aided_segmentation;       conf("segmentation_dataset.flow_aided", flow_aided_segmentation);

    /* Unscented transform. */
    double ut_alpha; conf("unscented_transform.alpha", ut_alpha);
    double ut_beta;  conf("unscented_transform.beta", ut_beta);
    double ut_kappa; conf("unscented_transform.kappa", ut_kappa);

    /* Parameters summary. */
    std::cout << "Sample_time: " << sample_time << std::endl;
    std::cout << std::endl;

    std::cout << "Camera dataset:" << std::endl;

    std::cout << "- width: " << camera_width << std::endl;
    std::cout << "- height: " << camera_height << std::endl;
    std::cout << "- fx: " << camera_fx << std::endl;
    std::cout << "- fy: " << camera_fy << std::endl;
    std::cout << "- cx: " << camera_cx << std::endl;
    std::cout << "- cy: " << camera_cy << std::endl;
    std::cout << "- path: " << camera_path << std::endl;
    std::cout << "- data_prefix: " << camera_data_prefix << std::endl;
    std::cout << "- rgb_prefix: " << camera_rgb_prefix << std::endl;
    std::cout << "- depth_prefix: " << camera_depth_prefix << std::endl;
    std::cout << "- data_format: " << camera_data_format << std::endl;
    std::cout << "- rgb_format: " << camera_rgb_format << std::endl;
    std::cout << "- depth_format: " << camera_depth_format << std::endl;
    std::cout << "- heading_zeros: " << camera_heading_zeros << std::endl;
    std::cout << "- index_offset: " << camera_index_offset << std::endl;
    std::cout << std::endl;

    std::cout << "Initial conditions:" << std::endl;

    std::cout << "  - pose:" << std::endl;
    std::cout << "    - v: " << p_v_0.transpose() << std::endl;
    std::cout << "    - w: " << p_w_0.transpose() << std::endl;
    std::cout << "    - x: " << p_x_0.transpose() << std::endl;
    std::cout << "    - axis_angle: " << p_axis_angle_0.transpose() << std::endl;
    std::cout << "    - cov_v: " << p_cov_v_0.transpose() << std::endl;
    std::cout << "    - cov_w: " << p_cov_w_0.transpose() << std::endl;
    std::cout << "    - cov_x: " << p_cov_x_0.transpose() << std::endl;
    std::cout << "    - cov_q: " << p_cov_q_0.transpose() << std::endl;

    std::cout << "  - velocity:" << std::endl;
    std::cout << "    - v_0: " << v_v_0.transpose() << std::endl;
    std::cout << "    - w_0: " << v_w_0.transpose() << std::endl;
    std::cout << "    - cov_v_0: " << v_cov_v_0.transpose() << std::endl;
    std::cout << "    - cov_w_0: " << v_cov_w_0.transpose() << std::endl;
    std::cout << std::endl;

    std::cout << "Kinematic model:" << std::endl;

    std::cout << "  - pose:" << std::endl;
    std::cout << "    - psd_lin_acc: " << psd_lin_acc.transpose() << std::endl;
    std::cout << "    - sigma_ang_vel: " << sigma_ang_vel.transpose() << std::endl;

    std::cout << "  - velocity:" << std::endl;
    std::cout << "    - q_v: " << kin_q_v.transpose() << std::endl;
    std::cout << "    - q_w: " << kin_q_w.transpose() << std::endl;
    std::cout << std::endl;

    std::cout << "Logging:" << std::endl;

    std::cout << "- enable_log: " << enable_log << std::endl;
    std::cout << "- enable_log_segmentation: " << enable_log_segmentation << std::endl;
    std::cout << "- absolute_log_path: " << log_path << std::endl;
    std::cout << std::endl;

    std::cout << "Measurement model:" << std::endl;

    std::cout << "  - pose: " << std::endl;
    std::cout << "    - cov_v: " << p_meas_cov_v.transpose() << std::endl;
    std::cout << "    - cov_w: " << p_meas_cov_w.transpose() << std::endl;
    std::cout << "    - cov_x: " << p_meas_cov_x.transpose() << std::endl;
    std::cout << "    - cov_q: " << p_meas_cov_q.transpose() << std::endl;
    std::cout << "    - use_pose_measurement: " << use_pose_measurement << std::endl;
    std::cout << "    - use_pose_resync: " << use_pose_resync << std::endl;
    std::cout << "    - use_velocity_measurement: " << use_velocity_measurement << std::endl;

    std::cout << "  - velocity: " << std::endl;
    std::cout << "    - cov_flow: " << v_meas_cov_flow.transpose() << std::endl;
    std::cout << "    - depth_maximum: " << depth_maximum << std::endl;
    std::cout << "    - subsampling_radius: " << subsampling_radius << std::endl;
    std::cout << "    - weight_flow: " << flow_weighting << std::endl;
    std::cout << std::endl;

    std::cout << "Model:" << std::endl;

    std::cout << "- model_name: " << model_name << std::endl;
    std::cout << "- model_use_internal_db: " << model_use_internal_db << std::endl;
    std::cout << "- model_internal_db_name: " << model_internal_db_name << std::endl;
    std::cout << "- model_external_path: " << model_external_path << std::endl;
    std::cout << std::endl;

    std::cout << "Optical flow dataset:" << std::endl;

    std::cout << "- path: " << optical_flow_path << std::endl;
    std::cout << "- set: " << optical_flow_set << std::endl;
    std::cout << "- heading_zeros: " << optical_flow_heading_zeros << std::endl;
    std::cout << "- index_offset: " << optical_flow_index_offset << std::endl;
    std::cout << std::endl;

    std::cout << "Outlier rejection:" << std::endl;

    std::cout << "- enable: " << outlier_rejection_enable << std::endl;
    std::cout << "- gain: " << outlier_rejection_gain << std::endl;
    std::cout << std::endl;

    std::cout << "Pose dataset:" << std::endl;
    std::cout << "- path: " << pose_path << std::endl;
    std::cout << "- skip_rows: " << pose_skip_rows << std::endl;
    std::cout << "- skip_cols: " << pose_skip_cols << std::endl;

    if (pose_fps_reduction || pose_delay)
    {
        std::cout << "- fps_reduction: " << pose_fps_reduction << std::endl;
        std::cout << "- delay: " << pose_delay << std::endl;
        std::cout << "- original_fps: " << pose_original_fps << std::endl;
        std::cout << "- desired_fps: " << pose_desired_fps << std::endl;
    }
    std::cout << std::endl;

    std::cout << "Segmentation dataset:" << std::endl;

    std::cout << "- flow_aided: " << flow_aided_segmentation << std::endl;
    std::cout << "- path: " << segmentation_path << std::endl;
    std::cout << "- format: " << segmentation_format << std::endl;
    std::cout << "- set: " << segmentation_set << std::endl;
    std::cout << "- heading_zeros: " << segmentation_heading_zeros << std::endl;
    std::cout << "- index_offset: " << segmentation_index_offset << std::endl;

    if (segmentation_fps_reduction || segmentation_delay)
    {
        std::cout << "- fps_reduction: " << segmentation_fps_reduction << std::endl;
        std::cout << "- delay: " << segmentation_delay << std::endl;
        std::cout << "- original_fps: " << segmentation_original_fps << std::endl;
        std::cout << "- desired_fps: " << segmentation_desired_fps << std::endl;
    }
    std::cout << std::endl;

    std::cout << "Unscented transform:" << std::endl;
    std::cout << "- alpha: " << ut_alpha << std::endl;
    std::cout << "- beta: "  << ut_beta << std::endl;
    std::cout << "- kappa: " << ut_kappa << std::endl;

    /* Compose vectors. */
    VectorXd p_initial_condition(13);
    p_initial_condition.head<3>() = p_v_0;
    p_initial_condition.segment<3>(3) = p_w_0;
    p_initial_condition.segment<3>(6) = p_x_0;
    Quaterniond q_0 (AngleAxisd(p_axis_angle_0(3), p_axis_angle_0.head<3>()));
    p_initial_condition.tail<4>()(0) = q_0.w();
    p_initial_condition.tail<4>()(1) = q_0.x();
    p_initial_condition.tail<4>()(2) = q_0.y();
    p_initial_condition.tail<4>()(3) = q_0.z();

    VectorXd p_initial_covariance(12);
    p_initial_covariance.head<3>() = p_cov_v_0;
    p_initial_covariance.segment<3>(3) = p_cov_w_0;
    p_initial_covariance.segment<3>(6) = p_cov_x_0;
    p_initial_covariance.tail<3>() = p_cov_q_0;

    VectorXd v_initial_condition(6);
    v_initial_condition.head<3>() = v_v_0;
    v_initial_condition.tail<3>() = v_w_0;

    VectorXd v_initial_covariance(6);
    v_initial_covariance.head<3>() = v_cov_v_0;
    v_initial_covariance.tail<3>() = v_cov_w_0;

    VectorXd p_model_covariance(6);
    p_model_covariance.head<3>() = sigma_ang_vel;
    p_model_covariance.tail<3>() = psd_lin_acc;

    VectorXd v_model_covariance(6);
    v_model_covariance.head<3>() = kin_q_v;
    v_model_covariance.tail<3>() = kin_q_w;

    VectorXd p_measurement_covariance(12);
    p_measurement_covariance.head<3>() = p_meas_cov_v;
    p_measurement_covariance.segment<3>(3) = p_meas_cov_w;
    p_measurement_covariance.segment<3>(6) = p_meas_cov_x;
    p_measurement_covariance.tail<3>() = p_meas_cov_q;

    VectorXd v_measurement_covariance = v_meas_cov_flow;

    /* Camera source. */
    std::unique_ptr<Camera> camera_src = std::make_unique<DatasetCamera>
    (
        camera_path,
        camera_data_prefix, camera_rgb_prefix, camera_depth_prefix,
        camera_data_format, camera_rgb_format, camera_depth_format,
        camera_heading_zeros, camera_index_offset,
        camera_width, camera_height,
        camera_fx, camera_cx, camera_fy, camera_cy
    );
    std::shared_ptr<CameraMeasurement> camera = std::make_shared<CameraMeasurement>(std::move(camera_src));

    /* Model parameters. */
    ModelParameters model_parameters;
    model_parameters.name(model_name);
    model_parameters.use_internal_db(model_use_internal_db);
    model_parameters.internal_db_name(model_internal_db_name);
    model_parameters.mesh_external_path(model_external_path);

    /* Pose source. */
    std::shared_ptr<RobotsIO::Utils::Transform> pose;
    if (pose_delay || pose_fps_reduction)
    {
        if (!pose_fps_reduction)
            pose_desired_fps = pose_original_fps;

        pose = std::make_shared<DatasetTransformDelayed>(pose_original_fps, pose_desired_fps, pose_delay, pose_path, pose_skip_rows, pose_skip_cols, 7);
    }
    else
        pose = std::make_shared<DatasetTransform>(pose_path, pose_skip_rows, pose_skip_cols, 7);

    /* Segmentation source. */
    std::shared_ptr<ImageSegmentationSource> segmentation;
    if (segmentation_delay || segmentation_fps_reduction)
    {
        if (!segmentation_fps_reduction)
            segmentation_desired_fps = segmentation_original_fps;

        segmentation = std::make_shared<DatasetImageSegmentationDelayed>
        (
            segmentation_original_fps, segmentation_desired_fps, segmentation_delay,
            segmentation_path, segmentation_format,
            camera_width, camera_height,
            segmentation_set, model_parameters,
            segmentation_heading_zeros, segmentation_index_offset
        );
    }
    else
        segmentation = std::make_shared<DatasetImageSegmentation>
        (
            segmentation_path, segmentation_format,
            camera_width, camera_height,
            segmentation_set, model_parameters,
            segmentation_heading_zeros, segmentation_index_offset
        );

    /* Flow source. */
    std::shared_ptr<ImageOpticalFlowSource> flow;
    // if (optical_flow_source == "NVOF")
// #ifdef HAS_NVOF
    // {
    //     flow = std::make_shared<OTL::ImageOpticalFlowNVOF>(camera, OTL::ImageOpticalFlowNVOF::NVOFPerformance::Slow, false);
    // }
// #else
    // {
    //     throw(std::runtime_error(log_name_ + "::ctor. Error: cannot use NVIDIA NVOF since your OpenCV installation does not support it."));
    // }
// #endif
    flow = std::make_shared<DatasetImageOpticalFlow>(optical_flow_path, optical_flow_set, camera_width, camera_height, optical_flow_heading_zeros, optical_flow_index_offset);


    /* Filter. */
    std::unique_ptr<Filter> filter = std::make_unique<Filter>
    (
        camera, segmentation, flow, pose,
        model_parameters,
        p_initial_condition, p_initial_covariance, p_model_covariance, p_measurement_covariance,
        v_initial_condition, v_initial_covariance, v_model_covariance, v_measurement_covariance,
        ut_alpha, ut_beta, ut_kappa,
        sample_time,
        use_pose_measurement, use_pose_resync, outlier_rejection_enable, outlier_rejection_gain,
        use_velocity_measurement, flow_weighting, flow_aided_segmentation, depth_maximum, subsampling_radius,
        enable_log, log_path, ""
    );
    if (enable_log_segmentation)
    {
        std::unique_ptr<Probe> probe_0 = std::unique_ptr<ImageFileProbe>
        (
            new ImageFileProbe(log_path + "/segmentation/", "", "png")
        );
        filter->set_probe("output_segmentation", std::move(probe_0));

        std::unique_ptr<Probe> probe_1 = std::unique_ptr<ImageFileProbe>
        (
            new ImageFileProbe(log_path + "/segmentation_refined/", "", "png")
        );
        filter->set_probe("output_segmentation_refined", std::move(probe_1));
    }
    if (enable_log)
        filter->enable_log(log_path, "");

    /* Run filter. */
    filter->boot();
    filter->run();
    if (!filter->wait())
        return EXIT_FAILURE;

    return EXIT_SUCCESS;
}
