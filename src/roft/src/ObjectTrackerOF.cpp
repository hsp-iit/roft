/*
 * Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#include <ObjectTrackerOF.h>

#include <OTL/CameraMeasurement.h>
#include <OTL/DatasetImageSegmentation.h>
#include <OTL/DatasetImageSegmentationDelayed.h>
#include <OTL/DatasetImageOpticalFlow.h>
#include <OTL/ImageOpticalFlowNVOF.h>
#include <OTL/ImageOpticalFlowSource.h>
#include <OTL/ImageSegmentationSource.h>
#include <OTL/ModelParameters.h>
#include <OTL/OFAidedFilter.h>

#include <RobotsIO/Camera/Camera.h>
#include <RobotsIO/Camera/DatasetCamera.h>
#include <RobotsIO/Camera/YarpCamera.h>
#include <RobotsIO/Camera/CameraParameters.h>
#include <RobotsIO/Utils/DatasetTransform.h>
#include <RobotsIO/Utils/DatasetTransformDelayed.h>
#include <RobotsIO/Utils/ImageFileProbe.h>
#include <RobotsIO/Utils/Parameters.h>
#include <RobotsIO/Utils/YarpImageOfProbe.hpp>
#include <RobotsIO/Utils/YarpVectorOfProbe.hpp>
#include <RobotsIO/Utils/YarpBottleProbe.hpp>

#include <yarp/os/Bottle.h>
#include <yarp/os/Network.h>
#include <yarp/sig/Image.h>

using namespace Eigen;
using namespace OTL;
using namespace RobotsIO::Camera;
using namespace RobotsIO::Utils;
using namespace yarp::os;
using namespace yarp::sig;


ObjectTrackerOF::ObjectTrackerOF(const ResourceFinder& rf)
{
    const double sample_time = rf.check("sample_time", Value(0.03)).asDouble();

    /* Camera parameters. */
    const Bottle rf_camera = rf.findGroup("CAMERA");
    const std::string camera_source = rf_camera.check("source", Value("")).asString();
    const std::size_t camera_width = rf_camera.check("width", Value(640)).asInt();
    const std::size_t camera_height = rf_camera.check("height", Value(480)).asInt();
    const double camera_fx = rf_camera.check("fx", Value(320.0)).asDouble();
    const double camera_fy = rf_camera.check("fy", Value(240.0)).asDouble();
    const double camera_cx = rf_camera.check("cx", Value(0.0)).asDouble();
    const double camera_cy = rf_camera.check("cy", Value(0.0)).asDouble();

    std::string camera_path;
    std::string camera_data_prefix;
    std::string camera_rgb_prefix;
    std::string camera_depth_prefix;
    std::string camera_data_format;
    std::string camera_rgb_format;
    std::string camera_depth_format;
    std::size_t camera_heading_zeros;
    std::size_t camera_index_offset;
    if (camera_source == "dataset")
    {
        const Bottle rf_camera_dataset = rf.findGroup("CAMERA_DATASET");
        camera_path = rf_camera_dataset.check("path", Value("")).asString();
        camera_data_prefix = rf_camera_dataset.check("data_prefix", Value("")).asString();
        camera_rgb_prefix = rf_camera_dataset.check("rgb_prefix", Value("")).asString();
        camera_depth_prefix = rf_camera_dataset.check("depth_prefix", Value("")).asString();
        camera_data_format = rf_camera_dataset.check("data_format", Value("")).asString();
        camera_rgb_format = rf_camera_dataset.check("rgb_format", Value("")).asString();
        camera_depth_format = rf_camera_dataset.check("depth_format", Value("")).asString();
        camera_heading_zeros = rf_camera_dataset.check("heading_zeros", Value(0)).asInt();
        camera_index_offset = rf_camera_dataset.check("index_offset", Value(0)).asInt();
    }

    /* Initial conditions. */
    const Bottle rf_initial_conditions = rf.findGroup("INITIAL_CONDITION");

    const VectorXd p_v_0 = load_vector_double(rf_initial_conditions, "p_v_0", 3);
    const VectorXd p_w_0 = load_vector_double(rf_initial_conditions, "p_w_0", 3);
    const VectorXd p_x_0 = load_vector_double(rf_initial_conditions, "p_x_0", 3);
    const VectorXd p_axis_angle_0 = load_vector_double(rf_initial_conditions, "p_axis_angle_0", 4);
    const VectorXd p_cov_v_0 = load_vector_double(rf_initial_conditions, "p_cov_v_0", 3);
    const VectorXd p_cov_w_0 = load_vector_double(rf_initial_conditions, "p_cov_w_0", 3);
    const VectorXd p_cov_x_0 = load_vector_double(rf_initial_conditions, "p_cov_x_0", 3);
    const VectorXd p_cov_q_0 = load_vector_double(rf_initial_conditions, "p_cov_q_0", 3);

    const VectorXd v_v_0 = load_vector_double(rf_initial_conditions, "v_v_0", 3);
    const VectorXd v_w_0 = load_vector_double(rf_initial_conditions, "v_w_0", 3);
    const VectorXd v_cov_v_0 = load_vector_double(rf_initial_conditions, "v_cov_v_0", 3);
    const VectorXd v_cov_w_0 = load_vector_double(rf_initial_conditions, "v_cov_w_0", 3);

    /* Kinematic model. */
    const Bottle rf_kinematic_model = rf.findGroup("KINEMATIC_MODEL");

    const VectorXd psd_lin_acc = load_vector_double(rf_kinematic_model, "psd_lin_acc", 3);
    const VectorXd sigma_ang_vel = load_vector_double(rf_kinematic_model, "sigma_ang_vel", 3);

    const VectorXd kin_q_v = load_vector_double(rf_kinematic_model, "q_v", 3);
    const VectorXd kin_q_w = load_vector_double(rf_kinematic_model, "q_w", 3);

    /* Log parameters. */
    const Bottle rf_logging = rf.findGroup("LOG");
    bool enable_log = rf_logging.check("enable_log", Value(false)).asBool();
    bool enable_log_segmentation = rf_logging.check("enable_log_segmentation", Value(false)).asBool();
    const std::string log_path = rf_logging.check("absolute_log_path", Value("")).asString();
    if (enable_log && log_path == "")
    {
        std::cout << log_name_ << "::ctor. Invalid log path. Disabling log." << std::endl;

        enable_log = false;
    }

    /* Measurement parameters. */
    const Bottle rf_measurement = rf.findGroup("MEASUREMENT_MODEL");
    const VectorXd v_meas_cov_flow = load_vector_double(rf_measurement, "v_cov_flow", 2);

    const VectorXd p_meas_cov_v = load_vector_double(rf_measurement, "p_cov_v", 3);
    const VectorXd p_meas_cov_w = load_vector_double(rf_measurement, "p_cov_w", 3);
    const VectorXd p_meas_cov_x = load_vector_double(rf_measurement, "p_cov_x", 3);
    const VectorXd p_meas_cov_q = load_vector_double(rf_measurement, "p_cov_q", 3);

    const bool use_pose_measurement = rf_measurement.check("use_pose_measurement", Value(false)).asBool();
    const bool use_pose_resync = rf_measurement.check("use_pose_resync", Value(false)).asBool();
    const bool use_velocity_measurement = rf_measurement.check("use_vel_measurement", Value(false)).asBool();

    const double depth_maximum = rf_measurement.check("depth_maximum", Value(2.0)).asDouble();
    const double subsampling_radius = rf_measurement.check("subsampling_radius", Value(1.0)).asDouble();
    const bool flow_weighting = rf_measurement.check("flow_weighting", Value(false)).asBool();

    /* Model parameters. */
    const Bottle rf_model = rf.findGroup("MODEL");
    const std::string model_name = rf_model.check("name", Value("")).asString();
    const bool model_use_internal_db = rf_model.check("use_internal_db", Value(true)).asBool();
    const std::string model_internal_db_name = rf_model.check("internal_db_name", Value("YCBVideo")).asString();
    const std::string model_external_path = rf_model.check("external_path", Value("")).asString();

    /* Optical flow parameters. */
    const Bottle rf_optical_flow = rf.findGroup("OPTICAL_FLOW");
    const std::string optical_flow_source = rf_optical_flow.check("source", Value("")).asString();

    /* Optical flow dataset parameters. */
    std::string optical_flow_path;
    std::string optical_flow_set;
    std::size_t optical_flow_heading_zeros;
    std::size_t optical_flow_index_offset;
    if (optical_flow_source == "dataset")
    {
        const Bottle rf_optical_flow_dataset = rf.findGroup("OPTICAL_FLOW_DATASET");
        optical_flow_path = rf_optical_flow_dataset.check("path", Value("")).asString();
        optical_flow_set = rf_optical_flow_dataset.check("set", Value("")).asString();
        optical_flow_heading_zeros = rf_optical_flow_dataset.check("heading_zeros", Value(0)).asInt();
        optical_flow_index_offset = rf_optical_flow_dataset.check("index_offset", Value(0)).asInt();
    }

    /* Outlier detection parameters. */
    const Bottle rf_outlier = rf.findGroup("OUTLIER_REJECTION");
    const bool outlier_rejection_enable = rf_outlier.check("enable", Value(false)).asBool();
    const double outlier_rejection_gain = rf_outlier.check("gain", Value(1.0)).asDouble();

    /* Pose source .*/
    const Bottle& pose_bottle = rf.findGroup("POSE");
    const std::string pose_source_name = pose_bottle.check("source", Value("YARP")).asString();

    std::string pose_path;
    std::size_t pose_skip_rows;
    std::size_t pose_skip_cols;
    bool pose_fps_reduction;
    bool pose_delay;
    double pose_original_fps;
    double pose_desired_fps;
    if (pose_source_name == "dataset")
    {
        const Bottle rf_pose_dataset = rf.findGroup("POSE_DATASET");
        pose_path = rf_pose_dataset.check("path", Value("")).asString();
        pose_skip_rows = rf_pose_dataset.check("skip_rows", Value(0)).asInt();
        pose_skip_cols = rf_pose_dataset.check("skip_cols", Value(0)).asInt();

        const Bottle rf_pose_dataset_inference = rf.findGroup("POSE_DATASET_INFERENCE_SIMULATION");
        pose_fps_reduction = rf_pose_dataset_inference.check("fps_reduction", Value(false)).asBool();
        pose_delay = rf_pose_dataset_inference.check("delay", Value(false)).asBool();
        pose_original_fps = rf_pose_dataset_inference.check("original_fps", Value(1.0)).asDouble();
        pose_desired_fps = rf_pose_dataset_inference.check("desired_fps", Value(1.0)).asDouble();
    }

    /* Segmentation parameters. */
    const Bottle rf_segmentation = rf.findGroup("SEGMENTATION");
    const std::string segmentation_source = rf_segmentation.check("source", Value("")).asString();
    const bool flow_aided_segmentation = rf_segmentation.check("flow_aided", Value(false)).asBool();

    /* Segmentation dataset parameters. */
    std::string segmentation_path;
    std::string segmentation_format;
    std::string segmentation_set;
    std::size_t segmentation_heading_zeros;
    std::size_t segmentation_index_offset;
    bool segmentation_fps_reduction;
    bool segmentation_delay;
    double segmentation_original_fps;
    double segmentation_desired_fps;
    if (segmentation_source == "dataset")
    {
        const Bottle rf_segmentation_dataset = rf.findGroup("SEGMENTATION_DATASET");
        segmentation_path = rf_segmentation_dataset.check("path", Value("")).asString();
        segmentation_format = rf_segmentation_dataset.check("format", Value("")).asString();
        segmentation_set = rf_segmentation_dataset.check("set", Value("")).asString();
        segmentation_heading_zeros = rf_segmentation_dataset.check("heading_zeros", Value(0)).asInt();
        segmentation_index_offset = rf_segmentation_dataset.check("index_offset", Value(0)).asInt();

        const Bottle rf_segmentation_dataset_inference = rf.findGroup("SEGMENTATION_DATASET_INFERENCE_SIMULATION");
        segmentation_fps_reduction = rf_segmentation_dataset_inference.check("fps_reduction", Value(false)).asBool();
        segmentation_delay = rf_segmentation_dataset_inference.check("delay", Value(false)).asBool();
        segmentation_original_fps = rf_segmentation_dataset_inference.check("original_fps", Value(1.0)).asDouble();
        segmentation_desired_fps = rf_segmentation_dataset_inference.check("desired_fps", Value(1.0)).asDouble();
    }

    /* Unscented transform. */
    const Bottle rf_unscented_transform = rf.findGroup("UNSCENTED_TRANSFORM");
    const double ut_alpha = rf_unscented_transform.check("alpha", Value("1.0")).asDouble();
    const double ut_beta = rf_unscented_transform.check("beta", Value("2.0")).asDouble();
    const double ut_kappa = rf_unscented_transform.check("kappa", Value("0.0")).asDouble();

    /* Parameters summary. */
    std::cout << log_name_ << " parameters:" << std::endl;

    std::cout << "sample_time: " << sample_time << std::endl;

    std::cout << "Camera:" << std::endl;

    std::cout << "- source: " << camera_source << std::endl;
    std::cout << "- width: " << camera_width << std::endl;
    std::cout << "- height: " << camera_height << std::endl;
    std::cout << "- fx: " << camera_fx << std::endl;
    std::cout << "- fy: " << camera_fy << std::endl;
    std::cout << "- cx: " << camera_cx << std::endl;
    std::cout << "- cy: " << camera_cy << std::endl;

    if (camera_source == "dataset")
    {
        std::cout << "Camera dataset:" << std::endl;
        std::cout << "- path: " << camera_path << std::endl;
        std::cout << "- data_prefix: " << camera_data_prefix << std::endl;
        std::cout << "- rgb_prefix: " << camera_rgb_prefix << std::endl;
        std::cout << "- depth_prefix: " << camera_depth_prefix << std::endl;
        std::cout << "- data_format: " << camera_data_prefix << std::endl;
        std::cout << "- data_format: " << camera_rgb_format << std::endl;
        std::cout << "- data_format: " << camera_depth_format << std::endl;
        std::cout << "- heading_zeros: " << camera_heading_zeros << std::endl;
        std::cout << "- index_offset: " << camera_index_offset << std::endl;
    }

    std::cout << "Initial conditions:" << std::endl;

    std::cout << "  - position:" << std::endl;
    std::cout << "    - v_0: " << p_v_0.transpose() << std::endl;
    std::cout << "    - w_0: " << p_w_0.transpose() << std::endl;
    std::cout << "    - x_0: " << p_x_0.transpose() << std::endl;
    std::cout << "    - axis_angle_0: " << p_axis_angle_0.transpose() << std::endl;
    std::cout << "    - cov_v_0: " << p_cov_v_0.transpose() << std::endl;
    std::cout << "    - cov_w_0: " << p_cov_w_0.transpose() << std::endl;
    std::cout << "    - cov_x_0: " << p_cov_x_0.transpose() << std::endl;
    std::cout << "    - cov_q_0: " << p_cov_q_0.transpose() << std::endl;

    std::cout << "  - velocity:" << std::endl;
    std::cout << "    - v_0: " << v_v_0.transpose() << std::endl;
    std::cout << "    - w_0: " << v_w_0.transpose() << std::endl;
    std::cout << "    - cov_v_0: " << v_cov_v_0.transpose() << std::endl;
    std::cout << "    - cov_w_0: " << v_cov_w_0.transpose() << std::endl;

    std::cout << "Kinematic model:" << std::endl;

    std::cout << "  - position:" << std::endl;
    std::cout << "    - psd_lin_acc: " << psd_lin_acc.transpose() << std::endl;
    std::cout << "    - sigma_ang_vel: " << sigma_ang_vel.transpose() << std::endl;

    std::cout << "  - velocity:" << std::endl;
    std::cout << "    - q_v: " << kin_q_v.transpose() << std::endl;
    std::cout << "    - q_w: " << kin_q_w.transpose() << std::endl;

    std::cout << "Logging:" << std::endl;

    std::cout << "- enable_log: " << enable_log << std::endl;
    std::cout << "- enable_log_segmentation: " << enable_log_segmentation << std::endl;
    std::cout << "- absolute_log_path: " << log_path << std::endl;

    std::cout << "Measurement model:" << std::endl;

    std::cout << "  - position: " << std::endl;
    std::cout << "    - cov_v: " << p_meas_cov_v.transpose() << std::endl;
    std::cout << "    - cov_w: " << p_meas_cov_w.transpose() << std::endl;
    std::cout << "    - cov_x: " << p_meas_cov_x.transpose() << std::endl;
    std::cout << "    - cov_q: " << p_meas_cov_q.transpose() << std::endl;
    std::cout << "    - use_pose_measurement: " << use_pose_measurement << std::endl;
    std::cout << "    - use_pose_resync: " << use_pose_resync << std::endl;
    std::cout << "    - use_velocity_measurement: " << use_velocity_measurement << std::endl;

    std::cout << "  - velocity: " << std::endl;
    std::cout << "  - cov_flow: " << v_meas_cov_flow.transpose() << std::endl;
    std::cout << "  - depth_maximum: " << depth_maximum << std::endl;
    std::cout << "  - subsampling_radius: " << subsampling_radius << std::endl;
    std::cout << "  - flow_weighting: " << flow_weighting << std::endl;

    std::cout << "Model:" << std::endl;

    std::cout << "- model_name: " << model_name << std::endl;
    std::cout << "- model_use_internal_db: " << model_use_internal_db << std::endl;
    std::cout << "- model_internal_db_name: " << model_internal_db_name << std::endl;
    std::cout << "- model_external_path: " << model_external_path << std::endl;

    std::cout << "Optical flow:" << std::endl;

    std::cout << "- source: " << optical_flow_source << std::endl;

    if (optical_flow_source == "dataset")
    {
        std::cout << "Optical flow dataset:" << std::endl;
        std::cout << "- path: " << optical_flow_path << std::endl;
        std::cout << "- set: " << optical_flow_set << std::endl;
        std::cout << "- heading_zeros: " << optical_flow_heading_zeros << std::endl;
        std::cout << "- index_offset: " << optical_flow_index_offset << std::endl;
    }

    std::cout << "Outlier rejection:" << std::endl;

    std::cout << "- enable: " << outlier_rejection_enable << std::endl;
    std::cout << "- gain: " << outlier_rejection_gain << std::endl;

    std::cout << "Pose:" << std::endl;
    std::cout << "- source: " << pose_source_name << std::endl;

    if (pose_source_name == "dataset")
    {
        std::cout << "Pose dataset:" << std::endl;
        std::cout << "- path: " << pose_path << std::endl;
        std::cout << "- skip_rows: " << pose_skip_rows << std::endl;
        std::cout << "- skip_cols: " << pose_skip_cols << std::endl;

        if (pose_fps_reduction || pose_delay)
        {
            std::cout << "Pose dataset inference simulation:" << std::endl;
            std::cout << "- fps_reduction: " << pose_fps_reduction << std::endl;
            std::cout << "- delay: " << pose_delay << std::endl;
            std::cout << "- original_fps: " << pose_original_fps << std::endl;
            std::cout << "- desired_fps: " << pose_desired_fps << std::endl;
        }
    }

    std::cout << "Segmentation:" << std::endl;

    std::cout << "- source: " << segmentation_source << std::endl;
    std::cout << "- flow_aided: " << flow_aided_segmentation << std::endl;

    if (segmentation_source == "dataset")
    {
        std::cout << "Segmentation dataset:" << std::endl;

        std::cout << "- path: " << segmentation_path << std::endl;
        std::cout << "- format: " << segmentation_format << std::endl;
        std::cout << "- set: " << segmentation_set << std::endl;
        std::cout << "- heading_zeros: " << segmentation_heading_zeros << std::endl;
        std::cout << "- index_offset: " << segmentation_index_offset << std::endl;

        if (segmentation_fps_reduction || segmentation_delay)
        {
            std::cout << "Segmentation dataset inference simulation:" << std::endl;
            std::cout << "- fps_reduction: " << segmentation_fps_reduction << std::endl;
            std::cout << "- delay: " << segmentation_delay << std::endl;
            std::cout << "- original_fps: " << segmentation_original_fps << std::endl;
            std::cout << "- desired_fps: " << segmentation_desired_fps << std::endl;
        }
    }

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
    std::unique_ptr<Camera> camera_src;
    if (camera_source == "dataset")
    {
        camera_src = std::unique_ptr<DatasetCamera>
        (
            new DatasetCamera(camera_path,
                              camera_data_prefix, camera_rgb_prefix, camera_depth_prefix,
                              camera_data_format, camera_rgb_format, camera_depth_format,
                              camera_heading_zeros, camera_index_offset,
                              camera_width, camera_height,
                              camera_fx, camera_cx, camera_fy, camera_cy)
        );
    }
    else
        throw(std::runtime_error(log_name_ + "::ctor. Error: unknown camera source  " + camera_source));

    std::shared_ptr<CameraMeasurement> camera = std::make_shared<CameraMeasurement>(std::move(camera_src));

    /* Model parameters. */
    ModelParameters model_parameters;
    model_parameters.name(model_name);
    model_parameters.use_internal_db(model_use_internal_db);
    model_parameters.internal_db_name(model_internal_db_name);
    model_parameters.mesh_external_path(model_external_path);

    /* Pose source. */
    std::shared_ptr<RobotsIO::Utils::Transform> pose;
    if (pose_source_name == "dataset")
    {
        if (pose_delay || pose_fps_reduction)
        {
            if (!pose_fps_reduction)
                pose_desired_fps = pose_original_fps;

            pose = std::shared_ptr<DatasetTransformDelayed>
            (
                new DatasetTransformDelayed(pose_original_fps, pose_desired_fps, pose_delay,
                                            pose_path, pose_skip_rows, pose_skip_cols, 7)
            );
        }
        else
        {
            pose = std::shared_ptr<DatasetTransform>
            (
                new DatasetTransform(pose_path, pose_skip_rows, pose_skip_cols, 7)
            );
        }
    }
    else
        throw(std::runtime_error(log_name_ + "::ctor. Unknown pose source " + pose_source_name + "."));

    /* Segmentation source. */
    std::shared_ptr<ImageSegmentationSource> segmentation;
    if (segmentation_source == "dataset")
    {
        if (segmentation_delay || segmentation_fps_reduction)
        {
            if (!segmentation_fps_reduction)
                segmentation_desired_fps = segmentation_original_fps;

            segmentation = std::shared_ptr<DatasetImageSegmentation>
            (
                new DatasetImageSegmentationDelayed(segmentation_original_fps, segmentation_desired_fps, segmentation_delay,
                                                    segmentation_path,
                                                    segmentation_format,
                                                    camera_width, camera_height,
                                                    segmentation_set,
                                                    model_parameters,
                                                    segmentation_heading_zeros, segmentation_index_offset)
            );
        }
        else
        {
            segmentation = std::shared_ptr<DatasetImageSegmentation>
            (
                new DatasetImageSegmentation(segmentation_path,
                                             segmentation_format,
                                             camera_width, camera_height,
                                             segmentation_set,
                                             model_parameters,
                                             segmentation_heading_zeros, segmentation_index_offset)
            );
        }
    }
    else
        throw(std::runtime_error(log_name_ + "::ctor. Error: unknown segmentation source  " + segmentation_source));

    /* Flow source. */
    std::shared_ptr<ImageOpticalFlowSource> flow;
    if (optical_flow_source == "NVOF")
#ifdef HAS_NVOF
    {
        flow = std::make_shared<OTL::ImageOpticalFlowNVOF>(camera, OTL::ImageOpticalFlowNVOF::NVOFPerformance::Slow, false);
    }
#else
    {
        throw(std::runtime_error(log_name_ + "::ctor. Error: cannot use NVIDIA NVOF since your OpenCV installation does not support it."));
    }
#endif
    else if (optical_flow_source == "dataset")
    {
        flow = std::make_shared<DatasetImageOpticalFlow>(optical_flow_path, optical_flow_set, camera_width, camera_height, optical_flow_heading_zeros, optical_flow_index_offset);
    }
    else
        throw(std::runtime_error(log_name_ + "::ctor. Error: unknown optical flow source  " + optical_flow_source));

    /* Filter. */
    auto filter = std::unique_ptr<OFAidedFilter>
    (
        new OFAidedFilter
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
        )
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

    filter_ = std::move(filter);
}


bool ObjectTrackerOF::run()
{
    filter_->boot();

    filter_->run();

    if (!filter_->wait())
        return false;

    return true;
}


VectorXd ObjectTrackerOF::load_vector_double(const Bottle &rf, const std::string key, const std::size_t size)
{
    if (rf.find(key).isNull())
        throw(std::runtime_error("robmo-misc-object-tracker-of::load_vector_double. Cannot find key " + key + "."));

    Bottle* b = rf.find(key).asList();
    if (b == nullptr)
        throw(std::runtime_error("robmo-misc-object-tracker-of::load_vector_double. Cannot get vector having key " + key + " as a list."));


    if (b->size() != size)
        throw(std::runtime_error("robmo-misc-object-tracker-of::load_vector_double. Vector having key " + key + " has size "  + std::to_string(b->size()) + " (expected is " + std::to_string(size) + ")."));

    VectorXd vector(size);
    for (std::size_t i = 0; i < b->size(); i++)
    {
        Value item_v = b->get(i);
        if (item_v.isNull())
            throw(std::runtime_error("robmo-misc-object-tracker-of::load_vector_double." + std::to_string(i) + "-th element of of vector having key " + key + " is null."));

        if (!item_v.isDouble())
            throw(std::runtime_error("robmo-misc-object-tracker-of::load_vector_double." + std::to_string(i) + "-th element of of vector having key " + key + " is not a double."));

        vector(i) = item_v.asDouble();
    }

    return vector;
}
