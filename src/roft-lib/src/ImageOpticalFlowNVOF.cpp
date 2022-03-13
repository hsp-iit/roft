/*
 * Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#include <ROFT/ImageOpticalFlowNVOF.h>


using namespace Eigen;
using namespace ROFT;
using namespace RobotsIO;
using namespace bfl;
using namespace cv::cuda;
using CameraTuple = std::tuple<Transform<double, 3, Affine>, cv::Mat, MatrixXf>;


ImageOpticalFlowNVOF::ImageOpticalFlowNVOF
(
    std::shared_ptr<CameraMeasurement> camera_measurement,
    const NVOFPerformance_1_0& performance_setting,
    const bool use_temporal_hints
) :
    camera_measurement_(camera_measurement),
    nvof_version_(1),
    grid_size_(4),
    scaling_factor_(float(1 << 5)), /* NVIDIA names it the S10.5 format. */
    matrix_type_(CV_16SC2)
{
    /* Extract camera parameters. */
    bool valid_camera_parameters = false;
    std::tie(valid_camera_parameters, camera_parameters_) = camera_measurement_->camera_parameters();

    /* Initialize optical flow. */
    nvof_1_0_ = NvidiaOpticalFlow_1_0::create
    (
#if CV_MAJOR_VERSION == 4 && CV_MINOR_VERSION == 5 && CV_SUBMINOR_VERSION >= 2
        cv::Size(camera_parameters_.width(), camera_parameters_.height()),
#else
        camera_parameters_.width(), camera_parameters_.height(),
#endif
        static_cast<cv::cuda::NvidiaOpticalFlow_1_0::NVIDIA_OF_PERF_LEVEL>(performance_setting),
        use_temporal_hints
    );

    flow_ = cv::Mat(cv::Size(camera_parameters_.height(), camera_parameters_.width()), matrix_type_);
}


#if CV_MAJOR_VERSION == 4 && CV_MINOR_VERSION == 5 && CV_SUBMINOR_VERSION >= 2
ImageOpticalFlowNVOF::ImageOpticalFlowNVOF
(
    std::shared_ptr<CameraMeasurement> camera_measurement,
    const NVOFPerformance_2_0& performance_setting,
    const bool use_temporal_hints
) :
    camera_measurement_(camera_measurement),
    nvof_version_(2),
    grid_size_(1),
    scaling_factor_(1.0),
    matrix_type_(CV_32FC2)
{
    /* Extract camera parameters. */
    bool valid_camera_parameters = false;
    std::tie(valid_camera_parameters, camera_parameters_) = camera_measurement_->camera_parameters();

    /* Initialize optical flow. */
    nvof_2_0_ = NvidiaOpticalFlow_2_0::create
    (
        cv::Size(camera_parameters_.width(), camera_parameters_.height()),
        static_cast<cv::cuda::NvidiaOpticalFlow_2_0::NVIDIA_OF_PERF_LEVEL>(performance_setting),
        cv::cuda::NvidiaOpticalFlow_2_0::NV_OF_OUTPUT_VECTOR_GRID_SIZE_1,
        cv::cuda::NvidiaOpticalFlow_2_0::NV_OF_HINT_VECTOR_GRID_SIZE_1,
        use_temporal_hints
    );

    flow_ = cv::Mat(cv::Size(camera_parameters_.height(), camera_parameters_.width()), matrix_type_);
}
#endif


ImageOpticalFlowNVOF::~ImageOpticalFlowNVOF()
{
    if (nvof_version_ == 1)
    {
        if (nvof_1_0_ != nullptr)
            nvof_1_0_->collectGarbage();
    }
#if CV_MAJOR_VERSION == 4 && CV_MINOR_VERSION == 5 && CV_SUBMINOR_VERSION >= 2
    else if (nvof_version_ == 2)
    {
        if (nvof_2_0_ != nullptr)
            nvof_2_0_->collectGarbage();
    }
#endif
}


bool ImageOpticalFlowNVOF::step_frame()
{
    /* Extract camera data. */
    Data camera_data;
    bool valid_data = false;
    std::tie(valid_data, camera_data) = camera_measurement_->measure();
    if (!valid_data)
        return false;

    /* Get RGB frame. */
    CameraTuple camera_tuple = bfl::any::any_cast<CameraTuple>(camera_data);
    cv::Mat frame = std::get<1>(camera_tuple);

    /* We need the last frame in order to evaluate the optical flow. */
    if (!last_frame_in_)
    {
        cv::Mat last_frame;
        cv::cvtColor(frame, last_frame, cv::COLOR_BGR2GRAY);
        last_frame_in_ = true;

        gpu_last_frame_ = GpuMat(last_frame);

        return false;
    }

    /* Convert to gray-scale .*/
    cv::cvtColor(frame, frame, cv::COLOR_BGR2GRAY);

    /* Allocate storage .*/
    GpuMat gpu_frame(frame);
    GpuMat gpu_flow;

    /* Evaluate flow. */
    if (nvof_version_ == 1)
        nvof_1_0_->calc(gpu_last_frame_, gpu_frame, gpu_flow);
#if CV_MAJOR_VERSION == 4 && CV_MINOR_VERSION == 5 && CV_SUBMINOR_VERSION >= 2
    else if (nvof_version_ == 2)
        nvof_2_0_->calc(gpu_last_frame_, gpu_frame, gpu_flow);
#endif

    if(nvof_version_ == 1)
    {
        /* In this case we avoid upsampling as data is just duplicated all-over the pixels. */
        gpu_flow.download(flow_);
    }
#if CV_MAJOR_VERSION == 4 && CV_MINOR_VERSION == 5 && CV_SUBMINOR_VERSION >= 2
    else if (nvof_version_ == 2)
    {
        /* In this case per-pixel flow is provided with minimum overhead. */
        nvof_2_0_->convertToFloat(gpu_flow, flow_);
    }
#endif

    /* Store last frame. */
    gpu_last_frame_ = gpu_frame;

    flow_in_ = true;

    return true;
}


std::tuple<bool, cv::Mat> ImageOpticalFlowNVOF::flow(const bool& blocking)
{
    return std::make_tuple(flow_in_, flow_);
}


bool ImageOpticalFlowNVOF::is_stepping_required() const
{
    return true;
}


std::size_t ImageOpticalFlowNVOF::get_grid_size() const
{
    return grid_size_;
}


float ImageOpticalFlowNVOF::get_scaling_factor() const
{
    return scaling_factor_;
}


int ImageOpticalFlowNVOF::get_matrix_type() const
{
    return matrix_type_;
}
