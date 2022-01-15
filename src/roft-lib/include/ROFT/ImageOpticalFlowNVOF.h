/*
 * Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#ifndef ROFT_IMAGEOPTICALFLOWNVOF_H
#define ROFT_IMAGEOPTICALFLOWNVOF_H

#include <ROFT/CameraMeasurement.h>
#include <ROFT/ImageOpticalFlowSource.h>

#include <RobotsIO/Camera/Camera.h>
#include <RobotsIO/Camera/CameraParameters.h>

#include <opencv2/cudaoptflow.hpp>

namespace ROFT {
    class ImageOpticalFlowNVOF;
}


class ROFT::ImageOpticalFlowNVOF : public ROFT::ImageOpticalFlowSource
{
public:
    enum class NVOFPerformance_1_0 : std::underlying_type<cv::cuda::NvidiaOpticalFlow_1_0::NVIDIA_OF_PERF_LEVEL>::type
    {
        Slow = cv::cuda::NvidiaOpticalFlow_1_0::NVIDIA_OF_PERF_LEVEL::NV_OF_PERF_LEVEL_SLOW,
        Medium = cv::cuda::NvidiaOpticalFlow_1_0::NVIDIA_OF_PERF_LEVEL::NV_OF_PERF_LEVEL_MEDIUM,
        Fast = cv::cuda::NvidiaOpticalFlow_1_0::NVIDIA_OF_PERF_LEVEL::NV_OF_PERF_LEVEL_FAST
    };

#if CV_MAJOR_VERSION == 4 && CV_MINOR_VERSION == 5 && CV_SUBMINOR_VERSION >= 2
    enum class NVOFPerformance_2_0 : std::underlying_type<cv::cuda::NvidiaOpticalFlow_2_0::NVIDIA_OF_PERF_LEVEL>::type
    {
        Slow = cv::cuda::NvidiaOpticalFlow_2_0::NVIDIA_OF_PERF_LEVEL::NV_OF_PERF_LEVEL_SLOW,
        Medium = cv::cuda::NvidiaOpticalFlow_2_0::NVIDIA_OF_PERF_LEVEL::NV_OF_PERF_LEVEL_MEDIUM,
        Fast = cv::cuda::NvidiaOpticalFlow_2_0::NVIDIA_OF_PERF_LEVEL::NV_OF_PERF_LEVEL_FAST
    };
#endif

    ImageOpticalFlowNVOF(std::shared_ptr<ROFT::CameraMeasurement> camera_measurement, const NVOFPerformance_1_0& performance_setting, const bool use_temporal_hints = false);

#if CV_MAJOR_VERSION == 4 && CV_MINOR_VERSION == 5 && CV_SUBMINOR_VERSION >= 2
    ImageOpticalFlowNVOF(std::shared_ptr<ROFT::CameraMeasurement> camera_measurement, const NVOFPerformance_2_0& performance_setting, const bool use_temporal_hints = false);
#endif

    ~ImageOpticalFlowNVOF();

    bool step_frame() override;

    std::tuple<bool, cv::Mat> flow(const bool& blocking) override;

    bool is_stepping_required() const override;

    std::size_t get_grid_size() const override;

    float get_scaling_factor() const override;

    int get_matrix_type() const override;

private:
    const std::shared_ptr<ROFT::CameraMeasurement> camera_measurement_;

    RobotsIO::Camera::CameraParameters camera_parameters_;

    cv::Ptr<cv::cuda::NvidiaOpticalFlow_1_0> nvof_1_0_;

    cv::Ptr<cv::cuda::NvidiaOpticalFlow_2_0> nvof_2_0_;

    cv::cuda::GpuMat gpu_last_frame_;

    cv::Mat flow_;

    bool last_frame_in_ = false;

    bool flow_in_ = false;

    int nvof_version_;

    const std::size_t grid_size_;

    const float scaling_factor_;

    const int matrix_type_;

    const std::string log_name_ = "ImageOpticalFlowNVOF";
};

#endif /* ROFT_IMAGEOPTICALFLOWNVOF_H */
