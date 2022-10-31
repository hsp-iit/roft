/*
 * Copyright (C) 2022 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#ifndef ROFT_IMAGESEGMENTATIONOFAIDEDSOURCE_HPP
#define ROFT_IMAGESEGMENTATIONOFAIDEDSOURCE_HPP

#include <ROFT/CameraMeasurement.h>
#include <ROFT/ImageOpticalFlowSource.h>
#include <ROFT/OpticalFlowUtilities.h>

#include <RobotsIO/Camera/CameraParameters.h>
#include <RobotsIO/Utils/Segmentation.h>

#include <opencv2/opencv.hpp>

#include <memory>
#include <string>

namespace ROFT {
    template <class T>
    class ImageSegmentationOFAidedSource;
}


template <class T>
class ROFT::ImageSegmentationOFAidedSource : public RobotsIO::Utils::Segmentation
{
public:
    ImageSegmentationOFAidedSource(std::shared_ptr<RobotsIO::Utils::Segmentation> segmentation_source, std::shared_ptr<ROFT::ImageOpticalFlowSource> flow_source, std::shared_ptr<ROFT::CameraMeasurement> camera_measurement, const std::size_t connected_components = 1, const bool synchronous_mode = true);

    virtual ~ImageSegmentationOFAidedSource();

    bool step_frame() override;

    bool is_stepping_required() const override;

    void reset_data_loading_time() override;

    double get_data_loading_time() const override;

    std::pair<bool, cv::Mat> segmentation(const bool& blocking = false) override;

private:
    cv::Mat map(const std::vector<cv::Mat>& flow);

    std::shared_ptr<RobotsIO::Utils::Segmentation> segmentation_;

    std::shared_ptr<ROFT::ImageOpticalFlowSource> flow_;

    std::shared_ptr<ROFT::CameraMeasurement> camera_;

    std::size_t number_connected_components_;

    bool synchronous_mode_ = true;

    bool segmentation_available_ = false;

    bool is_first_frame_ = true;

    const std::size_t flow_grid_size_;

    const float flow_scaling_factor_;

    const int segm_frames_between_iterations_;

    cv::Mat mask_;

    cv::Mat coords_;

    std::vector<cv::Mat> flow_buffer_;

    const std::string log_name_ = "ImageSegmentationOFAidedSource";
};


template <class T>
ROFT::ImageSegmentationOFAidedSource<T>::ImageSegmentationOFAidedSource
(
    std::shared_ptr<RobotsIO::Utils::Segmentation> segmentation_source,
    std::shared_ptr<ROFT::ImageOpticalFlowSource> flow_source,
    std::shared_ptr<ROFT::CameraMeasurement> camera_measurement,
    const std::size_t connected_components,
    const bool synchronous_mode
) :
    segmentation_(segmentation_source),
    flow_(flow_source),
    camera_(camera_measurement),
    number_connected_components_(connected_components),
    synchronous_mode_(synchronous_mode),
    flow_grid_size_(flow_source->get_grid_size()),
    flow_scaling_factor_(flow_source->get_scaling_factor()),
    segm_frames_between_iterations_(segmentation_source->get_frames_between_iterations())
{
    RobotsIO::Camera::CameraParameters parameters;
    std::tie(std::ignore, parameters) = camera_->camera_parameters();

    coords_ = cv::Mat(cv::Size(parameters.width(), parameters.height()), CV_32FC2);
    for (std::size_t i = 0; i < parameters.width(); i++)
    {
        for(std::size_t j = 0; j < parameters.height(); j++)
        {
            coords_.at<cv::Vec2f>(j, i) = cv::Vec2f(float(i), float(j));
        }
    }
}


template <class T>
ROFT::ImageSegmentationOFAidedSource<T>::~ImageSegmentationOFAidedSource()
{}


template <class T>
bool ROFT::ImageSegmentationOFAidedSource<T>::step_frame()
{
    /* No stepping/freezing on camera and flow on purpose.
       Stepping is performed on the underlying segmentation source only, if required. */
    if (segmentation_->is_stepping_required())
        segmentation_->step_frame();

    /* Get segmentation. */
    bool valid_segmentation = false;
    cv::Mat mask;
    std::tie(valid_segmentation, mask) = segmentation_->segmentation(false);

    if (!segmentation_available_ && valid_segmentation)
    {
        /* The first time the mask is received, it is considered as an initialization. */
        segmentation_available_ = true;
        mask.copyTo(mask_);

        /* The first time the mask is received, valid_segmentation set to false
           to avoid breaking the propagation mechanism. */
        valid_segmentation = false;
    }

    /* Get flow. */
    bool valid_flow = false;
    cv::Mat flow;
    std::tie(valid_flow, flow) = flow_->flow(false);
    valid_flow &= !is_first_frame_;
    if (valid_flow)
    {
        /* If flow is available, store it in the buffer. */
        flow_buffer_.push_back(flow.clone());
    }

    /* Check if the mask is informative, otherwise it is skipped. */
    if (valid_segmentation)
    {
        cv::Mat non_zero_coordinates;
        findNonZero(mask, non_zero_coordinates);
        if (non_zero_coordinates.total() == 0)
            valid_segmentation = false;
    }

    if (valid_segmentation)
    {
        /* If a new mask is available, the buffered optical flow is used to propagate it. */
        mask.copyTo(mask_);
        cv::remap(mask_, mask_, map(flow_buffer_), cv::Mat(), cv::INTER_LINEAR, cv::BORDER_CONSTANT);

        /* Remove buffered flows. */
        flow_buffer_.clear();

    }
    else if (valid_flow && !valid_segmentation)
    {
        /* If no mask available, propagate the last mask using the current optical flow. */
        mask_.at<uchar>(0, 0) = 0;
        cv::remap(mask_, mask_, map({flow}), cv::Mat(), cv::INTER_LINEAR, cv::BORDER_CONSTANT);
    }

    is_first_frame_ = false;

    return true;
}


template <class T>
cv::Mat ROFT::ImageSegmentationOFAidedSource<T>::map(const std::vector<cv::Mat>& flow)
{
    cv::Mat map = cv::Mat(cv::Size(coords_.cols, coords_.rows), CV_32FC2, cv::Scalar(0.0, 0.0));

    int flow_starting_index = int(flow.size()) - segm_frames_between_iterations_;
    if (flow_starting_index < 0)
        flow_starting_index = 0;

    cv::Mat non_zero_coordinates;
    findNonZero(mask_, non_zero_coordinates);
    for (std::size_t i = 0; i < non_zero_coordinates.total(); i++)
    {
        const cv::Point& p = non_zero_coordinates.at<cv::Point>(i);
        const cv::Vec2f& target = coords_.at<cv::Vec2f>(p.y, p.x);
        float t_x = target(0);
        float t_y = target(1);

        bool error = false;
        for (int j = flow_starting_index; j < flow.size(); j++)
        {
            if (j < 0)
                continue;

            if ((int(t_x) < 0) || (int(t_x) >= coords_.cols) || (int(t_y) < 0) || (int(t_y) >= coords_.rows))
            {
                error = true;
                break;
            }

            const T& f = flow.at(j).at<T>(int(t_y / flow_grid_size_), int(t_x / flow_grid_size_));
            /* Given that T is template paramter, f might not be neessary a float, hence it is required to cast it. */
            t_x += float(f(0)) / flow_scaling_factor_;
            t_y += float(f(1)) / flow_scaling_factor_;
        }

        if (error || (int(t_x) < 0) || (int(t_x) >= coords_.cols) || (int(t_y) < 0) || (int(t_y) >= coords_.rows))
            continue;

        map.at<cv::Vec2f>(int(t_y), int(t_x)) = cv::Vec2f(p.x, p.y);
    }

    return map;
}


template <class T>
bool ROFT::ImageSegmentationOFAidedSource<T>::is_stepping_required() const
{
    return (synchronous_mode_ == true);
}


template <class T>
std::pair<bool, cv::Mat> ROFT::ImageSegmentationOFAidedSource<T>::segmentation(const bool& blocking)
{
    return std::make_pair(segmentation_available_, mask_);
}


template <class T>
void ROFT::ImageSegmentationOFAidedSource<T>::reset_data_loading_time()
{
    segmentation_->reset_data_loading_time();
}


template <class T>
double ROFT::ImageSegmentationOFAidedSource<T>::get_data_loading_time() const
{
    return segmentation_->get_data_loading_time();
}


#endif /* ROFT_IMAGESEGMENTATIONOFAIDEDSOURCE_HPP */
