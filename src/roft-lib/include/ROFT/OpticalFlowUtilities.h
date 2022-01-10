/*
 * Copyright (C) 2022 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#ifndef ROFT_OPTICALFLOWUTILITIES_H
#define ROFT_OPTICALFLOWUTILITIES_H

#include <opencv2/opencv.hpp>

#include <string>


namespace ROFT {
    namespace OpticalFlowUtils {

        inline bool is_flow_valid(const float& f_x, const float& f_y)
        {
            return !cvIsNaN(f_x) && !cvIsNaN(f_y) && fabs(f_x) < 1e9 && fabs(f_y) < 1e9;
        }

        cv::Vec3b compute_flow_color(const float& fx, const float& fy);

        std::pair<bool, cv::Mat> draw_flow(const cv::Mat& flow, const float& max_motion = 10);

        std::pair<bool, cv::Mat> read_flow(const std::string& file_name);

        bool save_flow(const cv::Mat& flow, const std::string& output_path);

        std::pair<bool, cv::Mat> pad_flow(const cv::Mat& flow, const std::size_t& width, const std::size_t& height);
    }
}

#endif /* OPTICALFLOWUTILITIES */
