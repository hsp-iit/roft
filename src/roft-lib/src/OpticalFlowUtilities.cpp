/*
 * Copyright (C) 2022 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#include <ROFT/OpticalFlowUtilities.h>

#include <algorithm>
#include <iostream>


cv::Vec3b ROFT::OpticalFlowUtils::compute_flow_color(const float& fx, const float& fy)
{
    throw(std::runtime_error("ROFT::OpticalFlowUtils::compute_flow_color. Not implemented."));
}


std::pair<bool, cv::Mat> ROFT::OpticalFlowUtils::draw_flow(const cv::Mat& flow, const float& max_motion)
{
    throw(std::runtime_error("ROFT::OpticalFlowUtils::draw_flow. Not implemented."));
}


std::pair<bool, cv::Mat> ROFT::OpticalFlowUtils::read_flow(const std::string& file_name)
{
    const std::string log_name = "ROFT::OpticalFlowUtils::read_flow";

    std::FILE* in;
    if ((in = std::fopen(file_name.c_str(), "rb")) == nullptr)
    {
        std::cout << log_name << " Error: cannot load flow frame " + file_name << std::endl;

        return std::make_pair(false, cv::Mat());
    }

    /* Load image type .*/
    int frame_type;
    if (std::fread(&frame_type, sizeof(frame_type), 1, in) != 1)
    {
        std::cout << log_name << " Error: cannot load flow frame type for frame" + file_name << std::endl;

        std::fclose(in);

        return std::make_pair(false, cv::Mat());
    }

    /* Load image size .*/
    std::size_t frame_size[2];
    if (std::fread(frame_size, sizeof(frame_size), 1, in) != 1)
    {
        std::cout << log_name << " Error: cannot load flow frame size for frame" + file_name << std::endl;

        std::fclose(in);

        return std::make_pair(false, cv::Mat());
    }

    /* Load image. */
    cv::Mat flow(cv::Size(frame_size[0], frame_size[1]), frame_type);
    if (std::fread(flow.data, flow.elemSize() / 2, 2 * frame_size[0] * frame_size[1], in) != (2 * frame_size[0] * frame_size[1]))
    {
        std::cout << log_name << " Error: cannot load flow data for frame" + file_name << std::endl;

        std::fclose(in);

        return std::make_pair(false, cv::Mat());
    }

    std::fclose(in);

    return std::make_pair(true, flow);
}


bool ROFT::OpticalFlowUtils::save_flow(const cv::Mat& flow, const std::string& output_path)
{
    const std::string log_name = "ROFT::OpticalFlowUtils::save_flow";

    if (flow.empty())
        return false;

    if ((flow.type() != CV_32FC2) && (flow.type() != CV_16SC2))
    {
        std::cerr << log_name + " Error: only CV_32FC2 or CV_16SC2 frames are supported." << std::endl;

        return false;
    }

    std::FILE* out = fopen(output_path.c_str(), "wb");
    if (out == nullptr)
    {
        std::cerr << log_name + " Error: cannot open output file." << std::endl;

        return false;
    }

    const int frame_type = flow.type();
    if (fwrite(&frame_type, sizeof(frame_type), 1, out) != 1)
    {
        std::cerr << log_name + " Error: cannot write frame type." << std::endl;

        fclose(out);

        return false;
    }

    const std::size_t frame_size[2] = {std::size_t(flow.cols), std::size_t(flow.rows)};
    if (fwrite(frame_size, sizeof(frame_size), 1, out) != 1)
    {
        std::cerr << log_name + " Error: cannot write frame size." << std::endl;

        fclose(out);

        return false;
    }

    if (fwrite(flow.data, flow.elemSize() / 2, 2 * flow.cols * flow.rows, out) != (2 * flow.cols * flow.rows))
    {
        std::cerr << log_name + " Error: cannot write frame." << std::endl;

        fclose(out);

        return false;
    }

    if (fclose(out) != 0)
    {
        std::cerr << log_name + " Error: cannot close the file." << std::endl;

        return false;
    }

    return true;
}


std::pair<bool, cv::Mat> ROFT::OpticalFlowUtils::pad_flow(const cv::Mat& flow, const std::size_t& width, const std::size_t& height)
{
    const std::string log_name = "ROFT::OpticalFlowUtils::pad_flow";

    auto placeholder = std::make_pair<bool, cv::Mat>(false, cv::Mat());

    if ((width < flow.cols) || (height < flow.rows))
    {
        std::cerr << log_name + " Error: cannot pad to " << width << " x " << height
                  << " as the input flow image has size " << flow.cols << " x " << flow.rows << std::endl;

        return placeholder;
    }

    /* Flow of target size. */
    cv::Mat output(cv::Size(width, height), CV_32FC2, cv::Scalar(0.0, 0.0));

    /* Copy input to the padded output.*/
    flow.copyTo(output(cv::Rect_<int>((width - flow.cols) / 2, (height - flow.rows) / 2, flow.cols, flow.rows)));

    return std::make_pair(true, output);
}
