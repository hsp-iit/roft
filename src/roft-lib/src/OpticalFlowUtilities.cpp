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
    static bool first = true;

    // relative lengths of color transitions:
    // these are chosen based on perceptual similarity
    // (e.g. one can distinguish more shades between red and yellow
    //  than between yellow and green)
    const int RY = 15;
    const int YG = 6;
    const int GC = 4;
    const int CB = 11;
    const int BM = 13;
    const int MR = 6;
    const int NCOLS = RY + YG + GC + CB + BM + MR;
    static cv::Vec3i colorWheel[NCOLS];

    if (first)
    {
        int k = 0;

        for (int i = 0; i < RY; ++i, ++k)
            colorWheel[k] = cv::Vec3i(255, 255 * i / RY, 0);

        for (int i = 0; i < YG; ++i, ++k)
            colorWheel[k] = cv::Vec3i(255 - 255 * i / YG, 255, 0);

        for (int i = 0; i < GC; ++i, ++k)
            colorWheel[k] = cv::Vec3i(0, 255, 255 * i / GC);

        for (int i = 0; i < CB; ++i, ++k)
            colorWheel[k] = cv::Vec3i(0, 255 - 255 * i / CB, 255);

        for (int i = 0; i < BM; ++i, ++k)
            colorWheel[k] = cv::Vec3i(255 * i / BM, 0, 255);

        for (int i = 0; i < MR; ++i, ++k)
            colorWheel[k] = cv::Vec3i(255, 0, 255 - 255 * i / MR);

        first = false;
    }

    const float rad = sqrt(fx * fx + fy * fy);
    const float a = atan2(-fy, -fx) / (float)CV_PI;

    const float fk = (a + 1.0f) / 2.0f * (NCOLS - 1);
    const int k0 = static_cast<int>(fk);
    const int k1 = (k0 + 1) % NCOLS;
    const float f = fk - k0;

    cv::Vec3b pix;

    for (int b = 0; b < 3; b++)
    {
        const float col0 = colorWheel[k0][b] / 255.0f;
        const float col1 = colorWheel[k1][b] / 255.0f;

        float col = (1 - f) * col0 + f * col1;

        if (rad <= 1)
            col = 1 - rad * (1 - col); // increase saturation with radius
        else
            col *= .75; // out of range

        pix[2 - b] = static_cast<uchar>(255.0 * col);
    }

    return pix;
}


std::pair<bool, cv::Mat> ROFT::OpticalFlowUtils::draw_flow(const cv::Mat& flow, const float& max_motion)
{
    if (flow.empty())
        return std::make_pair(false, cv::Mat());

    if (flow.type() != CV_32FC2)
    {
        std::cerr << "ROFT::OpticalFlowUtils::draw_flow. Error: only CV_32FC2 frames are supported." << std::endl;

        return std::make_pair(false, cv::Mat());
    }

    cv::Mat out;
    out.create(flow.size(), CV_8UC3);
    out.setTo(cv::Scalar::all(0));

    // determine motion range:
    float maxrad = max_motion;

    if (max_motion <= 0)
    {
        maxrad = 1;
        for (int y = 0; y < flow.rows; ++y)
        {
            for (int x = 0; x < flow.cols; ++x)
            {
                const cv::Vec2f f = flow.at<cv::Vec2f>(y, x);
                cv::Point2f u(f(0), f(1));

                if (!is_flow_valid(u.x, u.y))
                    continue;

                maxrad = std::max(maxrad, float(sqrt(u.x * u.x + u.y * u.y)));
            }
        }
    }

    for (int y = 0; y < flow.rows; ++y)
    {
        for (int x = 0; x < flow.cols; ++x)
        {
            const cv::Vec2f f = flow.at<cv::Vec2f>(y, x);
            cv::Point2f u(f(0), f(1));

            if (!is_flow_valid(u.x, u.y))
                continue;

            out.at<cv::Vec3b>(y, x) = compute_flow_color(u.x / maxrad, u.y / maxrad);
        }
    }

    return std::make_pair(true, out);
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
