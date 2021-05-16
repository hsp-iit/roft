/*
 * Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#include <librealsense2/rs.hpp>

#include <opencv2/opencv.hpp>

#include <RobotsIO/Utils/DepthToFile.h>

#include <chrono>
#include <fstream>


int main(int argc, char ** argv)
{
    /* Create pipeline. */
    rs2::pipeline pipe;
    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_DEPTH, 1280, 720, RS2_FORMAT_Z16, 30);
    cfg.enable_stream(RS2_STREAM_COLOR, 1280, 720, RS2_FORMAT_RGB8, 30);
    cfg.enable_stream(RS2_STREAM_INFRARED, 1, 1280, 720, RS2_FORMAT_Y8, 30);
    cfg.enable_stream(RS2_STREAM_INFRARED, 2, 1280, 720, RS2_FORMAT_Y8, 30);
    pipe.start(cfg);

    /* Create instance of alignment procedure. */
    rs2::align align_to_color(RS2_STREAM_COLOR);

    /* Storage for rgb, depth and time stamps. */
    std::vector<cv::Mat> rgb_frames;
    std::vector<cv::Mat> depth_frames;
    std::vector<double> stamps;

    /* Take note of current time. */
    auto t0 = std::chrono::steady_clock::now();

    /* Instantiate global timer. */
    double timer = 0.0;

    /* While loop for data collection. */
    rs2::colorizer c;
    while (true)
    {
        /* Local timer. */
        auto time0 = std::chrono::steady_clock::now();

        /* Wait for new frames. */
        rs2::frameset frameset = pipe.wait_for_frames();

        /* Align frames. */
        frameset = align_to_color.process(frameset);

        /* Extract separate frames and colorize. */
        auto depth = frameset.get_depth_frame();
        auto color = frameset.get_color_frame();
        /* Not sure it is required, however it stabilizes frame rate! */
        auto colorized_depth = c.colorize(depth);

        /* Wrap data around OpenCV matrices. */
        const int w = depth.as<rs2::video_frame>().get_width();
        const int h = depth.as<rs2::video_frame>().get_height();
        cv::Mat cv_rgb(cv::Size(w, h), CV_8UC3, (void*) color.get_data(), cv::Mat::AUTO_STEP);
        cv::Mat cv_depth(cv::Size(w, h), CV_16U, (void*) depth.get_data(), cv::Mat::AUTO_STEP);
        cv_depth.convertTo(cv_depth, CV_32FC1, 0.001);

        /* Wait some time such that the pipeline reaches staedy state behavior. */
        if (std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - t0).count() > 4.0)
        {
            rgb_frames.push_back(cv_rgb.clone());
            depth_frames.push_back(cv_depth.clone());

            double delta = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - time0).count() / 1000.0 / 1000.0;
            timer += delta;
            stamps.push_back(timer);

            // std::cout << "Elapsed " << std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - time0).count() / 1000.0 << " (ms)" << std::endl;
        }

        /* Collect a one minute long sequence. */
        if (std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - t0).count() > 60.0)
            break;
    }

    /* Save on disk. */
    std::cout << "Saving..." << std::endl;

    /* Output file for timestamps. */
    std::ofstream data;
    data.open("./data.txt");

    for (std::size_t i = 0; i < rgb_frames.size(); i++)
    {
        std::cout << "frame " << i << std::endl;

        /* Save RGB frame. */
        cv::cvtColor(rgb_frames.at(i), rgb_frames.at(i), cv::COLOR_BGR2RGB);
        cv::imwrite("./rgb/" + std::to_string(i) + ".png", rgb_frames.at(i));

        /* Save depth frame. */
        RobotsIO::Utils::depth_to_file("./depth/" + std::to_string(i) + ".float", depth_frames.at(i));

        data << stamps.at(i) << " " << stamps.at(i) << " 0.0 0.0 0.0 0.0 0.0 0.0 0.0" << std::endl;
    }
    data.close();

    return EXIT_SUCCESS;
}
