/*
 * Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#include <OTL/CameraMeasurement.h>
#include <OTL/ImageOpticalFlowNVOF.h>
#include <OTL/OpticalFlowUtilities.h>

#include <RobotsIO/Camera/DatasetCamera.h>

#include <sstream>
#include <string>

using namespace OTL;
using namespace OTL::OpticalFlowUtils;
using namespace RobotsIO::Camera;


bool parse_size_t (char** argv, const std::size_t& index, const std::string& name, std::size_t& retrieved)
{
    try
    {
        if (std::stoi(argv[index]) < 0)
            throw(std::invalid_argument(""));
        retrieved = std::stoul(argv[index]);
    }
    catch (std::invalid_argument)
    {
        std::cerr << "Invalid value " << argv[index] << " for parameter <" << name << ">." << std::endl;
        return false;
    }

    return true;
}


int main(int argc, char** argv)
{
    const std::string log_name = "6d-of-tracking-of-dumper";

    if (argc != 9)
    {
        std::cerr << "Synopsis: " + log_name + " <dataset_path> <data_format> <rgb_format> <heading_zeros> <index_offset> <camera_width> <camera_height> <output_path>" << std::endl << std::endl;

        std::cerr << "  It is expected that (<camera_width> x <camera_height>) frames of the form %0<heading_zero>d.<rgb_format> are available within <dataset_path>/rgb/. " << std::endl
                  << "  The first frame starts from index (0 + <index_offset>)." << std::endl << std::endl;

        std::cerr << "  It is also expected that a data.<data_format> file respecting RobotsIO::Camera log format is available in <dataset_path>." << std::endl;

        return EXIT_FAILURE;
    }

    const std::string dataset_path{argv[1]};

    const std::string data_format{argv[2]};

    const std::string rgb_format{argv[3]};

    std::size_t heading_zeros;
    if (!parse_size_t(argv, 4, "heading_zeros", heading_zeros))
        return EXIT_FAILURE;

    std::size_t index_offset;
    if (!parse_size_t(argv, 5, "index_offset", index_offset))
        return EXIT_FAILURE;

    std::size_t camera_width;
    if (!parse_size_t(argv, 6, "camera_width", camera_width))
        return EXIT_FAILURE;

    std::size_t camera_height;
    if (!parse_size_t(argv, 7, "camera_height", camera_height))
        return EXIT_FAILURE;

    std::string output_path{argv[8]};
    if (output_path.back() != '/')
        output_path += '/';

    /* Log parameters. */
    std::cout << "Running with:" << std::endl
              << "    Rgb frames in: " << (dataset_path.back() != '/' ? dataset_path + "/rgb/" : dataset_path + "rgb/")
              << "%" << heading_zeros << "d." << rgb_format << std::endl
              << "    Rgb frames resolution: " << camera_width << " x " << camera_height << std::endl
              << "    Output path: " << output_path << std::endl;

    /* Fields regarding depth and camera parameters are not filled as not required. */
    const std::string log_path = "";
    const std::string rgb_path = "rgb/";
    const std::string depth_path = "";
    const std::string depth_format = "";
    const double fx = 0.0;
    const double cx = 0.0;
    const double fy = 0.0;
    const double cy = 0.0;
    auto camera = std::unique_ptr<Camera>
    (
        new DatasetCamera(dataset_path, log_path, rgb_path, depth_path, data_format, rgb_format, depth_format, heading_zeros, index_offset, camera_width, camera_height, fx, cx, fy, cy)
    );

    /* Initialize camera measurement. */
    auto camera_measurement = std::make_shared<CameraMeasurement>(std::move(camera));

    /* Initialize optical flow source. */
    ImageOpticalFlowNVOF flow_source(camera_measurement, ImageOpticalFlowNVOF::NVOFPerformance::Slow, true);

    /* Process frames. */
    while(camera_measurement->freeze(CameraMeasurementType::RGB))
    {
        std::cout << "Processing frame # " << camera_measurement->camera_frame_index() << std::endl;

        bool valid_flow = false;
        cv::Mat flow;
        flow_source.step_frame();
        std::tie(valid_flow, flow) = flow_source.flow(false);

        if (valid_flow)
        {
            /* Compose output path. */
            std::ostringstream file_name;
            file_name << std::setw(heading_zeros) << std::setfill('0') << camera_measurement->camera_frame_index() << ".float";

            /* Save frame. */
            save_flow(flow, output_path + file_name.str());
        }
    }

    std::cout << std::endl << "Processing completed." << std::endl;

    return EXIT_SUCCESS;
}
