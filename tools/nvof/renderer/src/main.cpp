/*
 * Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#include <OTL/DatasetImageOpticalFlow.h>
#include <OTL/OpticalFlowUtilities.h>

#include <sstream>
#include <string>

using namespace OTL;
using namespace OTL::OpticalFlowUtils;


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
    const std::string log_name = "6d-of-tracking-of-renderer";

    if (argc != 8)
    {
        std::cerr << "Synopsis: " + log_name + " <dataset_path> <set> <index_offset> <heading_zeros> <camera_width> <camera_height> <output_path>" << std::endl << std::endl
                  << "  The first frame starts from index (0 + <index_offset>)." << std::endl << std::endl;

        std::cerr << "  It is expected that (<camera_width> x <camera_height>) frames of the form %0<heading_zero>d.float are available within <dataset_path>/optical_flow/<set>/. " << std::endl;

        return EXIT_FAILURE;
    }

    const std::string dataset_path{argv[1]};

    const std::string set{argv[2]};

    std::size_t index_offset;
    if (!parse_size_t(argv, 3, "index_offset", index_offset))
        return EXIT_FAILURE;

    std::size_t heading_zeros;
    if (!parse_size_t(argv, 4, "heading_zeros", heading_zeros))
        return EXIT_FAILURE;

    std::size_t camera_width;
    if (!parse_size_t(argv, 5, "camera_width", camera_width))
        return EXIT_FAILURE;

    std::size_t camera_height;
    if (!parse_size_t(argv, 6, "camera_height", camera_height))
        return EXIT_FAILURE;

    std::string output_path{argv[7]};
    if (output_path.back() != '/')
        output_path += '/';

    /* Log parameters. */
    std::cout << "Running with:" << std::endl
              << "    Flow frames in: " << dataset_path << "/optical_flow/" << set << "/"
              << "%" << heading_zeros << "d.float" << std::endl
              << "    Index offset: " << index_offset << std::endl
              << "    Frames resolution: " << camera_width << " x " << camera_height << std::endl
              << "    Output path: " << output_path << std::endl;

    /* Instantiate dataset optical flow. */
    auto dataset_flow = DatasetImageOpticalFlow(dataset_path, set, camera_width, camera_height, heading_zeros, index_offset);

    /* Process frames. */
    std::size_t frame_counter = 0;
    while(true)
    {
        dataset_flow.step_frame();

        bool valid_flow;
        cv::Mat flow;
        std::tie(valid_flow, flow) = dataset_flow.flow(false);

        if (!valid_flow)
            break;

        /* Draw flow. */
        bool valid_draw;
        cv::Mat draw;
        std::tie(valid_draw, draw) = draw_flow(flow);
        if (!valid_flow)
        {
            std::cerr << "Cannot draw optical flow." << std::endl;
            return EXIT_FAILURE;
        }

        /* Compose output path. */
        std::ostringstream file_name;
        file_name << std::setw(heading_zeros) << std::setfill('0') << frame_counter << ".jpg";

        /* Save frame. */
        cv::imwrite(output_path + file_name.str(), draw);

        frame_counter++;
    }

    std::cout << std::endl << "Processing completed." << std::endl;

    return EXIT_SUCCESS;
}
