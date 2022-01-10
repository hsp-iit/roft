/*
 * Copyright (C) 2022 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#ifndef ROFT_OPTICALFLOWSOURCE_H
#define ROFT_OPTICALFLOWSOURCE_H

#include <opencv2/opencv.hpp>

#include <string>

namespace ROFT {
    class ImageOpticalFlowSource;
}


class ROFT::ImageOpticalFlowSource
{
public:

    virtual ~ImageOpticalFlowSource();

    virtual bool reset();

    virtual bool step_frame();

    // virtual bool set_frame(const std::size_t& index);

    virtual bool is_stepping_required() const = 0;

    virtual double get_data_loading_time() const;

    virtual std::tuple<bool, cv::Mat> flow(const bool& blocking) = 0;

    /* A flow frame can be per-pixel or block-wise with blocks of size grid_size x grid_size. */
    virtual std::size_t get_grid_size() const = 0;

    /* Values of a flow frame might require additional scaling. */
    virtual float get_scaling_factor() const = 0;

    /* A flow frame can be of type CV_32FC2 or CV_16SC2. */
    virtual int get_matrix_type() const = 0;

private:

    const std::string log_name_ = "ImageOpticalFlowSource";
};

#endif /* ROFT_IMAGEOPTICALFLOWSOURCE_H */
