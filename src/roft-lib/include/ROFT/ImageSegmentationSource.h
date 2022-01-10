/*
 * Copyright (C) 2022 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#ifndef ROFT_IMAGESEGMENTATIONSOURCE_H
#define ROFT_IMAGESEGMENTATIONSOURCE_H

#include <opencv2/opencv.hpp>

#include <string>

namespace ROFT {
    class ImageSegmentationSource;
}


class ROFT::ImageSegmentationSource
{
public:

    virtual ~ImageSegmentationSource();

    virtual bool reset();

    virtual bool step_frame();

    // virtual bool set_frame(const std::size_t& index);

    virtual bool is_stepping_required() const = 0;

    virtual void reset_data_loading_time();

    virtual double get_data_loading_time() const;

    virtual int get_frames_between_iterations() const;

    virtual std::pair<bool, cv::Mat> segmentation(const bool& blocking) = 0;

private:

    const std::string log_name_ = "ImageSegmentationSource";
};

#endif /* ROFT_IMAGESEGMENTATIONSOURCE_H */
