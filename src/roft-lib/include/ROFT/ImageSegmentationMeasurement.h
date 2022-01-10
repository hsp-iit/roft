/*
 * Copyright (C) 2022 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#ifndef ROFT_IMAGESEGMENTATIONMEASUREMENT_H
#define ROFT_IMAGESEGMENTATIONMEASUREMENT_H

#include <BayesFilters/Data.h>
#include <BayesFilters/MeasurementModel.h>

#include <ROFT/ImageSegmentationSource.h>

#include <opencv2/opencv.hpp>

#include <memory>

namespace ROFT {
    class ImageSegmentationMeasurement;
}


class ROFT::ImageSegmentationMeasurement : public bfl::MeasurementModel
{
public:

    ImageSegmentationMeasurement(std::shared_ptr<ImageSegmentationSource> segmentation_source,  const std::size_t& width = 0, const std::size_t& height = 0);

    ~ImageSegmentationMeasurement();

    bool freeze(const bfl::Data& data = bfl::Data()) override;

    std::pair<bool, bfl::Data> measure(const bfl::Data& data = bfl::Data()) const override;

    std::pair<bool, bfl::Data> predictedMeasure(const Eigen::Ref<const Eigen::MatrixXd>& cur_states) const override;

    std::pair<bool, bfl::Data> innovation(const bfl::Data& predicted_measurements, const bfl::Data& measurements) const override;

    void reset();

    void reset_data_loading_time();

    double get_data_loading_time() const;

protected:
    std::shared_ptr<ImageSegmentationSource> segmentation_source_;

    std::size_t width_;

    std::size_t height_;

    cv::Mat segmentation_;

    /* i.e. whether segmentation was received at least one time. */
    bool segmentation_available_ = false;

    /* i.e. if the current segmentation_ is new. */
    bool new_segmentation_ = false;

    double data_loading_time_ = 0.0;

    const std::string log_name_ = "ImageSegmentationMeasurement";
};

#endif /* ROFT_IMAGESEGMENTATIONMEASUREMENT_H */
