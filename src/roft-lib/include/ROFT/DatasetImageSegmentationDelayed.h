/*
 * Copyright (C) 2022 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#ifndef ROFT_DATASETIMAGESEGMENTATIONDELAYED_H
#define ROFT_DATASETIMAGESEGMENTATIONDELAYED_H

#include <ROFT/DatasetImageSegmentation.h>

namespace ROFT {
    class DatasetImageSegmentationDelayed;
}


class ROFT::DatasetImageSegmentationDelayed : public ROFT::DatasetImageSegmentation
{
public:
    DatasetImageSegmentationDelayed(const float& fps, const float& simulated_fps, const bool simulate_inference_time, const std::string& dataset_path, const std::string& format, const std::size_t width, const std::size_t height, const std::string& segmentation_set, const ROFT::ModelParameters& model_parameters, const std::size_t heading_zeros = 0, const std::size_t index_offset = 0);

    ~DatasetImageSegmentationDelayed();

    std::pair<bool, cv::Mat> segmentation(const bool& blocking) override;

    void reset_data_loading_time() override;

    double get_data_loading_time() const override;

    int get_frames_between_iterations() const override;

private:
    const float fps_;

    const float simulated_fps_;

    const bool simulate_inference_time_;

    const int head_0_;

    const int delay_;

    double data_loading_time_ = 0.0;
};

#endif /* ROFT_DATASETIMAGESEGMENTATIONDELAYED_H */
