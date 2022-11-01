/*
 * Copyright (C) 2022 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#ifndef ROFT_DATASETIMAGESEGMENTATION_H
#define ROFT_DATASETIMAGESEGMENTATION_H

#include <ROFT/ModelParameters.h>

#include <RobotsIO/Utils/Segmentation.h>

#include <memory>
#include <string>

namespace ROFT {
    class DatasetImageSegmentation;
}


class ROFT::DatasetImageSegmentation : public RobotsIO::Utils::Segmentation
{
public:
    DatasetImageSegmentation(const std::string& dataset_path, const std::string& format, const std::size_t width, const std::size_t height, const std::string& segmentation_set, const ROFT::ModelParameters& model_parameters, const std::size_t heading_zeros = 0, const std::size_t index_offset = 0, const bool simulate_missing_detections = false);

    ~DatasetImageSegmentation();

    bool reset() override;

    bool step_frame() override;

    bool is_stepping_required() const override;

    void reset_data_loading_time() override;

    double get_data_loading_time() const override;

    std::pair<bool, cv::Mat> segmentation(const bool& blocking) override;

protected:
    std::string compose_file_name(const int& index, const std::size_t& number_of_digits);

    std::pair<bool, cv::Mat> read_file(const std::size_t& file_name);

    std::string dataset_path_;

    const std::string format_;

    const std::size_t width_;

    const std::size_t height_;

    std::string object_name_;

    int head_;

    const std::size_t index_offset_;

    const std::size_t heading_zeros_;

    const bool simulate_missing_detections_;

    std::pair<bool, cv::Mat> output_;

    double data_loading_time_ = 0.0;

    const std::string log_name_ = "DatasetImageSegmentation";
};

#endif /* ROFT_DATASETIMAGESEGMENTATION_H */
