/*
 * Copyright (C) 2022 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#ifndef ROFT_DATASETIMAGEOPTICALFLOW_H
#define ROFT_DATASETIMAGEOPTICALFLOW_H

#include <ROFT/ImageOpticalFlowSource.h>

#include <memory>
#include <string>

namespace ROFT {
    class DatasetImageOpticalFlow;
}


class ROFT::DatasetImageOpticalFlow : public ROFT::ImageOpticalFlowSource
{
public:
    DatasetImageOpticalFlow(const std::string& dataset_path, const std::string& set, const std::size_t width, const std::size_t height, const std::size_t& heading_zeros = 0, const std::size_t& index_offset = 0);

    ~DatasetImageOpticalFlow();

    bool reset() override;

    bool step_frame() override;

    bool is_stepping_required() const override;

    std::size_t get_grid_size() const override;

    float get_scaling_factor() const override;

    double get_data_loading_time() const override;

    int get_matrix_type() const override;

    std::tuple<bool, cv::Mat> flow(const bool& blocking) override;

private:
    std::string compose_file_name(const int& index, const std::size_t& number_of_digits);

    std::string dataset_path_;

    const std::size_t width_;

    const std::size_t height_;

    int head_;

    const std::size_t index_offset_;

    const std::size_t heading_zeros_;

    std::size_t grid_size_;

    float scaling_factor_;

    int matrix_type_;

    std::tuple<bool, cv::Mat> output_;

    double data_loading_time_ = 0.0;

    const std::string log_name_ = "DatasetImageOpticalFlow";
};

#endif /* ROFT_DATASETIMAGEOPTICALFLOW_H */
