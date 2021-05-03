/*
 * Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#include <SuperimposeMesh/SICAD.h>

#include <opencv2/opencv.hpp>

#include <iostream>
#include <map>

#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>


std::map<std::string, float> extract_intrinsics(const pybind11::dict& dictionary)
{
    std::map<std::string, float> intrinsics;

    for (std::pair<pybind11::handle, pybind11::handle> item : dictionary)
    {
        std::string key = item.first.cast<std::string>();

        if (key == "name")
            continue;
        else if ((key == "width") or (key == "height"))
            intrinsics[key] = item.second.cast<int>();
        else if ((key == "fx") or (key == "fy") or (key == "cx") or (key == "cy"))
            intrinsics[key] = item.second.cast<float>();
    }

    return intrinsics;
}


void render
(
    const std::string& mesh_path,
    const std::string& rgb_path,
    const std::string& output_path,
    pybind11::dict& cam_intrinsics_pybind,
    const pybind11::array_t<float>& poses_pybind
)
{
    /* Extract camera intrinsics. */
    auto cam_intrinsics = extract_intrinsics(cam_intrinsics_pybind);

    /* Extract view of poses without bound checks for performance. */
    auto poses = poses_pybind.unchecked<2>();

    /* Initialize mesh path. */
    SICAD::ModelPathContainer model_path;
    model_path["object"] = mesh_path;

    /* Initialize renderer. */
    SICAD renderer
    (
        model_path,
        cam_intrinsics["width"],
        cam_intrinsics["height"],
        cam_intrinsics["fx"],
        cam_intrinsics["fy"],
        cam_intrinsics["cx"],
        cam_intrinsics["cy"],
        1
    );
    renderer.setBackgroundOpt(true);
    renderer.setOglToCam({1.0, 0.0, 0.0, static_cast<float>(M_PI)});

    SICAD::ModelPose last_pose;
    bool is_last_pose = false;

    std::cout << "[object_renderer::renderer(). Processing frames." << std::endl;
    std::size_t index = 0;
    cv::Mat image;
    while (!(image = cv::imread(rgb_path + "/" + std::to_string(index) + ".png", cv::IMREAD_COLOR)).empty())
    {
        if (PyErr_CheckSignals() != 0)
            throw pybind11::error_already_set();

        /* Render the object mesh on grayed out background. */
        cv::cvtColor(image, image, cv::COLOR_RGB2GRAY);
        cv::cvtColor(image, image, cv::COLOR_GRAY2RGB);

        float x = poses(index, 0);
        float y = poses(index, 1);
        float z = poses(index, 2);
        bool invalid_pose = (x == 0.0) && (y == 0.0) && (z == 0.0);

        if ((!invalid_pose) || (is_last_pose))
        {
            SICAD::ModelPose pose;
            SICAD::ModelPoseContainer pose_container;

            if (invalid_pose)
                pose = last_pose;
            else
            {
                pose.push_back(x); // x
                pose.push_back(y); // y
                pose.push_back(z); // z
                pose.push_back(poses(index, 3)); // axis x
                pose.push_back(poses(index, 4)); // axis y
                pose.push_back(poses(index, 5)); // axis z
                pose.push_back(poses(index, 6)); // axis angle
            }
            pose_container.emplace("object", pose);

            /* Camera pose placeholder. */
            double cam_x [4] = {0.0, 0.0, 0.0};
            double cam_o [4] = {1.0, 0.0, 0.0, 0.0};

            /* Render scene. */
            bool outcome = renderer.superimpose(pose_container, cam_x, cam_o, image);

            if (!outcome)
                throw std::runtime_error("object_renderer::renderer(). Error: error while trying to render the image.");

            cv::cvtColor(image, image, cv::COLOR_RGB2BGR);

            is_last_pose = true;
            last_pose = pose;
        }

        /* Save to file. */
        cv::imwrite(output_path + "/" + std::to_string(index) + ".png", image);

        index++;
    }
}


PYBIND11_MODULE(object_renderer, m)
{
    m.doc() = "Object renderer";
    m.def("render", &render, "Object renderer");
}
