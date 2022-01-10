/*
 * Copyright (C) 2022 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#include <ROFT/MeshResource.h>

#include <fstream>
#include <sstream>

#include <cmrc/cmrc.hpp>
CMRC_DECLARE(otl_resources);

using namespace ROFT;


MeshResource::MeshResource(const std::string& name, const std::string& set)
{
    const std::string virtual_path = "__prc/meshes/" + set + "/" + name + ".obj";

    auto cmrc_fs = cmrc::otl_resources::get_filesystem();

    if (!(cmrc_fs.exists(virtual_path) && cmrc_fs.is_file(virtual_path)))
        throw(std::runtime_error(log_name_ + "::ctor. Cannot find requested mesh among available resources."));

    auto mesh_cmrc_file = cmrc_fs.open(virtual_path);
    data_.assign(mesh_cmrc_file.cbegin(), mesh_cmrc_file.cend());
}


MeshResource::MeshResource(const ROFT::ModelParameters& model_parameters)
{
    if (model_parameters.use_internal_db())
    {
        const std::string virtual_path = "__prc/meshes/" + model_parameters.internal_db_name() + "/" + model_parameters.name() + ".obj";

        auto cmrc_fs = cmrc::otl_resources::get_filesystem();

        if (!(cmrc_fs.exists(virtual_path) && cmrc_fs.is_file(virtual_path)))
            throw(std::runtime_error(log_name_ + "::ctor. Cannot find requested mesh among available resources."));

        auto mesh_cmrc_file = cmrc_fs.open(virtual_path);
        data_.assign(mesh_cmrc_file.cbegin(), mesh_cmrc_file.cend());
    }
    else
    {
        std::ifstream in;
        in.open(model_parameters.mesh_external_path());
        if (!in.is_open())
            throw(std::runtime_error(log_name_ + "::ctor. Cannot open model from external path " + model_parameters.mesh_external_path() + "."));

        std::stringstream buffer;
        buffer << in.rdbuf();
        in.close();

        data_ = buffer.str();
    }
}


MeshResource::~MeshResource()
{}


const std::string& MeshResource::as_string() const
{
    return data_;
}
