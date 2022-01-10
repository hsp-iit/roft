/*
 * Copyright (C) 2022 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#ifndef ROFT_MESHRESOURCE_H
#define ROFT_MESHRESOURCE_H

#include <ROFT/ModelParameters.h>

#include <Eigen/Dense>

#include <memory>
#include <string>

namespace ROFT {
    class MeshResource;
}


class ROFT::MeshResource
{
public:
    MeshResource(const std::string& name, const std::string& set);

    MeshResource(const ROFT::ModelParameters& model_parameters);

    virtual ~MeshResource();

    virtual const std::string& as_string() const;

private:
    std::string data_;

    const std::string log_name_ = "MeshResource";
};

#endif /* ROFT_MESHRESOURCE_H */
