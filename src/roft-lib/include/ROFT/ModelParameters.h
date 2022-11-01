/*
 * Copyright (C) 2022 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#ifndef ROFT_MODELPARAMETERS_H
#define ROFT_MODELPARAMETERS_H

#include <RobotsIO/Utils/Parameters.h>

namespace ROFT {
    class ModelParameters;
}


class ROFT::ModelParameters : public RobotsIO::Utils::Parameters
{
public:
    ModelParameters();

    virtual ~ModelParameters();

    robots_io_accessor(ModelParameters);

    robots_io_declare_std_field(ModelParameters, string, name);

    robots_io_declare_field(ModelParameters, bool, use_internal_db);

    robots_io_declare_std_field(ModelParameters, string, internal_db_name);

    robots_io_declare_std_field(ModelParameters, string, mesh_external_path);

    robots_io_declare_std_field(ModelParameters, string, textured_mesh_external_path);

    robots_io_declare_std_field(ModelParameters, string, cloud_external_path);
};

#endif /* ROFT_MODELPARAMETERS_H */
