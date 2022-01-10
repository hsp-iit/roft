/*
 * Copyright (C) 2022 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#include <ROFT/ModelParameters.h>

using namespace ROFT;


robots_io_accessor_impl(ModelParameters);


robots_io_declare_std_field_impl(ModelParameters, string, name);


robots_io_declare_field_impl(ModelParameters, bool, use_internal_db);


robots_io_declare_std_field_impl(ModelParameters, string, internal_db_name);


robots_io_declare_std_field_impl(ModelParameters, string, mesh_external_path);

robots_io_declare_std_field_impl(ModelParameters, string, cloud_external_path);


ModelParameters::ModelParameters()
{
    /* Set default values */
    name("");

    use_internal_db(false);

    internal_db_name("");

    mesh_external_path("");

    cloud_external_path("");
}


ModelParameters::~ModelParameters()
{}
