/*
 * Copyright (C) 2022 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 *
 *
 * Part of this code is taken from https://github.com/robotology/superimpose-mesh-lib/commits/impl/depth
 *
 * This is the original BSD 3-Clause LICENSE the original code was provided with:
 *
 * Copyright (c) 2016-2019, Istituto Italiano di Tecnologia (IIT) All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
 * - Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
 * - Redistributions in binary form must reproduce the above copyright notice this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 * - Neither the name of the organization nor the names of its contributors may be used to endorse or promote products derived from this software without
 *   specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL CLAUDIO FANTACCI BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <ROFT/SICADShader.h>

#include <cmrc/cmrc.hpp>
CMRC_DECLARE(resources);

#include <fstream>
#include <sstream>
#include <iostream>

using namespace ROFT;


SICADShader::SICADShader(const std::string& vertex_shader_path, const std::string& fragment_shader_path)
{
    std::string sourcecode_vertex_shader;
    std::string sourcecode_fragmentshader;

    /* Retrieve the vertex/fragment source code from path. */
    try
    {
        auto cmrc_fs = cmrc::resources::get_filesystem();

        if (cmrc_fs.exists(vertex_shader_path)   && cmrc_fs.is_file(vertex_shader_path) &&
            cmrc_fs.exists(fragment_shader_path) && cmrc_fs.is_file(fragment_shader_path))
        {
            auto vertex_shader_cmrc_file = cmrc_fs.open(vertex_shader_path);
            sourcecode_vertex_shader.assign(vertex_shader_cmrc_file.cbegin(), vertex_shader_cmrc_file.cend());


            auto fragment_shader_cmrc_file = cmrc_fs.open(fragment_shader_path);
            sourcecode_fragmentshader.assign(fragment_shader_cmrc_file.cbegin(), fragment_shader_cmrc_file.cend());
        }
        else
        {
            std::ifstream file_vertex_shader;
            file_vertex_shader.exceptions(std::ifstream::badbit);

            file_vertex_shader.open(vertex_shader_path);

            std::stringstream vertex_shader_stream;
            vertex_shader_stream << file_vertex_shader.rdbuf();

            file_vertex_shader.close();


            std::ifstream file_fragment_shader;
            file_fragment_shader.exceptions(std::ifstream::badbit);

            file_fragment_shader.open(fragment_shader_path);

            std::stringstream fragment_shader_stream;
            fragment_shader_stream << file_fragment_shader.rdbuf();

            file_fragment_shader.close();


            sourcecode_vertex_shader = vertex_shader_stream.str();
            sourcecode_fragmentshader = fragment_shader_stream.str();
        }
    }
    catch (const std::ifstream::failure& e)
    {
        throw std::runtime_error("ERROR::SHADER::CTOR\nERROR:\n\tCould not read shader source code.\n" + std::string(e.what()));
    }


    /* Compile shaders. */
    const GLchar* ptr_sourcecode_vertex_shader = sourcecode_vertex_shader.c_str();
    const GLchar* ptr_sourcecode_fragment_shader = sourcecode_fragmentshader.c_str();

    GLuint vertex;
    GLuint fragment;
    GLint success;
    GLchar info_log[512];

    /* Vertex Shader. */
    vertex = glCreateShader(GL_VERTEX_SHADER);
    glShaderSource(vertex, 1, &ptr_sourcecode_vertex_shader, NULL);
    glCompileShader(vertex);

    /* Print compile errors if any. */
    glGetShaderiv(vertex, GL_COMPILE_STATUS, &success);
    if (!success)
    {
        glGetShaderInfoLog(vertex, 512, NULL, info_log);
        throw std::runtime_error("ERROR::SHADER::CTOR\nERROR:\n\tVertex shader program compilation error.\nLOG:\n\t" + std::string(info_log));
    };


    /* Fragment Shader. */
    fragment = glCreateShader(GL_FRAGMENT_SHADER);
    glShaderSource(fragment, 1, &ptr_sourcecode_fragment_shader, NULL);
    glCompileShader(fragment);

    /* Print compile errors if any. */
    glGetShaderiv(fragment, GL_COMPILE_STATUS, &success);
    if (!success)
    {
        glGetShaderInfoLog(vertex, 512, NULL, info_log);
        throw std::runtime_error("ERROR::SHADER::CTOR\nERROR:\n\tFragment shader program compilation error.\nLOG:\n\t" + std::string(info_log));
    };


    /* Shader Program. */
    shader_program_id_ = glCreateProgram();
    glAttachShader(shader_program_id_, vertex);
    glAttachShader(shader_program_id_, fragment);
    glLinkProgram(shader_program_id_);

    /* Print linking errors if any. */
    glGetProgramiv(shader_program_id_, GL_LINK_STATUS, &success);
    if (!success)
    {
        glGetProgramInfoLog(shader_program_id_, 512, NULL, info_log);
        throw std::runtime_error("ERROR::SHADER::CTOR\nERROR:\n\tShader program link fail.\nLOG:\n\t" + std::string(info_log));
    }

    /* Delete the shaders as they're linked into our program now and no longer necessery. */
    glDeleteShader(vertex);
    glDeleteShader(fragment);
}


void SICADShader::install()
{
    glUseProgram(shader_program_id_);
}


void SICADShader::uninstall()
{
    glUseProgram(0);
}
