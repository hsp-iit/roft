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

#include <SuperimposeMesh/Shader.h>

#include <ROFT/SICADModel.h>

#include <sstream>
#include <iostream>

#include <assimp/Importer.hpp>
#include <assimp/postprocess.h>

#include <glm/glm.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace ROFT;


SICADModel::SICADModel(const GLchar* path)
{
    loadModel(path);
}


SICADModel::SICADModel(const std::basic_istream<char>* model_stream)
{
    loadModel(model_stream);
}


void SICADModel::Draw(SICADShader shader)
{
    Shader* shader_ptr = (Shader*)&shader;
    for(GLuint i = 0; i < meshes_.size(); i++)
    {
        meshes_[i].Draw(*shader_ptr);
    }
}


bool SICADModel::has_texture()
{
    return (textures_loaded_.size() > 0 ? true : false);
}


void SICADModel::loadModel(std::string path)
{
    Assimp::Importer import;
    const aiScene* scene = import.ReadFile(path, aiProcess_Triangulate | aiProcess_FlipUVs);

    if(!scene || scene->mFlags == AI_SCENE_FLAGS_INCOMPLETE || !scene->mRootNode)
    {
        std::cerr << "ERROR::ASSIMP::" << import.GetErrorString() << std::endl;
        return;
    }

    size_t foundpos = path.find_last_of('/');
    if (foundpos == std::string::npos)
    {
       directory_ = ".";
    }
    else
    {
       directory_ = path.substr(0, foundpos);
    }

    processNode(scene->mRootNode, scene);
}


void SICADModel::loadModel(const std::basic_istream<char>* model_stream)
{
    std::ostringstream sstream;
    sstream << model_stream->rdbuf();
    const std::string model_data(sstream.str());
    const char* model_data_pointer = model_data.c_str();

    Assimp::Importer import;
    const aiScene* scene = import.ReadFileFromMemory(model_data_pointer, model_data.length(), aiProcess_Triangulate | aiProcess_FlipUVs);

    processNode(scene->mRootNode, scene);
}


void SICADModel::processNode(aiNode* node, const aiScene* scene)
{
    /* Process all the node's meshes (if any). */
    for (GLuint i = 0; i < node->mNumMeshes; ++i)
    {
        aiMesh* mesh = scene->mMeshes[node->mMeshes[i]];
        meshes_.push_back(processMesh(mesh, scene));
    }

    /* Then do the same for each of its children. */
    for (GLuint i = 0; i < node->mNumChildren; ++i)
    {
        processNode(node->mChildren[i], scene);
    }
}


Mesh SICADModel::processMesh(aiMesh* mesh, const aiScene* scene)
{
    std::vector<Mesh::Vertex> vertices;
    std::vector<GLuint> indices;
    std::vector<Mesh::Texture> textures;

    /* Process vertices. */
    for (GLuint i = 0; i < mesh->mNumVertices; ++i)
    {
        Mesh::Vertex vertex;

        /* Process vertex positions, normals and texture coordinates. */
        vertex.Position = glm::vec3(mesh->mVertices[i].x, mesh->mVertices[i].y, mesh->mVertices[i].z);
        vertex.Normal = glm::vec3(mesh->mNormals[i].x,  mesh->mNormals[i].y,  mesh->mNormals[i].z);

        /* Does the mesh contain texture coordinates? */
        if (mesh->mTextureCoords[0])
        {
            vertex.TexCoords = glm::vec2(mesh->mTextureCoords[0][i].x, mesh->mTextureCoords[0][i].y);
        }
        else
        {
            vertex.TexCoords = glm::vec2(0.0f, 0.0f);
        }

        vertices.push_back(vertex);
    }

    /* Process indices. */
    for (GLuint i = 0; i < mesh->mNumFaces; ++i)
    {
        aiFace face = mesh->mFaces[i];
        for (GLuint j = 0; j < face.mNumIndices; ++j)
        {
            indices.push_back(face.mIndices[j]);
        }
    }

    /* Process textures. */
    /* The texture code is taken as-is. The tutorial on texture was skipped. */
    if (mesh->mMaterialIndex > 0)
    {
        aiMaterial* material = scene->mMaterials[mesh->mMaterialIndex];

        std::vector<Mesh::Texture> diffuseMaps = loadMaterialTextures(material, aiTextureType_DIFFUSE, "texture_diffuse");
        textures.insert(textures.end(), diffuseMaps.begin(), diffuseMaps.end());

        std::vector<Mesh::Texture> specularMaps = loadMaterialTextures(material, aiTextureType_SPECULAR, "texture_specular");
        textures.insert(textures.end(), specularMaps.begin(), specularMaps.end());
    }

    return Mesh(vertices, indices, textures);
}


std::vector<Mesh::Texture> SICADModel::loadMaterialTextures(aiMaterial* mat, aiTextureType type, std::string typeName)
{
    std::vector<Mesh::Texture> textures;
    for (GLuint i = 0; i < mat->GetTextureCount(type); ++i)
    {
        aiString str;
        mat->GetTexture(type, i, &str);
        GLboolean skip = false;
        for (GLuint j = 0; j < textures_loaded_.size(); ++j)
        {
            if (textures_loaded_[j].path == str)
            {
                textures.push_back(textures_loaded_[j]);
                skip = true;
                break;
            }
        }

        /* If texture hasn't been loaded already, load it. */
        if (!skip)
        {
            Mesh::Texture texture;

            texture.id = TextureFromFile(str.C_Str(), directory_);
            texture.type = typeName;
            texture.path = str;
            textures.push_back(texture);

            /* Add to loaded textures. */
            textures_loaded_.push_back(texture);
        }
    }

    return textures;
}


GLint SICADModel::TextureFromFile(const char* path, std::string directory)
{
    std::string filename = directory + "/" + std::string(path);
    cv::Mat image = cv::imread(filename, cv::IMREAD_ANYCOLOR);

    /* Generate texture ID and load texture data. */
    GLuint textureID;
    glGenTextures(1, &textureID);

    /* Assign texture to ID. */
    glBindTexture(GL_TEXTURE_2D, textureID);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, image.cols, image.rows, 0, GL_RGB, GL_UNSIGNED_BYTE, image.ptr());
    glGenerateMipmap(GL_TEXTURE_2D);

    /* Parameters. */
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glBindTexture(GL_TEXTURE_2D, 0);

    return textureID;
}
