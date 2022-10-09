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

#include <ROFT/SICAD.h>

#include <iostream>
#include <exception>
#include <string>

#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>

#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <opencv2/imgproc/imgproc.hpp>

using namespace ROFT;

int SICAD::class_counter_ = 0;
GLsizei SICAD::renderbuffer_size_ = 0;


SICAD::SICAD
(
    const ModelPathContainer& objfile_map,
    const GLsizei cam_width,
    const GLsizei cam_height,
    const GLfloat cam_fx,
    const GLfloat cam_fy,
    const GLfloat cam_cx,
    const GLfloat cam_cy
) :
    SICAD(cam_width, cam_height, cam_fx, cam_fy, cam_cx, cam_cy, 1, "__prc/shader", { 1.0f, 0.0f, 0.0f, 0.0f }, objfile_map)
{ }


SICAD::SICAD
(
    const ModelStreamContainer& objstream_map,
    const GLsizei cam_width,
    const GLsizei cam_height,
    const GLfloat cam_fx,
    const GLfloat cam_fy,
    const GLfloat cam_cx,
    const GLfloat cam_cy
) :
    SICAD(cam_width, cam_height, cam_fx, cam_fy, cam_cx, cam_cy, 1, "__prc/shader", { 1.0f, 0.0f, 0.0f, 0.0f }, ModelPathContainer(), objstream_map)
{ }


SICAD::SICAD
(
    const ModelPathContainer& objfile_map,
    const GLsizei cam_width,
    const GLsizei cam_height,
    const GLfloat cam_fx,
    const GLfloat cam_fy,
    const GLfloat cam_cx,
    const GLfloat cam_cy,
    const GLint num_images
) :
    SICAD(cam_width, cam_height, cam_fx, cam_fy, cam_cx, cam_cy, num_images, "__prc/shader", { 1.0f, 0.0f, 0.0f, 0.0f }, objfile_map)
{ }


SICAD::SICAD
(
    const ModelStreamContainer& objstream_map,
    const GLsizei cam_width,
    const GLsizei cam_height,
    const GLfloat cam_fx,
    const GLfloat cam_fy,
    const GLfloat cam_cx,
    const GLfloat cam_cy,
    const GLint num_images
) :
    SICAD(cam_width, cam_height, cam_fx, cam_fy, cam_cx, cam_cy, num_images, "__prc/shader", { 1.0f, 0.0f, 0.0f, 0.0f }, ModelPathContainer(), objstream_map)
{ }


SICAD::SICAD
(
    const ModelPathContainer& objfile_map,
    const GLsizei cam_width,
    const GLsizei cam_height,
    const GLfloat cam_fx,
    const GLfloat cam_fy,
    const GLfloat cam_cx,
    const GLfloat cam_cy,
    const GLint num_images,
    const std::string& shader_folder
) :
    SICAD(cam_width, cam_height, cam_fx, cam_fy, cam_cx, cam_cy, num_images, shader_folder, { 1.0f, 0.0f, 0.0f, 0.0f }, objfile_map)
{ }


SICAD::SICAD
(
    const GLsizei cam_width,
    const GLsizei cam_height,
    const GLfloat cam_fx,
    const GLfloat cam_fy,
    const GLfloat cam_cx,
    const GLfloat cam_cy,
    const GLint num_images,
    const std::string& shader_folder,
    const std::vector<float>& ogl_to_cam,
    const ModelPathContainer& objfile_map,
    const ModelStreamContainer& objstream_map
) :
    cam_fx_(cam_fx),
    cam_fy_(cam_fy),
    cam_cx_(cam_cx),
    cam_cy_(cam_cy)
{
    if (ogl_to_cam.size() != 4)
        throw std::runtime_error("ERROR::SICAD::CTOR\nERROR:\n\tWrong size provided for ogl_to_cam.\n\tShould be 4, was given " + std::to_string(ogl_to_cam.size()) + ".");


    std::cout << log_ID_ << "Start setting up OpenGL rendering facilities." << std::endl;


    /* Initialize GLFW. */
    if (glfwInit() == GL_FALSE)
        throw std::runtime_error("ERROR::SICAD::CTOR\nERROR:\n\tFailed to initialize GLFW.");


    /* Set context properties by "hinting" specific (property, value) pairs. */
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_RESIZABLE, GL_FALSE);
    glfwWindowHint(GLFW_VISIBLE, GL_FALSE);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
    glfwWindowHint(GLFW_CONTEXT_RELEASE_BEHAVIOR, GLFW_RELEASE_BEHAVIOR_NONE);
#ifdef GLFW_MAC
    glfwWindowHint(GLFW_COCOA_RETINA_FRAMEBUFFER, GL_FALSE);
#endif


    /* Create window to create context and enquire OpenGL for the maximum size of the renderbuffer */
    window_ = glfwCreateWindow(1, 1, "OpenGL window", nullptr, nullptr);
    if (window_ == nullptr)
    {
        glfwTerminate();
        throw std::runtime_error("ERROR::SICAD::CTOR\nERROR:\n\tFailed to create GLFW window.");
    }

    /* Make the OpenGL context of window the current one handled by this thread. */
    glfwMakeContextCurrent(window_);


    /* Enquire GPU for maximum renderbuffer size (both width and height) of the default framebuffer */
    glGetIntegerv(GL_MAX_RENDERBUFFER_SIZE, &renderbuffer_size_);
    std::cout << log_ID_ << "Max renderbuffer size is " + std::to_string(renderbuffer_size_) + "x" + std::to_string(renderbuffer_size_) + " size." << std::endl;


    /* Given image size */
    image_width_ = cam_width;
    image_height_ = cam_height;
    std::cout << log_ID_ << "Given image size " + std::to_string(image_width_) + "x" + std::to_string(image_height_) + "." << std::endl;


    /* Compute the maximum number of images that can be rendered conditioned on the maximum renderbuffer size */
    factorize_int(num_images, std::floor(renderbuffer_size_ / image_width_), std::floor(renderbuffer_size_ / image_height_), tiles_cols_, tiles_rows_);
    tiles_num_ = tiles_rows_ * tiles_cols_;
    std::cout << log_ID_ << "Required to render " + std::to_string(num_images) + " image(s)." << std::endl;
    std::cout << log_ID_ << "Allowed number or rendered images is " + std::to_string(tiles_num_) + " (" + std::to_string(tiles_rows_) + "x" + std::to_string(tiles_cols_) + " grid)." << std::endl;

    /* Set framebuffer size. */
    framebuffer_width_ = image_width_ * tiles_cols_;
    framebuffer_height_ = image_height_ * tiles_rows_;

    /* Set rendered image size. May vary in HDPI monitors. */
    tile_img_width_ = framebuffer_width_ / tiles_cols_;
    tile_img_height_ = framebuffer_height_ / tiles_rows_;
    std::cout << log_ID_ << "The rendered image size is " + std::to_string(tile_img_width_) + "x" + std::to_string(tile_img_height_) + "." << std::endl;


    /* Initialize GLEW to use the OpenGL implementation provided by the videocard manufacturer. */
    glewExperimental = GL_TRUE;
    if (glewInit() != GLEW_OK)
        throw std::runtime_error("ERROR::SICAD::CTOR\nERROR:\n\tFailed to initialize GLEW.");

    /* Swap buffers immediately when calling glfwSwapBuffers, without waiting for a refresh. */
    glfwSwapInterval(0);

    /* Set GL property. */
    glfwPollEvents();
    main_thread_id_ = std::this_thread::get_id();


    std::cout << log_ID_ << "Succesfully set up OpenGL facilities!" << std::endl;


    std::cout << log_ID_ << "Setting up OpenGL shaders, buffers and textures." << std::endl;


    /* Rotation from real camera to OpenGL frame */
    ogl_to_cam_ = glm::mat3(glm::rotate(glm::mat4(1.0f), ogl_to_cam[3], glm::make_vec3(ogl_to_cam.data())));


	/* Create a framebuffer object. */
    glGenFramebuffers(1, &fbo_);
    glBindFramebuffer(GL_FRAMEBUFFER, fbo_);

	/* Create a framebuffer color texture. */
    glGenTextures(1, &texture_color_buffer_);
    glBindTexture(GL_TEXTURE_2D, texture_color_buffer_);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, framebuffer_width_, framebuffer_height_, 0, GL_RGB, GL_UNSIGNED_BYTE, NULL);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST_MIPMAP_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glBindTexture(GL_TEXTURE_2D, 0);

    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, texture_color_buffer_, 0);

	/* Create a framebuffer depth texture. */
	glGenTextures(1, &texture_depth_buffer_);
	glBindTexture(GL_TEXTURE_2D, texture_depth_buffer_);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_R32F, framebuffer_width_, framebuffer_height_, 0, GL_RED, GL_FLOAT, NULL);
	//glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST_MIPMAP_NEAREST);
	//glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glBindTexture(GL_TEXTURE_2D, 0);

	glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT1, GL_TEXTURE_2D, texture_depth_buffer_, 0);

	/* Instruct OpenGL to render to both framebuffer color attachments. */
	unsigned int attachments[3] = { GL_COLOR_ATTACHMENT0, GL_COLOR_ATTACHMENT1 };
	glDrawBuffers(2, attachments);

    /* Create a framebuffer depth texture for depth test. */
    glGenTextures(1, &texture_depthtest_buffer_);
    glBindTexture(GL_TEXTURE_2D, texture_depthtest_buffer_);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT, framebuffer_width_, framebuffer_height_, 0, GL_DEPTH_COMPONENT, GL_UNSIGNED_BYTE, NULL);
    glBindTexture(GL_TEXTURE_2D, 0);

    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, texture_depthtest_buffer_, 0);

    /* Check whether the framebuffer has been completely created or not. */
    if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
        throw std::runtime_error("ERROR::SICAD::CTOR::\nERROR:\n\tCustom framebuffer could not be created.");


    /* Enable depth and scissor test. */
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_SCISSOR_TEST);


    /* Unbind framebuffer. */
    glBindFramebuffer(GL_FRAMEBUFFER, 0);


    /* Crate the vertices for 3D reference frame. */
    glGenVertexArrays(1, &vao_frame_);
    glBindVertexArray(vao_frame_);

    glGenBuffers(1, &vbo_frame_);
    glBindBuffer(GL_ARRAY_BUFFER, vbo_frame_);

    GLfloat vert_frame[] = {// Positions       // Colors
                            0.0f, 0.0f, 0.0f,  1.0f, 0.0f, 0.0f,   // Origin X
                            0.1f, 0.0f, 0.0f,  1.0f, 0.0f, 0.0f,   // End X
                            0.0f, 0.0f, 0.0f,  0.0f, 1.0f, 0.0f,   // Origin Y
                            0.0f, 0.1f, 0.0f,  0.0f, 1.0f, 0.0f,   // End Y
                            0.0f, 0.0f, 0.0f,  0.0f, 0.0f, 1.0f,   // Origin Z
                            0.0f, 0.0f, 0.1f,  0.0f, 0.0f, 1.0f }; // End Z

    glBufferData(GL_ARRAY_BUFFER, sizeof(vert_frame), vert_frame, GL_STATIC_DRAW);

    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(GLfloat), (GLvoid*)(0));
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(GLfloat), (GLvoid*)(3 * sizeof(GLfloat)));

    glEnableVertexAttribArray(0);
    glEnableVertexAttribArray(1);

    glBindVertexArray(0);


    /* Create a background texture. */
    glGenTextures(1, &texture_background_);

    /* Crate the squared support for the backround texture. */
    glGenVertexArrays(1, &vao_background_);
    glBindVertexArray(vao_background_);

    glGenBuffers(1, &vbo_background_);
    glBindBuffer(GL_ARRAY_BUFFER, vbo_background_);

    GLfloat vert_background[] = {// Positions    // Colors            // Texture Coords
                                    1.0f,  1.0f,    1.0f, 0.0f, 0.0f,    1.0f, 1.0f,   // Top Right
                                    1.0f, -1.0f,    0.0f, 1.0f, 0.0f,    1.0f, 0.0f,   // Bottom Right
                                   -1.0f, -1.0f,    0.0f, 0.0f, 1.0f,    0.0f, 0.0f,   // Bottom Left
                                   -1.0f,  1.0f,    1.0f, 1.0f, 0.0f,    0.0f, 1.0f }; // Top Left

    glBufferData(GL_ARRAY_BUFFER, sizeof(vert_background), vert_background, GL_STATIC_DRAW);

    glGenBuffers(1, &ebo_background_);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo_background_);

    GLuint indices[] = { 0, 1, 3,   // First Triangle
                         1, 2, 3 }; // Second Triangle

    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(indices), indices, GL_STATIC_DRAW);

    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 7 * sizeof(GLfloat), (GLvoid*)(0));
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 7 * sizeof(GLfloat), (GLvoid*)(2 * sizeof(GLfloat)));
    glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, 7 * sizeof(GLfloat), (GLvoid*)(5 * sizeof(GLfloat)));

    glEnableVertexAttribArray(0);
    glEnableVertexAttribArray(1);
    glEnableVertexAttribArray(2);

    glBindVertexArray(0);


    /* Crate the Pixel Buffer Objects for reading rendered images and manipulate data directly on GPU. */
    glGenBuffers(2, pbo_);
    unsigned int number_of_channel = 3;

    glBindBuffer(GL_PIXEL_PACK_BUFFER, pbo_[0]);
    glBufferData(GL_PIXEL_PACK_BUFFER, framebuffer_width_ * framebuffer_height_ * number_of_channel, 0, GL_STREAM_READ);

    glBindBuffer(GL_PIXEL_PACK_BUFFER, pbo_[1]);
    glBufferData(GL_PIXEL_PACK_BUFFER, framebuffer_width_ * framebuffer_height_ * number_of_channel, 0, GL_STREAM_READ);

    glBindBuffer(GL_PIXEL_PACK_BUFFER, 0);

    /* FIXME
     * Delete std::nothrow and change try-catch logic.
     */
    /* FIXME
     * Add std::make_unique in an utility header.
     */
    /* Crate background shader program. */
    std::cout << log_ID_ << "Setting up background shader." << std::endl;

    try
    {
        shader_background_ = new (std::nothrow) SICADShader((shader_folder + "/shader_background.vert").c_str(), (shader_folder + "/shader_background.frag").c_str());
    }
    catch (const std::runtime_error& e)
    {
        throw std::runtime_error(e.what());
    }
    if (shader_background_ == nullptr)
        throw std::runtime_error("ERROR::SICAD::CTOR\nERROR:\n\tBackground shader source file not found!");

    std::cout << log_ID_ << "Background shader succesfully set up!" << std::endl;


    /* Crate shader program for mesh model. */
    std::cout << log_ID_ << "Setting up shader for mesh models." << std::endl;

    try
    {
        shader_cad_ = new (std::nothrow) SICADShader((shader_folder + "/shader_model.vert").c_str(), (shader_folder + "/shader_model.frag").c_str());
    }
    catch (const std::runtime_error& e)
    {
        throw std::runtime_error(e.what());
    }
    if (shader_cad_ == nullptr)
        throw std::runtime_error("ERROR::SICAD::CTOR\nERROR:\n\t3D model shader source file not found!");

    std::cout << log_ID_ << "Shader for mesh models succesfully set up!" << std::endl;


    /* Crate shader program for textured model. */
    std::cout << log_ID_ << "Setting up shader for textured mesh model." << std::endl;

    try
    {
        shader_mesh_texture_ = std::unique_ptr<SICADShader>(new SICADShader((shader_folder + "/shader_model.vert").c_str(), (shader_folder + "/shader_model_texture.frag").c_str()));
    }
    catch (const std::runtime_error& e)
    {
        throw std::runtime_error("ERROR::SICAD::CTOR\nERROR:\n\tFailed to create shader program for textured meshes.\n" + std::string(e.what()));
    }

    std::cout << log_ID_ << "Shader for textured mesh models succesfully set up!" << std::endl;


    /* Crate axis frame shader program. */
    std::cout << log_ID_ << "Setting up maxis frame shader." << std::endl;

    try
    {
        shader_frame_ = new (std::nothrow) SICADShader((shader_folder + "/shader_frame.vert").c_str(), (shader_folder + "/shader_frame.frag").c_str());
    }
    catch (const std::runtime_error& e)
    {
        throw std::runtime_error(e.what());
    }
    if (shader_frame_ == nullptr)
        throw std::runtime_error("ERROR::SICAD::CTOR\nERROR:\n\tAxis frame shader source file not found!");

    std::cout << log_ID_ << "Axis frame shader succesfully set up!" << std::endl;


    /* Load models from file streams, if any. */
    for (const ModelStreamElement& pair : objstream_map)
    {
        auto search = model_obj_.find(pair.first);
        if(search == model_obj_.end())
        {
            std::cout << log_ID_ << "Loading " + pair.first + " model for OpenGL rendering from stream." << std::endl;

            model_obj_[pair.first] = new (std::nothrow) SICADModel(pair.second);

            if (model_obj_[pair.first] == nullptr)
                throw std::runtime_error("ERROR::SICAD::CTOR\nERROR:\n\t" + pair.first + " model cannot be loaded from stream!");
        }
        else
        {
            std::cout << log_ID_ << "Skipping " + pair.first + " model for OpenGL rendering. Object name already exists." << std::endl;
            std::cout << log_ID_ << "If you want to update " + pair.first + " model for OpenGL rendering, use the updateModel function." << std::endl;
        }
    }

    /* Load models from file paths, if any. */
    for (const ModelPathElement& pair : objfile_map)
    {
        auto search = model_obj_.find(pair.first);
        if(search == model_obj_.end())
        {
            std::cout << log_ID_ << "Loading " + pair.first + " model for OpenGL rendering from " << pair.second << "." << std::endl;

            model_obj_[pair.first] = new (std::nothrow) SICADModel(pair.second.c_str());

            if (model_obj_[pair.first] == nullptr)
                throw std::runtime_error("ERROR::SICAD::CTOR\nERROR:\n\t" + pair.first + " model file from " + pair.second + " not found!");
        }
        else
        {
            std::cout << log_ID_ << "Skipping " + pair.first + " model for OpenGL rendering. Object name already exists." << std::endl;
            std::cout << log_ID_ << "If you want to update " + pair.first + " model for OpenGL rendering, use the updateModel function." << std::endl;
        }
    }

    back_proj_ = glm::ortho(-1.001f, 1.001f, -1.001f, 1.001f, 0.0f, far_*100.f);

    glfwMakeContextCurrent(nullptr);


    std::cout << log_ID_ << "Succesfully set up OpenGL shaders, buffers and textures." << std::endl;


    /* Set projection matrix */
    std::cout << log_ID_ << "Setting up projection matrix." << std::endl;

    if (!setProjectionMatrix(cam_width, cam_height, cam_fx, cam_fy, cam_cx, cam_cy))
        throw std::runtime_error("ERROR::SICAD::CTOR\nERROR:\n\tFailed to set projection matrix.");


    /* Increase static class counter. */
    ++class_counter_;


    std::cout << log_ID_ << "Initialization completed!" << std::endl;
}


SICAD::~SICAD()
{
    std::cout << log_ID_ << "Deallocating OpenGL resources..." << std::endl;


    glfwMakeContextCurrent(window_);


    for (const ModelElement& pair : model_obj_)
    {
        std::cout << log_ID_ << "Deleting OpenGL "+ pair.first+" model." << std::endl;
        delete pair.second;
    }


	glDeleteTextures(1, &texture_color_buffer_);
	glDeleteTextures(1, &texture_depth_buffer_);
    glDeleteTextures(1, &texture_depthtest_buffer_);
    glDeleteFramebuffers(1, &fbo_);
    glDeleteVertexArrays(1, &vao_background_);
    glDeleteBuffers(1, &ebo_background_);
    glDeleteBuffers(1, &vbo_background_);
    glDeleteVertexArrays(1, &vao_frame_);
    glDeleteBuffers(1, &vbo_frame_);
    glDeleteTextures(1, &texture_background_);
    glDeleteBuffers(2, pbo_);


    std::cout << log_ID_ << "Deleting OpenGL shaders." << std::endl;
    delete shader_background_;
    delete shader_cad_;
    delete shader_frame_;


    std::cout << log_ID_ << "Closing OpenGL window/context." << std::endl;
    glfwSetWindowShouldClose(window_, GL_TRUE);
    glfwMakeContextCurrent(nullptr);


    class_counter_--;
    if (class_counter_ == 0)
    {
        std::cout << log_ID_ << "Terminating GLFW." << std::endl;
        glfwTerminate();
    }


    std::cout << log_ID_ << "OpenGL resource deallocation completed!" << std::endl;
}


bool SICAD::getOglWindowShouldClose()
{
    return (glfwWindowShouldClose(window_) == GL_TRUE ? true : false);
}


void SICAD::setOglWindowShouldClose(bool should_close)
{
    glfwSetWindowShouldClose(window_, GL_TRUE);

    pollOrPostEvent();
}


bool SICAD::superimpose
(
    const ModelPoseContainer& objpos_map,
    const double* cam_x,
    const double* cam_o,
    cv::Mat& img
)
{
    glfwMakeContextCurrent(window_);

    glBindFramebuffer(GL_FRAMEBUFFER, fbo_);

    /* Render in the upper-left-most tile of the render grid */
    glViewport(0,               framebuffer_height_ - tile_img_height_,
               tile_img_width_, tile_img_height_                       );
    glScissor (0,               framebuffer_height_ - tile_img_height_,
               tile_img_width_, tile_img_height_                       );

    /* Clear the colorbuffer. */
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    /* Draw the background picture. */
    if (getBackgroundOpt())
        renderBackground(img);

    /* View mesh filled or as wireframe. */
    setWireframe(getWireframeOpt());

    /* View transformation matrix. */
    glm::mat4 view = getViewTransformationMatrix(cam_x, cam_o);

    /* Install/Use the program specified by the shader. */
    shader_cad_->install();
    glUniformMatrix4fv(glGetUniformLocation(shader_cad_->get_program(), "view"), 1, GL_FALSE, glm::value_ptr(view));
    shader_cad_->uninstall();

    shader_mesh_texture_->install();
    glUniformMatrix4fv(glGetUniformLocation(shader_mesh_texture_->get_program(), "view"), 1, GL_FALSE, glm::value_ptr(view));
    shader_mesh_texture_->uninstall();

    shader_frame_->install();
    glUniformMatrix4fv(glGetUniformLocation(shader_frame_->get_program(), "view"), 1, GL_FALSE, glm::value_ptr(view));
    shader_frame_->uninstall();

    /* Model transformation matrix. */
    for (const ModelPoseContainerElement& pair : objpos_map)
    {
        const double* pose = pair.second.data();

        glm::mat4 model = glm::rotate(glm::mat4(1.0f), static_cast<float>(pose[6]), glm::vec3(static_cast<float>(pose[3]), static_cast<float>(pose[4]), static_cast<float>(pose[5])));
        model[3][0] = pose[0];
        model[3][1] = pose[1];
        model[3][2] = pose[2];

        auto iter_model = model_obj_.find(pair.first);
        if (iter_model != model_obj_.end())
        {
            if ((iter_model->second)->has_texture())
            {
                shader_mesh_texture_->install();
                glUniformMatrix4fv(glGetUniformLocation(shader_mesh_texture_->get_program(), "model"), 1, GL_FALSE, glm::value_ptr(model));

                (iter_model->second)->Draw(*shader_mesh_texture_);

                shader_mesh_texture_->uninstall();
            }
            else
            {
                shader_cad_->install();
                glUniformMatrix4fv(glGetUniformLocation(shader_cad_->get_program(), "model"), 1, GL_FALSE, glm::value_ptr(model));

                (iter_model->second)->Draw(*shader_cad_);

                shader_cad_->uninstall();
            }
        }
        else if (pair.first == "frame")
        {
            shader_frame_->install();
            glUniformMatrix4fv(glGetUniformLocation(shader_frame_->get_program(), "model"), 1, GL_FALSE, glm::value_ptr(model));
            glBindVertexArray(vao_frame_);
            glDrawArrays(GL_LINES, 0, 6);
            glBindVertexArray(0);
            shader_frame_->uninstall();
        }
    }

    /* Read before swap. glReadPixels read the current framebuffer, i.e. the back one. */
    /* See: http://stackoverflow.com/questions/16809833/opencv-image-loading-for-opengl-texture#16812529
       and http://stackoverflow.com/questions/9097756/converting-data-from-glreadpixels-to-opencvmat#9098883 */
    cv::Mat ogl_pixel(framebuffer_height_ / tiles_rows_, framebuffer_width_ / tiles_cols_, CV_8UC3);
    glReadBuffer(GL_COLOR_ATTACHMENT0);
    glPixelStorei(GL_PACK_ALIGNMENT, (ogl_pixel.step & 3) ? 1 : 4);
    glPixelStorei(GL_PACK_ROW_LENGTH, ogl_pixel.step/ogl_pixel.elemSize());
    glReadPixels(0, framebuffer_height_ - tile_img_height_, tile_img_width_, tile_img_height_, GL_BGR, GL_UNSIGNED_BYTE, ogl_pixel.data);

    cv::flip(ogl_pixel, img, 0);

    /* Swap the buffers. */
    glfwSwapBuffers(window_);

    pollOrPostEvent();

    glBindFramebuffer(GL_FRAMEBUFFER, 0);

    glfwMakeContextCurrent(nullptr);

    return true;
}


bool SICAD::superimpose
(
    const ModelPoseContainer& objpos_map,
    const double* cam_x,
    const double* cam_o,
    cv::Mat& img,
    cv::Mat& depth
)
{
    glfwMakeContextCurrent(window_);

    glBindFramebuffer(GL_FRAMEBUFFER, fbo_);

    /* Render in the upper-left-most tile of the render grid */
    glViewport(0, framebuffer_height_ - tile_img_height_,
               tile_img_width_, tile_img_height_);
    glScissor(0, framebuffer_height_ - tile_img_height_,
              tile_img_width_, tile_img_height_);

    /* Clear the colorbuffer. */
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    /* Draw the background picture. */
    if (getBackgroundOpt())
        renderBackground(img);

    /* View mesh filled or as wireframe. */
    setWireframe(getWireframeOpt());

    /* View transformation matrix. */
    glm::mat4 view = getViewTransformationMatrix(cam_x, cam_o);

    /* Install/Use the program specified by the shader. */
    shader_cad_->install();
    glUniformMatrix4fv(glGetUniformLocation(shader_cad_->get_program(), "view"), 1, GL_FALSE, glm::value_ptr(view));
    shader_cad_->uninstall();

    shader_mesh_texture_->install();
    glUniformMatrix4fv(glGetUniformLocation(shader_mesh_texture_->get_program(), "view"), 1, GL_FALSE, glm::value_ptr(view));
    shader_mesh_texture_->uninstall();

    shader_frame_->install();
    glUniformMatrix4fv(glGetUniformLocation(shader_frame_->get_program(), "view"), 1, GL_FALSE, glm::value_ptr(view));
    shader_frame_->uninstall();

    /* Model transformation matrix. */
    for (const ModelPoseContainerElement& pair : objpos_map)
    {
        const double* pose = pair.second.data();

        glm::mat4 model = glm::rotate(glm::mat4(1.0f), static_cast<float>(pose[6]), glm::vec3(static_cast<float>(pose[3]), static_cast<float>(pose[4]), static_cast<float>(pose[5])));
        model[3][0] = pose[0];
        model[3][1] = pose[1];
        model[3][2] = pose[2];

        auto iter_model = model_obj_.find(pair.first);
        if (iter_model != model_obj_.end())
        {
            if ((iter_model->second)->has_texture())
            {
                shader_mesh_texture_->install();
                glUniformMatrix4fv(glGetUniformLocation(shader_mesh_texture_->get_program(), "model"), 1, GL_FALSE, glm::value_ptr(model));

                (iter_model->second)->Draw(*shader_mesh_texture_);

                shader_mesh_texture_->uninstall();
            }
            else
            {
                shader_cad_->install();
                glUniformMatrix4fv(glGetUniformLocation(shader_cad_->get_program(), "model"), 1, GL_FALSE, glm::value_ptr(model));

                (iter_model->second)->Draw(*shader_cad_);

                shader_cad_->uninstall();
            }
        }
        else if (pair.first == "frame")
        {
            shader_frame_->install();
            glUniformMatrix4fv(glGetUniformLocation(shader_frame_->get_program(), "model"), 1, GL_FALSE, glm::value_ptr(model));
            glBindVertexArray(vao_frame_);
            glDrawArrays(GL_LINES, 0, 6);
            glBindVertexArray(0);
            shader_frame_->uninstall();
        }
    }

    /* Read before swap. glReadPixels read the current framebuffer, i.e. the back one. */
    /* See: http://stackoverflow.com/questions/16809833/opencv-image-loading-for-opengl-texture#16812529
       and http://stackoverflow.com/questions/9097756/converting-data-from-glreadpixels-to-opencvmat#9098883 */
    // cv::Mat ogl_pixel(framebuffer_height_ / tiles_rows_, framebuffer_width_ / tiles_cols_, CV_8UC3);
    // glReadBuffer(GL_COLOR_ATTACHMENT0);
    // glPixelStorei(GL_PACK_ALIGNMENT, (ogl_pixel.step & 3) ? 1 : 4);
    // glPixelStorei(GL_PACK_ROW_LENGTH, ogl_pixel.step / ogl_pixel.elemSize());
    // glReadPixels(0, framebuffer_height_ - tile_img_height_, tile_img_width_, tile_img_height_, GL_BGR, GL_UNSIGNED_BYTE, ogl_pixel.data);

    // cv::flip(ogl_pixel, img, 0);


    //cv::Mat ogl_depth(framebuffer_height_, framebuffer_width_, CV_32FC1);
    //glReadBuffer(GL_DEPTH_ATTACHMENT);
    //glReadPixels(0, 0, framebuffer_width_, framebuffer_height_, GL_DEPTH_COMPONENT, GL_FLOAT, ogl_depth.ptr<float>());

    //cv::flip(ogl_depth, depth, 0);

    ///* Math-reworked version of the fragment code of https://learnopengl.com/Advanced-OpenGL/Depth-testing, section "Visualizing the depth buffer". */
    //const float iv1 = near_ * far_;
    //const float iv2 = near_ - far_;
    //depth.forEach<float>([&iv1, &iv2, this](float& p, const int* pos) -> void { p = iv1 / (far_ + p * iv2); });

	cv::Mat ogl_depth(framebuffer_height_, framebuffer_width_, CV_32FC1);
	glReadBuffer(GL_COLOR_ATTACHMENT1);
	glReadPixels(0, 0, framebuffer_width_, framebuffer_height_, GL_RED, GL_FLOAT, ogl_depth.ptr<float>());

	cv::flip(ogl_depth, depth, 0);


    /* Swap the buffers. */
    glfwSwapBuffers(window_);

    pollOrPostEvent();

    glBindFramebuffer(GL_FRAMEBUFFER, 0);

    glfwMakeContextCurrent(nullptr);

    return true;
}


bool SICAD::superimpose
(
    const std::vector<ModelPoseContainer>& objpos_multimap,
    const double* cam_x,
    const double* cam_o,
    cv::Mat& img
)
{
    /* Model transformation matrix. */
    const int objpos_num = objpos_multimap.size();
    if (objpos_num != tiles_num_) return false;

    glfwMakeContextCurrent(window_);

    glBindFramebuffer(GL_FRAMEBUFFER, fbo_);

    /* View transformation matrix. */
    glm::mat4 view = getViewTransformationMatrix(cam_x, cam_o);

    /* Install/Use the program specified by the shader. */
    shader_cad_->install();
    glUniformMatrix4fv(glGetUniformLocation(shader_cad_->get_program(), "view"), 1, GL_FALSE, glm::value_ptr(view));
    shader_cad_->uninstall();

    shader_mesh_texture_->install();
    glUniformMatrix4fv(glGetUniformLocation(shader_mesh_texture_->get_program(), "view"), 1, GL_FALSE, glm::value_ptr(view));
    shader_mesh_texture_->uninstall();

    shader_frame_->install();
    glUniformMatrix4fv(glGetUniformLocation(shader_frame_->get_program(), "view"), 1, GL_FALSE, glm::value_ptr(view));
    shader_frame_->uninstall();

    for (unsigned int i = 0; i < tiles_rows_; ++i)
    {
        for (unsigned int j = 0; j < tiles_cols_; ++j)
        {
            /* Multimap index */
            int idx = i * tiles_cols_ + j;

            /* Render starting by the upper-left-most tile of the render grid, proceding by columns and rows. */
            glViewport(tile_img_width_ * j, framebuffer_height_ - (tile_img_height_ * (i + 1)),
                       tile_img_width_    , tile_img_height_                                   );
            glScissor (tile_img_width_ * j, framebuffer_height_ - (tile_img_height_ * (i + 1)),
                       tile_img_width_    , tile_img_height_                                   );

            /* Clear the colorbuffer. */
            glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

            /* Draw the background picture. */
            if (getBackgroundOpt())
                renderBackground(img);

            /* View mesh filled or as wireframe. */
            setWireframe(getWireframeOpt());

            /* Install/Use the program specified by the shader. */
            for (const ModelPoseContainerElement& pair : objpos_multimap[idx])
            {
                const double* pose = pair.second.data();

                glm::mat4 model = glm::rotate(glm::mat4(1.0f), static_cast<float>(pose[6]), glm::vec3(static_cast<float>(pose[3]), static_cast<float>(pose[4]), static_cast<float>(pose[5])));
                model[3][0] = static_cast<float>(pose[0]);
                model[3][1] = static_cast<float>(pose[1]);
                model[3][2] = static_cast<float>(pose[2]);

                auto iter_model = model_obj_.find(pair.first);
                if (iter_model != model_obj_.end())
                {
                    if ((iter_model->second)->has_texture())
                    {
                        shader_mesh_texture_->install();
                        glUniformMatrix4fv(glGetUniformLocation(shader_mesh_texture_->get_program(), "model"), 1, GL_FALSE, glm::value_ptr(model));

                        (iter_model->second)->Draw(*shader_mesh_texture_);

                        shader_mesh_texture_->uninstall();
                    }
                    else
                    {
                        shader_cad_->install();
                        glUniformMatrix4fv(glGetUniformLocation(shader_cad_->get_program(), "model"), 1, GL_FALSE, glm::value_ptr(model));

                        (iter_model->second)->Draw(*shader_cad_);

                        shader_cad_->uninstall();
                    }
                }
                else if (pair.first == "frame")
                {
                    shader_frame_->install();
                    glUniformMatrix4fv(glGetUniformLocation(shader_frame_->get_program(), "model"), 1, GL_FALSE, glm::value_ptr(model));
                    glBindVertexArray(vao_frame_);
                    glDrawArrays(GL_LINES, 0, 6);
                    glBindVertexArray(0);
                    shader_frame_->uninstall();
                }
            }
        }
    }

    /* Read before swap. glReadPixels read the current framebuffer, i.e. the back one. */
    /* See: http://stackoverflow.com/questions/16809833/opencv-image-loading-for-opengl-texture#16812529
       and http://stackoverflow.com/questions/9097756/converting-data-from-glreadpixels-to-opencvmat#9098883 */
    cv::Mat ogl_pixel(framebuffer_height_, framebuffer_width_, CV_8UC3);
    glReadBuffer(GL_COLOR_ATTACHMENT0);
    glPixelStorei(GL_PACK_ALIGNMENT, (ogl_pixel.step & 3) ? 1 : 4);
    glPixelStorei(GL_PACK_ROW_LENGTH, ogl_pixel.step/ogl_pixel.elemSize());
    glReadPixels(0, 0, framebuffer_width_, framebuffer_height_, GL_BGR, GL_UNSIGNED_BYTE, ogl_pixel.data);

    cv::flip(ogl_pixel, img, 0);


    /* Swap the buffers. */
    glfwSwapBuffers(window_);

    pollOrPostEvent();

    glBindFramebuffer(GL_FRAMEBUFFER, 0);

    glfwMakeContextCurrent(nullptr);

    return true;
}


bool SICAD::superimpose
(
    const std::vector<ModelPoseContainer>& objpos_multimap,
    const double* cam_x,
    const double* cam_o,
    cv::Mat& img,
    cv::Mat& depth
)
{
    /* Model transformation matrix. */
    const int objpos_num = objpos_multimap.size();
    if (objpos_num != tiles_num_) return false;

    glfwMakeContextCurrent(window_);

    glBindFramebuffer(GL_FRAMEBUFFER, fbo_);

    /* View transformation matrix. */
    glm::mat4 view = getViewTransformationMatrix(cam_x, cam_o);

    /* Install/Use the program specified by the shader. */
    shader_cad_->install();
    glUniformMatrix4fv(glGetUniformLocation(shader_cad_->get_program(), "view"), 1, GL_FALSE, glm::value_ptr(view));
    shader_cad_->uninstall();

    shader_mesh_texture_->install();
    glUniformMatrix4fv(glGetUniformLocation(shader_mesh_texture_->get_program(), "view"), 1, GL_FALSE, glm::value_ptr(view));
    shader_mesh_texture_->uninstall();

    shader_frame_->install();
    glUniformMatrix4fv(glGetUniformLocation(shader_frame_->get_program(), "view"), 1, GL_FALSE, glm::value_ptr(view));
    shader_frame_->uninstall();

    for (unsigned int i = 0; i < tiles_rows_; ++i)
    {
        for (unsigned int j = 0; j < tiles_cols_; ++j)
        {
            /* Multimap index */
            int idx = i * tiles_cols_ + j;

            /* Render starting by the upper-left-most tile of the render grid, proceding by columns and rows. */
            glViewport(tile_img_width_ * j, framebuffer_height_ - (tile_img_height_ * (i + 1)),
                       tile_img_width_, tile_img_height_);
            glScissor(tile_img_width_ * j, framebuffer_height_ - (tile_img_height_ * (i + 1)),
                      tile_img_width_, tile_img_height_);

            /* Clear the colorbuffer. */
            glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

            /* Draw the background picture. */
            if (getBackgroundOpt())
                renderBackground(img);

            /* View mesh filled or as wireframe. */
            setWireframe(getWireframeOpt());

            /* Install/Use the program specified by the shader. */
            for (const ModelPoseContainerElement& pair : objpos_multimap[idx])
            {
                const double* pose = pair.second.data();

                glm::mat4 model = glm::rotate(glm::mat4(1.0f), static_cast<float>(pose[6]), glm::vec3(static_cast<float>(pose[3]), static_cast<float>(pose[4]), static_cast<float>(pose[5])));
                model[3][0] = static_cast<float>(pose[0]);
                model[3][1] = static_cast<float>(pose[1]);
                model[3][2] = static_cast<float>(pose[2]);

                auto iter_model = model_obj_.find(pair.first);
                if (iter_model != model_obj_.end())
                {
                    if ((iter_model->second)->has_texture())
                    {
                        shader_mesh_texture_->install();
                        glUniformMatrix4fv(glGetUniformLocation(shader_mesh_texture_->get_program(), "model"), 1, GL_FALSE, glm::value_ptr(model));

                        (iter_model->second)->Draw(*shader_mesh_texture_);

                        shader_mesh_texture_->uninstall();
                    }
                    else
                    {
                        shader_cad_->install();
                        glUniformMatrix4fv(glGetUniformLocation(shader_cad_->get_program(), "model"), 1, GL_FALSE, glm::value_ptr(model));

                        (iter_model->second)->Draw(*shader_cad_);

                        shader_cad_->uninstall();
                    }
                }
                else if (pair.first == "frame")
                {
                    shader_frame_->install();
                    glUniformMatrix4fv(glGetUniformLocation(shader_frame_->get_program(), "model"), 1, GL_FALSE, glm::value_ptr(model));
                    glBindVertexArray(vao_frame_);
                    glDrawArrays(GL_LINES, 0, 6);
                    glBindVertexArray(0);
                    shader_frame_->uninstall();
                }
            }
        }
    }

    /* Read before swap. glReadPixels read the current framebuffer, i.e. the back one. */
    /* See: http://stackoverflow.com/questions/16809833/opencv-image-loading-for-opengl-texture#16812529
    and http://stackoverflow.com/questions/9097756/converting-data-from-glreadpixels-to-opencvmat#9098883 */
    // cv::Mat ogl_pixel(framebuffer_height_, framebuffer_width_, CV_8UC3);
    // glReadBuffer(GL_COLOR_ATTACHMENT0);
    // glPixelStorei(GL_PACK_ALIGNMENT, (ogl_pixel.step & 3) ? 1 : 4);
    // glPixelStorei(GL_PACK_ROW_LENGTH, ogl_pixel.step / ogl_pixel.elemSize());
    // glReadPixels(0, 0, framebuffer_width_, framebuffer_height_, GL_BGR, GL_UNSIGNED_BYTE, ogl_pixel.data);

    // cv::flip(ogl_pixel, img, 0);


    //cv::Mat ogl_depth(framebuffer_height_, framebuffer_width_, CV_32FC1);
    //glReadBuffer(GL_DEPTH_ATTACHMENT);
    //glReadPixels(0, 0, framebuffer_width_, framebuffer_height_, GL_DEPTH_COMPONENT, GL_FLOAT, ogl_depth.ptr<float>());

    //cv::flip(ogl_depth, depth, 0);

    //const float iv1 = near_ * far_;
    //const float iv2 = near_ - far_;
    //depth.forEach<float>([&iv1, &iv2, this](float& p, const int* pos) -> void { p = iv1 / (far_ + p * iv2); });

	cv::Mat ogl_depth(framebuffer_height_, framebuffer_width_, CV_32FC1);
	glReadBuffer(GL_COLOR_ATTACHMENT1);
	glReadPixels(0, 0, framebuffer_width_, framebuffer_height_, GL_RED, GL_FLOAT, ogl_depth.ptr<float>());

	cv::flip(ogl_depth, depth, 0);


    /* Swap the buffers. */
    glfwSwapBuffers(window_);

    pollOrPostEvent();

    glBindFramebuffer(GL_FRAMEBUFFER, 0);

    glfwMakeContextCurrent(nullptr);

    return true;
}


bool SICAD::superimpose
(
    const ModelPoseContainer& objpos_map,
    const double* cam_x,
    const double* cam_o,
    cv::Mat& img,
    const GLsizei cam_width,
    const GLsizei cam_height,
    const GLfloat cam_fx,
    const GLfloat cam_fy,
    const GLfloat cam_cx,
    const GLfloat cam_cy
)
{
    if (!setProjectionMatrix(cam_width, cam_height, cam_fx, cam_fy, cam_cx, cam_cy))
        return false;

    return superimpose(objpos_map, cam_x, cam_o, img);
}


bool SICAD::superimpose
(
    const std::vector<ModelPoseContainer>& objpos_multimap,
    const double* cam_x,
    const double* cam_o,
    cv::Mat& img,
    const GLsizei cam_width,
    const GLsizei cam_height,
    const GLfloat cam_fx,
    const GLfloat cam_fy,
    const GLfloat cam_cx,
    const GLfloat cam_cy
)
{
    if (!setProjectionMatrix(cam_width, cam_height, cam_fx, cam_fy, cam_cx, cam_cy))
        return false;

    return superimpose(objpos_multimap, cam_x, cam_o, img);
}


bool SICAD::superimpose
(
    const ModelPoseContainer& objpos_map,
    const double* cam_x,
    const double* cam_o,
    const size_t pbo_index
)
{
    if (!(pbo_index < pbo_number_))
    {
        std::cerr << "ERROR::SICAD::SUPERIMPOSE\nERROR:\n\tSICAD PBO index out of bound." << std::endl;
        return false;
    }


    glfwMakeContextCurrent(window_);

    glBindFramebuffer(GL_FRAMEBUFFER, fbo_);

    /* Render in the upper-left-most tile of the render grid */
    glViewport(0,               framebuffer_height_ - tile_img_height_,
               tile_img_width_, tile_img_height_                       );
    glScissor (0,               framebuffer_height_ - tile_img_height_,
               tile_img_width_, tile_img_height_                       );

    /* Clear the colorbuffer. */
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    /* View mesh filled or as wireframe. */
    setWireframe(getWireframeOpt());

    /* View transformation matrix. */
    glm::mat4 view = getViewTransformationMatrix(cam_x, cam_o);

    /* Install/Use the program specified by the shader. */
    shader_cad_->install();
    glUniformMatrix4fv(glGetUniformLocation(shader_cad_->get_program(), "view"), 1, GL_FALSE, glm::value_ptr(view));
    shader_cad_->uninstall();

    shader_mesh_texture_->install();
    glUniformMatrix4fv(glGetUniformLocation(shader_mesh_texture_->get_program(), "view"), 1, GL_FALSE, glm::value_ptr(view));
    shader_mesh_texture_->uninstall();

    shader_frame_->install();
    glUniformMatrix4fv(glGetUniformLocation(shader_frame_->get_program(), "view"), 1, GL_FALSE, glm::value_ptr(view));
    shader_frame_->uninstall();

    /* Model transformation matrix. */
    for (const ModelPoseContainerElement& pair : objpos_map)
    {
        const double* pose = pair.second.data();

        glm::mat4 model = glm::rotate(glm::mat4(1.0f), static_cast<float>(pose[6]), glm::vec3(static_cast<float>(pose[3]), static_cast<float>(pose[4]), static_cast<float>(pose[5])));
        model[3][0] = pose[0];
        model[3][1] = pose[1];
        model[3][2] = pose[2];

        auto iter_model = model_obj_.find(pair.first);
        if (iter_model != model_obj_.end())
        {
            if ((iter_model->second)->has_texture())
            {
                shader_mesh_texture_->install();
                glUniformMatrix4fv(glGetUniformLocation(shader_mesh_texture_->get_program(), "model"), 1, GL_FALSE, glm::value_ptr(model));

                (iter_model->second)->Draw(*shader_mesh_texture_);

                shader_mesh_texture_->uninstall();
            }
            else
            {
                shader_cad_->install();
                glUniformMatrix4fv(glGetUniformLocation(shader_cad_->get_program(), "model"), 1, GL_FALSE, glm::value_ptr(model));

                (iter_model->second)->Draw(*shader_cad_);

                shader_cad_->uninstall();
            }
        }
        else if (pair.first == "frame")
        {
            shader_frame_->install();
            glUniformMatrix4fv(glGetUniformLocation(shader_frame_->get_program(), "model"), 1, GL_FALSE, glm::value_ptr(model));
            glBindVertexArray(vao_frame_);
            glDrawArrays(GL_LINES, 0, 6);
            glBindVertexArray(0);
            shader_frame_->uninstall();
        }
    }

    glReadBuffer(GL_COLOR_ATTACHMENT0);
    glBindBuffer(GL_PIXEL_PACK_BUFFER, pbo_[pbo_index]);
    glReadPixels(0, framebuffer_height_ - tile_img_height_, tile_img_width_, tile_img_height_, GL_BGR, GL_UNSIGNED_BYTE, 0);

    /* Swap the buffers. */
    glfwSwapBuffers(window_);

    pollOrPostEvent();

    glBindBuffer(GL_PIXEL_PACK_BUFFER, 0);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);

    return true;
}


bool SICAD::superimpose
(
    const ModelPoseContainer& objpos_map,
    const double* cam_x,
    const double* cam_o,
    const size_t pbo_index,
    const cv::Mat& img
)
{
    if (!(pbo_index < pbo_number_))
    {
        std::cerr << "ERROR::SICAD::SUPERIMPOSE\nERROR:\n\tSICAD PBO index out of bound." << std::endl;
        return false;
    }


    glfwMakeContextCurrent(window_);

    glBindFramebuffer(GL_FRAMEBUFFER, fbo_);

    /* Render in the upper-left-most tile of the render grid */
    glViewport(0,               framebuffer_height_ - tile_img_height_,
               tile_img_width_, tile_img_height_                       );
    glScissor (0,               framebuffer_height_ - tile_img_height_,
               tile_img_width_, tile_img_height_                       );

    /* Clear the colorbuffer. */
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    /* Draw the background picture. */
    if (getBackgroundOpt())
        renderBackground(img);

    /* View mesh filled or as wireframe. */
    setWireframe(getWireframeOpt());

    /* View transformation matrix. */
    glm::mat4 view = getViewTransformationMatrix(cam_x, cam_o);

    /* Install/Use the program specified by the shader. */
    shader_cad_->install();
    glUniformMatrix4fv(glGetUniformLocation(shader_cad_->get_program(), "view"), 1, GL_FALSE, glm::value_ptr(view));
    shader_cad_->uninstall();

    shader_mesh_texture_->install();
    glUniformMatrix4fv(glGetUniformLocation(shader_mesh_texture_->get_program(), "view"), 1, GL_FALSE, glm::value_ptr(view));
    shader_mesh_texture_->uninstall();

    shader_frame_->install();
    glUniformMatrix4fv(glGetUniformLocation(shader_frame_->get_program(), "view"), 1, GL_FALSE, glm::value_ptr(view));
    shader_frame_->uninstall();

    /* Model transformation matrix. */
    for (const ModelPoseContainerElement& pair : objpos_map)
    {
        const double* pose = pair.second.data();

        glm::mat4 model = glm::rotate(glm::mat4(1.0f), static_cast<float>(pose[6]), glm::vec3(static_cast<float>(pose[3]), static_cast<float>(pose[4]), static_cast<float>(pose[5])));
        model[3][0] = pose[0];
        model[3][1] = pose[1];
        model[3][2] = pose[2];

        auto iter_model = model_obj_.find(pair.first);
        if (iter_model != model_obj_.end())
        {
            if ((iter_model->second)->has_texture())
            {
                shader_mesh_texture_->install();
                glUniformMatrix4fv(glGetUniformLocation(shader_mesh_texture_->get_program(), "model"), 1, GL_FALSE, glm::value_ptr(model));

                (iter_model->second)->Draw(*shader_mesh_texture_);

                shader_mesh_texture_->uninstall();
            }
            else
            {
                shader_cad_->install();
                glUniformMatrix4fv(glGetUniformLocation(shader_cad_->get_program(), "model"), 1, GL_FALSE, glm::value_ptr(model));

                (iter_model->second)->Draw(*shader_cad_);

                shader_cad_->uninstall();
            }
        }
        else if (pair.first == "frame")
        {
            shader_frame_->install();
            glUniformMatrix4fv(glGetUniformLocation(shader_frame_->get_program(), "model"), 1, GL_FALSE, glm::value_ptr(model));
            glBindVertexArray(vao_frame_);
            glDrawArrays(GL_LINES, 0, 6);
            glBindVertexArray(0);
            shader_frame_->uninstall();
        }
    }

    glReadBuffer(GL_COLOR_ATTACHMENT0);
    glBindBuffer(GL_PIXEL_PACK_BUFFER, pbo_[pbo_index]);
    glReadPixels(0, framebuffer_height_ - tile_img_height_, tile_img_width_, tile_img_height_, GL_BGR, GL_UNSIGNED_BYTE, 0);

    /* Swap the buffers. */
    glfwSwapBuffers(window_);

    pollOrPostEvent();

    glBindBuffer(GL_PIXEL_PACK_BUFFER, 0);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);

    return true;
}


bool SICAD::superimpose
(
    const std::vector<ModelPoseContainer>& objpos_multimap,
    const double* cam_x,
    const double* cam_o,
    const size_t pbo_index
)
{
    if (!(pbo_index < pbo_number_))
    {
        std::cerr << "ERROR::SICAD::SUPERIMPOSE\nERROR:\n\tSICAD PBO index out of bound." << std::endl;
        return false;
    }


    /* Model transformation matrix. */
    const int objpos_num = objpos_multimap.size();
    if (objpos_num != tiles_num_) return false;

    glfwMakeContextCurrent(window_);

    glBindFramebuffer(GL_FRAMEBUFFER, fbo_);

    /* View transformation matrix. */
    glm::mat4 view = getViewTransformationMatrix(cam_x, cam_o);

    /* Install/Use the program specified by the shader. */
    shader_cad_->install();
    glUniformMatrix4fv(glGetUniformLocation(shader_cad_->get_program(), "view"), 1, GL_FALSE, glm::value_ptr(view));
    shader_cad_->uninstall();

    shader_mesh_texture_->install();
    glUniformMatrix4fv(glGetUniformLocation(shader_mesh_texture_->get_program(), "view"), 1, GL_FALSE, glm::value_ptr(view));
    shader_mesh_texture_->uninstall();

    shader_frame_->install();
    glUniformMatrix4fv(glGetUniformLocation(shader_frame_->get_program(), "view"), 1, GL_FALSE, glm::value_ptr(view));
    shader_frame_->uninstall();

    for (unsigned int i = 0; i < tiles_rows_; ++i)
    {
        for (unsigned int j = 0; j < tiles_cols_; ++j)
        {
            /* Multimap index */
            int idx = i * tiles_cols_ + j;

            /* Render starting by the upper-left-most tile of the render grid, proceding by columns and rows. */
            glViewport(tile_img_width_ * j, framebuffer_height_ - (tile_img_height_ * (i + 1)),
                       tile_img_width_,     tile_img_height_                                   );
            glScissor (tile_img_width_ * j, framebuffer_height_ - (tile_img_height_ * (i + 1)),
                       tile_img_width_,     tile_img_height_                                   );

            /* Clear the colorbuffer. */
            glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

            /* View mesh filled or as wireframe. */
            setWireframe(getWireframeOpt());

            /* Install/Use the program specified by the shader. */
            for (const ModelPoseContainerElement& pair : objpos_multimap[idx])
            {
                const double* pose = pair.second.data();

                glm::mat4 model = glm::rotate(glm::mat4(1.0f), static_cast<float>(pose[6]), glm::vec3(static_cast<float>(pose[3]), static_cast<float>(pose[4]), static_cast<float>(pose[5])));
                model[3][0] = static_cast<float>(pose[0]);
                model[3][1] = static_cast<float>(pose[1]);
                model[3][2] = static_cast<float>(pose[2]);

                auto iter_model = model_obj_.find(pair.first);
                if (iter_model != model_obj_.end())
                {
                    if ((iter_model->second)->has_texture())
                    {
                        shader_mesh_texture_->install();
                        glUniformMatrix4fv(glGetUniformLocation(shader_mesh_texture_->get_program(), "model"), 1, GL_FALSE, glm::value_ptr(model));

                        (iter_model->second)->Draw(*shader_mesh_texture_);

                        shader_mesh_texture_->uninstall();
                    }
                    else
                    {
                        shader_cad_->install();
                        glUniformMatrix4fv(glGetUniformLocation(shader_cad_->get_program(), "model"), 1, GL_FALSE, glm::value_ptr(model));

                        (iter_model->second)->Draw(*shader_cad_);

                        shader_cad_->uninstall();
                    }
                }
                else if (pair.first == "frame")
                {
                    shader_frame_->install();
                    glUniformMatrix4fv(glGetUniformLocation(shader_frame_->get_program(), "model"), 1, GL_FALSE, glm::value_ptr(model));
                    glBindVertexArray(vao_frame_);
                    glDrawArrays(GL_LINES, 0, 6);
                    glBindVertexArray(0);
                    shader_frame_->uninstall();
                }
            }
        }
    }

    glReadBuffer(GL_COLOR_ATTACHMENT0);
    glBindBuffer(GL_PIXEL_PACK_BUFFER, pbo_[pbo_index]);
    glReadPixels(0, 0, framebuffer_width_, framebuffer_height_, GL_BGR, GL_UNSIGNED_BYTE, 0);

    /* Swap the buffers. */
    glfwSwapBuffers(window_);

    pollOrPostEvent();

    glBindBuffer(GL_PIXEL_PACK_BUFFER, 0);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);

    return true;
}


bool SICAD::superimpose
(
    const std::vector<ModelPoseContainer>& objpos_multimap,
    const double* cam_x,
    const double* cam_o,
    const size_t pbo_index,
    const cv::Mat& img
)
{
    if (!(pbo_index < pbo_number_))
    {
        std::cerr << "ERROR::SICAD::SUPERIMPOSE\nERROR:\n\tSICAD PBO index out of bound." << std::endl;
        return false;
    }


    /* Model transformation matrix. */
    const int objpos_num = objpos_multimap.size();
    if (objpos_num != tiles_num_) return false;

    glfwMakeContextCurrent(window_);

    glBindFramebuffer(GL_FRAMEBUFFER, fbo_);

    /* View transformation matrix. */
    glm::mat4 view = getViewTransformationMatrix(cam_x, cam_o);

    /* Install/Use the program specified by the shader. */
    shader_cad_->install();
    glUniformMatrix4fv(glGetUniformLocation(shader_cad_->get_program(), "view"), 1, GL_FALSE, glm::value_ptr(view));
    shader_cad_->uninstall();

    shader_mesh_texture_->install();
    glUniformMatrix4fv(glGetUniformLocation(shader_mesh_texture_->get_program(), "view"), 1, GL_FALSE, glm::value_ptr(view));
    shader_mesh_texture_->uninstall();

    shader_frame_->install();
    glUniformMatrix4fv(glGetUniformLocation(shader_frame_->get_program(), "view"), 1, GL_FALSE, glm::value_ptr(view));
    shader_frame_->uninstall();

    for (unsigned int i = 0; i < tiles_rows_; ++i)
    {
        for (unsigned int j = 0; j < tiles_cols_; ++j)
        {
            /* Multimap index */
            int idx = i * tiles_cols_ + j;

            /* Render starting by the upper-left-most tile of the render grid, proceding by columns and rows. */
            glViewport(tile_img_width_ * j, framebuffer_height_ - (tile_img_height_ * (i + 1)),
                       tile_img_width_,     tile_img_height_                                   );
            glScissor (tile_img_width_ * j, framebuffer_height_ - (tile_img_height_ * (i + 1)),
                       tile_img_width_,     tile_img_height_                                   );

            /* Clear the colorbuffer. */
            glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

            /* Draw the background picture. */
            if (getBackgroundOpt())
                renderBackground(img);

            /* View mesh filled or as wireframe. */
            setWireframe(getWireframeOpt());

            /* Install/Use the program specified by the shader. */
            for (const ModelPoseContainerElement& pair : objpos_multimap[idx])
            {
                const double* pose = pair.second.data();

                glm::mat4 model = glm::rotate(glm::mat4(1.0f), static_cast<float>(pose[6]), glm::vec3(static_cast<float>(pose[3]), static_cast<float>(pose[4]), static_cast<float>(pose[5])));
                model[3][0] = static_cast<float>(pose[0]);
                model[3][1] = static_cast<float>(pose[1]);
                model[3][2] = static_cast<float>(pose[2]);

                auto iter_model = model_obj_.find(pair.first);
                if (iter_model != model_obj_.end())
                {
                    if ((iter_model->second)->has_texture())
                    {
                        shader_mesh_texture_->install();
                        glUniformMatrix4fv(glGetUniformLocation(shader_mesh_texture_->get_program(), "model"), 1, GL_FALSE, glm::value_ptr(model));

                        (iter_model->second)->Draw(*shader_mesh_texture_);

                        shader_mesh_texture_->uninstall();
                    }
                    else
                    {
                        shader_cad_->install();
                        glUniformMatrix4fv(glGetUniformLocation(shader_cad_->get_program(), "model"), 1, GL_FALSE, glm::value_ptr(model));

                        (iter_model->second)->Draw(*shader_cad_);

                        shader_cad_->uninstall();
                    }
                }
                else if (pair.first == "frame")
                {
                    shader_frame_->install();
                    glUniformMatrix4fv(glGetUniformLocation(shader_frame_->get_program(), "model"), 1, GL_FALSE, glm::value_ptr(model));
                    glBindVertexArray(vao_frame_);
                    glDrawArrays(GL_LINES, 0, 6);
                    glBindVertexArray(0);
                    shader_frame_->uninstall();
                }
            }
        }
    }

    glReadBuffer(GL_COLOR_ATTACHMENT0);
    glBindBuffer(GL_PIXEL_PACK_BUFFER, pbo_[pbo_index]);
    glReadPixels(0, 0, framebuffer_width_, framebuffer_height_, GL_BGR, GL_UNSIGNED_BYTE, 0);

    /* Swap the buffers. */
    glfwSwapBuffers(window_);

    pollOrPostEvent();

    glBindBuffer(GL_PIXEL_PACK_BUFFER, 0);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);

    return true;
}


void SICAD::releaseContext() const
{
    glfwMakeContextCurrent(nullptr);
}


std::pair<const GLuint*, size_t> SICAD::getPBOs() const
{
    glfwMakeContextCurrent(window_);

    return std::make_pair(pbo_, pbo_number_);
}


std::pair<bool, GLuint> SICAD::getPBO(const size_t pbo_index) const
{
    if (pbo_index < pbo_number_)
    {
        glfwMakeContextCurrent(window_);

        return std::make_pair(true, pbo_[pbo_index]);
    }

    return std::make_pair(false, 0);
}


bool SICAD::setProjectionMatrix
(
    const GLsizei cam_width,
    const GLsizei cam_height,
    const GLfloat cam_fx,
    const GLfloat cam_fy,
    const GLfloat cam_cx,
    const GLfloat cam_cy
)
{
    glfwMakeContextCurrent(window_);

    /* Projection matrix. */
    /* In both OpenGL window coordinates and Hartley-Zisserman (HZ) image coordinate systems, (0,0) is the lower left corner with X and Y increasing right and up, respectively. In a normal image file, the (0,0) pixel is in the upper left corner.
       There are two possibilities to convert this coordinate system into the image file system coordinate (also used by OpenCV).
       1. We can render/draw our images upside down, so that we have already a correspondence between OpenGL and image coordinate systems.
          The projection matrix is the following:
          [2*K00/width,  -2*K01/width,   (width - 2*K02 + 2*x0)/width,                            0]
          [          0, -2*K11/height, (height - 2*K12 + 2*y0)/height,                            0]
          [          0,             0, (-zfar - znear)/(zfar - znear), -2*zfar*znear/(zfar - znear)]
          [          0,             0,                             -1,                            0]
       2. We can render/draw our image normally and compensate the the flipped image externally (e.g. with OpenCV flip()).
          The projection matrix is the following:
          [2*K00/width, -2*K01/width,    (width - 2*K02 + 2*x0)/width,                            0]
          [          0, 2*K11/height, (-height + 2*K12 + 2*y0)/height,                            0]
          [          0,            0,  (-zfar - znear)/(zfar - znear), -2*zfar*znear/(zfar - znear)]
          [          0,            0,                              -1,                            0]
       Where "Knm" is the (n,m) entry of the 3x3 HZ instrinsic camera calibration matrix K. K is upper triangular and scaled such that the lower-right entry is one. "width" and "height" are the size of the camera image, in pixels, and "x0" and "y0" are the camera image origin, which are normally zero. "znear" and "zfar" are the standard OpenGL near and far clipping planes, respectively. */
    // projection_ = glm::mat4(2.0f*(cam_fx/cam_width),    0,                              0,                                  0,
    //                         0,                          -2.0f*(cam_fy/cam_height),      0,                                  0,
    //                         1-2.0f*(cam_cx/cam_width),  1-2.0f*(cam_cy/cam_height),    -(far_+near_)/(far_-near_),         -1,
    //                         0,                          0,                             -2.0f*(far_*near_)/(far_-near_),     0);

    projection_ = glm::mat4(2.0f*(cam_fx/cam_width),    0,                           0,                               0,
                            0,                          2.0f*(cam_fy/cam_height),    0,                               0,
                            1-2.0f*(cam_cx/cam_width),  2.0f*(cam_cy/cam_height)-1, -(far_+near_)/(far_-near_),      -1,
                            0,                          0,                          -2.0f*(far_*near_)/(far_-near_),  0 );

    /* Install/Use the program specified by the shader. */
    shader_cad_->install();
    glUniformMatrix4fv(glGetUniformLocation(shader_cad_->get_program(), "projection"), 1, GL_FALSE, glm::value_ptr(projection_));
    shader_cad_->uninstall();

    shader_mesh_texture_->install();
    glUniformMatrix4fv(glGetUniformLocation(shader_mesh_texture_->get_program(), "projection"), 1, GL_FALSE, glm::value_ptr(projection_));
    shader_mesh_texture_->uninstall();

    shader_frame_->install();
    glUniformMatrix4fv(glGetUniformLocation(shader_frame_->get_program(), "projection"), 1, GL_FALSE, glm::value_ptr(projection_));
    shader_frame_->uninstall();

    glfwSwapBuffers(window_);
    glfwMakeContextCurrent(nullptr);

    return true;
}


void SICAD::setOglToCam(const std::vector<float>& ogl_to_cam)
{
    if (ogl_to_cam.size() != 4)
        throw std::runtime_error("ERROR::SICAD::setOglToCam\nERROR:\n\tWrong input size.\n\tShould be 4, was given " + std::to_string(ogl_to_cam.size()) + ".");

    ogl_to_cam_ = glm::mat3(glm::rotate(glm::mat4(1.0f), ogl_to_cam[3], glm::make_vec3(ogl_to_cam.data())));
}


void SICAD::setBackgroundOpt(bool show_background)
{
    show_background_ = show_background;
}


bool SICAD::getBackgroundOpt() const
{
    return show_background_;
}


void SICAD::setWireframeOpt(bool show_mesh_wires)
{
    if  (show_mesh_wires) show_mesh_mode_ = GL_LINE;
    else                  show_mesh_mode_ = GL_FILL;
}


void SICAD::setMipmapsOpt(const MIPMaps& mipmaps)
{
    mesh_mmaps_ = mipmaps;
}


GLenum SICAD::getWireframeOpt() const
{
    return show_mesh_mode_;
}


SICAD::MIPMaps SICAD::getMipmapsOpt() const
{
    return mesh_mmaps_;
}


int SICAD::getTilesNumber() const
{
    return tiles_rows_ * tiles_cols_;
}


int SICAD::getTilesRows() const
{
    return tiles_rows_;
}


int SICAD::getTilesCols() const
{
    return tiles_cols_;
}


glm::mat4 SICAD::getViewTransformationMatrix(const double* cam_x, const double* cam_o)
{
    glm::mat4 root_cam_t  = glm::translate(glm::mat4(1.0f),
                                           glm::vec3(static_cast<float>(cam_x[0]), static_cast<float>(cam_x[1]), static_cast<float>(cam_x[2])));
    glm::mat4 cam_to_root = glm::rotate(glm::mat4(1.0f),
                                        static_cast<float>(cam_o[3]), glm::vec3(static_cast<float>(cam_o[0]), static_cast<float>(cam_o[1]), static_cast<float>(cam_o[2])));

    glm::mat4 view = glm::lookAt(glm::vec3(root_cam_t[3].x, root_cam_t[3].y, root_cam_t[3].z),
                                 glm::vec3(root_cam_t[3].x, root_cam_t[3].y, root_cam_t[3].z) + glm::mat3(cam_to_root) * ogl_to_cam_ * glm::vec3(0.0f, 0.0f, -1.0f),
                                 glm::mat3(cam_to_root) * ogl_to_cam_ * glm::vec3(0.0f, 1.0f, 0.0f));

    return view;
}


void SICAD::pollOrPostEvent()
{
    if(main_thread_id_ == std::this_thread::get_id())
        glfwPollEvents();
    else
        glfwPostEmptyEvent();
}


void SICAD::renderBackground(const cv::Mat& img) const
{
    /* Load and generate the texture. */
    glBindTexture(GL_TEXTURE_2D, texture_background_);

    /* Set the texture wrapping/filtering options (on the currently bound texture object). */
    if (getMipmapsOpt() == MIPMaps::nearest)
    {
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST_MIPMAP_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    }
    else if (getMipmapsOpt() == MIPMaps::linear)
    {
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    }

    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, img.cols, img.rows, 0, GL_BGR, GL_UNSIGNED_BYTE, img.data);
    glGenerateMipmap(GL_TEXTURE_2D);

    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

    /* Install/Use the program specified by the shader. */
    shader_background_->install();
    glUniformMatrix4fv(glGetUniformLocation(shader_background_->get_program(), "projection"), 1, GL_FALSE, glm::value_ptr(back_proj_));

    glBindVertexArray(vao_background_);
    glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0);
    glBindVertexArray(0);

    glBindTexture(GL_TEXTURE_2D, 0);
    shader_background_->uninstall();
}


void SICAD::setWireframe(GLenum mode)
{
    glPolygonMode(GL_FRONT_AND_BACK, mode);
}


void SICAD::factorize_int
(
    const GLsizei area,
    const GLsizei width_limit,
    const GLsizei height_limit,
    GLsizei& width,
    GLsizei& height
)
{
    double sqrt_area = std::floor(std::sqrt(static_cast<double>(area)));
    height = std::min(static_cast<int>(sqrt_area), height_limit);
    width  = std::min(area / height,               width_limit);
}
