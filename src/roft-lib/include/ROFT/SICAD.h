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

#ifndef ROFT_SICAD_H
#define ROFT_SICAD_H

#include <SuperimposeMesh/Superimpose.h>

#include <ROFT/SICADShader.h>
#include <ROFT/SICADModel.h>

#include <istream>
#include <memory>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>

namespace ROFT {
    class SICAD;
}


/**
 * A Superimpose derived class to superimpose mesh models on images.
 */
class ROFT::SICAD : public Superimpose
{
public:
    typedef typename std::unordered_map<std::string, std::string> ModelPathContainer;

    typedef typename std::unordered_map<std::string, std::basic_istream<char>*> ModelStreamContainer;

    typedef typename std::pair<std::string, std::string> ModelPathElement;

    typedef typename std::pair<std::string, std::basic_istream<char>*> ModelStreamElement;

    typedef typename std::unordered_map<std::string, SICADModel*> ModelContainer;

    typedef typename std::pair<std::string, SICADModel*> ModelElement;

    enum class MIPMaps
    {
        nearest,
        linear
    };

    /**
     * Create a SICAD object with a dedicated OpenGL context and default shaders.
     *
     * Only 1 image will be rendered in the OpenGL context.
     *
     * The reference frame of the OpenGL virtual camera is the standard right-handed system and can be
     * changed by means of `setOglToCam()' method.
     *
     * @param objfile_map A (tag, path) container to associate a 'tag' to the mesh file specified in 'path'.
     * @param cam_width Camera or image width.
     * @param cam_height Camera or image height.
     * @param cam_fx focal Length along the x axis in pixels.
     * @param cam_fy focal Length along the y axis in pixels.
     */
    SICAD(const ModelPathContainer& objfile_map, const GLsizei cam_width, const GLsizei cam_height, const GLfloat cam_fx, const GLfloat cam_fy, const GLfloat cam_cx, const GLfloat cam_cy);

    /**
     * Create a SICAD object with a dedicated OpenGL context and default shaders.
     *
     * Only 1 image will be rendered in the OpenGL context.
     *
     * The reference frame of the OpenGL virtual camera is the standard right-handed system and can be
     * changed by means of `ogl_to_cam` parameter.
     *
     * @param objstream_map A (tag, stream) container to associate a 'tag' to the mesh file contained in the stream 'stream'.
     * @param cam_width Camera or image width.
     * @param cam_height Camera or image height.
     * @param cam_fx focal Length along the x axis in pixels.
     * @param cam_fy focal Length along the y axis in pixels.
     */
    SICAD(const ModelStreamContainer& objstream_map, const GLsizei cam_width, const GLsizei cam_height, const GLfloat cam_fx, const GLfloat cam_fy, const GLfloat cam_cx, const GLfloat cam_cy);

    /**
     * Create a SICAD object with a dedicated OpenGL context and default shaders.
     *
     * Up to `num_images` images will be rendered in the same OpenGL context and the result of
     * the process will be tiled up in a regular grid. This implies that the total number
     * of rendered images may be less than or equal to the required `num_images`. The total
     * number of rendered images is chosen to optimize performance and accessibility and can be
     * accessed through `SICAD::getTilesNumber()`.
     *
     * The reference frame of the OpenGL virtual camera is the standard right-handed system and can be
     * changed by means of `setOglToCam()' method.
     *
     * @param objfile_map A (tag, path) container to associate a 'tag' to the mesh file specified in 'path'.
     * @param cam_width Camera or image width.
     * @param cam_height Camera or image height.
     * @param cam_fx focal Length along the x axis in pixels.
     * @param cam_fy focal Length along the y axis in pixels.
     * @param num_images Number of images (i.e. viewports) rendered in the same GL context.
     */
    SICAD(const ModelPathContainer& objfile_map, const GLsizei cam_width, const GLsizei cam_height, const GLfloat cam_fx, const GLfloat cam_fy, const GLfloat cam_cx, const GLfloat cam_cy, const GLint num_images);

    /**
     * Create a SICAD object with a dedicated OpenGL context and default shaders.
     *
     * Up to `num_images` images will be rendered in the same OpenGL context and the result of
     * the process will be tiled up in a regular grid. This implies that the total number
     * of rendered images may be less than or equal to the required `num_images`. The total
     * number of rendered images is chosen to optimize performance and accessibility and can be
     * accessed through `SICAD::getTilesNumber()`.
     *
     * The reference frame of the OpenGL virtual camera is the standard right-handed system.
     *
     * @param objstream_map A (tag, stream) container to associate a 'tag' to the mesh file contained in the stream 'stream'.
     * @param cam_width Camera or image width.
     * @param cam_height Camera or image height.
     * @param cam_fx focal Length along the x axis in pixels.
     * @param cam_fy focal Length along the y axis in pixels.
     * @param num_images Number of images (i.e. viewports) rendered in the same GL context.
     */
    SICAD(const ModelStreamContainer& objstream_map, const GLsizei cam_width, const GLsizei cam_height, const GLfloat cam_fx, const GLfloat cam_fy, const GLfloat cam_cx, const GLfloat cam_cy, const GLint num_images);

    /**
     * Create a SICAD object with a dedicated OpenGL context and custom shaders.
     * The folder where the shaders are stored can be specified in `shader_folder`.
     *
     * The following shaders with this exact names are needed:
     *
     *  - `shader_model.vert` for the vertex shader for mesh models
     *  - `shader_model.frag` for the fragment shader for mesh models
     *  - `shader_model_texture.frag` for the fragment shader for textured mesh models
     *
     *  - `shader_frame.vert` for the vertex shader for reference frames
     *  - `shader_frame.frag` for the fragment shader for reference frames
     *
     *  - `shader_background.vert` for the vertex shader for background
     *  - `shader_background.frag` for the fragment shader for background
     *
     * Up to `num_images` images will be rendered in the same OpenGL context and the result of
     * the process will be tiled up in a regular grid. This implies that the total number
     * of rendered images may be less than or equal to the required `num_images`. The total
     * number of rendered images is chosen to optimize performance and accessibility and can be
     * accessed through `SICAD::getTilesNumber()`.
     *
     * The reference frame of the OpenGL virtual camera is the standard right-handed system.
     *
     * @param objfile_map A (tag, path) container to associate a 'tag' to the mesh file specified in 'path'.
     * @param cam_width Camera or image width.
     * @param cam_height Camera or image height.
     * @param cam_fx focal Length along the x axis in pixels.
     * @param cam_fy focal Length along the y axis in pixels.
     * @param num_images Number of images (i.e. viewports) rendered in the same GL context.
     * @param shader_folder Path to the folder containing the above-mentioned required shaders.
     */
    SICAD(const ModelPathContainer& objfile_map, const GLsizei cam_width, const GLsizei cam_height, const GLfloat cam_fx, const GLfloat cam_fy, const GLfloat cam_cx, const GLfloat cam_cy, const GLint num_images, const std::string& shader_folder);

    /**
     * Create a SICAD object with a dedicated OpenGL context and custom shaders.
     * The folder where the shaders are stored can be specified in `shader_folder`.
     *
     * The following shaders with this exact names are needed:
     *
     *  - `shader_model.vert` for the vertex shader for mesh models
     *  - `shader_model.frag` for the fragment shader for mesh models
     *  - `shader_model_texture.frag` for the fragment shader for textured mesh models
     *
     *  - `shader_frame.vert` for the vertex shader for reference frames
     *  - `shader_frame.frag` for the fragment shader for reference frames
     *
     *  - `shader_background.vert` for the vertex shader for background
     *  - `shader_background.frag` for the fragment shader for background
     *
     * Up to `num_images` images will be rendered in the same OpenGL context and the result of
     * the process will be tiled up in a regular grid. This implies that the total number
     * of rendered images may be less than or equal to the required `num_images`. The total
     * number of rendered images is chosen to optimize performance and accessibility and can be
     * accessed through `SICAD::getTilesNumber()`.
     *
     * The reference frame of the OpenGL virtual camera is the standard right-handed system and can be
     * changed by means of `ogl_to_cam` or using the `setOglToCam()` method.
     *
     * @param objfile_map A (tag, path) container to associate a 'tag' to the mesh file specified in 'path'.
     * @param cam_width Camera or image width.
     * @param cam_height Camera or image height.
     * @param cam_fx focal Length along the x axis in pixels.
     * @param cam_fy focal Length along the y axis in pixels.
     * @param num_images Number of images (i.e. viewports) rendered in the same GL context.
     * @param shader_folder Path to the folder containing the above-mentioned required shaders.
     * @param ogl_to_cam A 7-component pose vector, (x, y, z) position and a (ux, uy, uz, theta) axis-angle orientation, defining a camera rotation applied to the OpenGL camera.
     */
    SICAD(const GLsizei cam_width, const GLsizei cam_height, const GLfloat cam_fx, const GLfloat cam_fy, const GLfloat cam_cx, const GLfloat cam_cy, const GLint num_images, const std::string& shader_folder, const std::vector<float>& ogl_to_cam, const ModelPathContainer& objfile_map = ModelPathContainer(), const ModelStreamContainer& objstream_map = ModelStreamContainer());

    virtual ~SICAD();

    bool getOglWindowShouldClose();

    void setOglWindowShouldClose(bool should_close);

    /**
     * Render the mesh models in the pose specified in `objpos_map` and move the virtual camera in `cam_x` position with orientation `cam_o`.
     * The method then creates an image of the mesh models as they are seen by the virtual camera.
     *
     * @note If cv::Mat `img` is a background image it must be of size `cam_width * cam_height`, as specified during object construction,
     * and the `SICAD::setBackgroundOpt(bool show_background)` must have been invoked with `true`.
     *
     * @param objpos_map A (tag, pose) container to associate a 7-component `pose`, (x, y, z) position and a (ux, uy, uz, theta) axis-angle orientation, to a mesh with tag 'tag'.
     * @param cam_x (x, y, z) position.
     * @param cam_o (ux, uy, uz, theta) axis-angle orientation.
     * @param img An image representing the result of the superimposition. The variable is automatically resized if its size is not correct to store the entire result of the superimposition.
     *
     * @return true upon success, false otherswise.
     **/
    bool superimpose(const ModelPoseContainer& objpos_map, const double* cam_x, const double* cam_o, cv::Mat& img) override;

    bool superimpose(const ModelPoseContainer& objpos_map, const double* cam_x, const double* cam_o, cv::Mat& img, cv::Mat& depth);

    /**
     * Render the mesh models in the pose specified in each element of `objpos_multimap` and move the virtual camera in
     * `cam_x` position with orientation `cam_o`. Each group of meshes specified by the elements of `objpos_multimap` are rendered in a
     * different viewport. Each viewport reports the mesh models as they are seen by the virtual camera.
     * The method then creates a single image tiling the viewports in a regular grid.
     *
     * @note The size of the grid representing the tiled viewports can be accessed through `getTilesRows()` and `getTilesCols()`.
     *
     * @note If cv::Mat `img` is a background image it must be of size `cam_width * cam_height`, as specified during object construction,
     * and the `SICAD::setBackgroundOpt(bool show_background)` must have been invoked with `true`.
     *
     * @param objpos_map A (tag, pose) container to associate a 7-component `pose`, (x, y, z) position and a (ux, uy, uz, theta) axis-angle orientation, to a mesh with tag 'tag'.
     * @param cam_x (x, y, z) position.
     * @param cam_o (ux, uy, uz, theta) axis-angle orientation.
     * @param img An image representing the result of the superimposition. The variable is automatically resized if its size is not correct to store the entire result of the superimposition.
     *
     * @return true upon success, false otherswise.
     **/
    virtual bool superimpose(const std::vector<ModelPoseContainer>& objpos_multimap, const double* cam_x, const double* cam_o, cv::Mat& img);

    virtual bool superimpose(const std::vector<ModelPoseContainer>& objpos_multimap, const double* cam_x, const double* cam_o, cv::Mat& img, cv::Mat& dept);

    virtual bool superimpose(const ModelPoseContainer& objpos_map, const double* cam_x, const double* cam_o, cv::Mat& img,
                             const GLsizei cam_width, const GLsizei cam_height, const GLfloat cam_fx, const GLfloat cam_fy, const GLfloat cam_cx, const GLfloat cam_cy);

    virtual bool superimpose(const std::vector<ModelPoseContainer>& objpos_multimap, const double* cam_x, const double* cam_o, cv::Mat& img,
                             const GLsizei cam_width, const GLsizei cam_height, const GLfloat cam_fx, const GLfloat cam_fy, const GLfloat cam_cx, const GLfloat cam_cy);

    /**
     * Render the mesh models in the pose specified in `objpos_map` and move the virtual camera in `cam_x` position with orientation `cam_o`.
     * The method then stores the pixels of the mesh models as they are seen by the virtual camera in the `pbo_index`-th Pixel Buffer Object (PBO).
     *
     * @note By invoking this command rendered pixels are stored in the `pbo_index`-th PBO and, in order to use it, the OpenGL context must remain current.
     * As a consequence, once you are done working with the `pbo_index`-th PBO (can be accessed by means of `SICAD::getPBO(pbo_index)`) and before invoking again
     * any other `SICAD::superimpose()` function, you must invoke `SICAD::releaseContext()`.
     *
     * @param objpos_map A (tag, pose) container to associate a 7-component `pose`, (x, y, z) position and a (ux, uy, uz, theta) axis-angle orientation, to a mesh with tag 'tag'.
     * @param cam_x (x, y, z) position.
     * @param cam_o (ux, uy, uz, theta) axis-angle orientation.
     * @param pbo_index The index of the PBO where the pixel are stored.
     *
     * @return (true, PBO) upon success, (false, 0) otherswise.
     **/
    virtual bool superimpose(const ModelPoseContainer& objpos_map, const double* cam_x, const double* cam_o, const size_t pbo_index);

    /**
     * Render the mesh models in the pose specified in `objpos_map` and move the virtual camera in `cam_x` position with orientation `cam_o`.
     * The method then stores the pixels of the mesh models as they are seen by the virtual camera in the `pbo_index`-th Pixel Buffer Object (PBO).
     *
     * @note `img` must be of size `cam_width * cam_height`, as specified during object construction, and the
     * `SICAD::setBackgroundOpt(bool show_background)` must have been invoked with `true`.
     *
     * @param objpos_map A (tag, pose) container to associate a 7-component `pose`, (x, y, z) position and a (ux, uy, uz, theta) axis-angle orientation, to a mesh with tag 'tag'.
     * @param cam_x (x, y, z) position.
     * @param cam_o (ux, uy, uz, theta) axis-angle orientation.
     * @param pbo_index The index of the PBO where the pixel are stored.
     * @param img A background image.
     *
     * @return (true, PBO) upon success, (false, 0) otherswise.
     **/
    virtual bool superimpose(const ModelPoseContainer& objpos_map, const double* cam_x, const double* cam_o, const size_t pbo_index, const cv::Mat& img);

    /**
     * Render the mesh models in the pose specified in each element of `objpos_multimap` and move the virtual camera in
     * `cam_x` position with orientation `cam_o`. Each group of meshes specified by the elements of `objpos_multimap` are rendered in a
     * different viewport. Each viewport reports the mesh models as they are seen by the virtual camera.
     * The method then stores the pixels of the viewports in the `pbo_index`-th Pixel Buffer Object (PBO) by tiling them in a regular grid.
     *
     * @note By invoking this command rendered pixels are stored in the `pbo_index`-th PBO and, in order to use it, the OpenGL context must remain current.
     * As a consequence, once you are done working with the `pbo_index`-th PBO (can be accessed by means of `SICAD::getPBO(pbo_index)`) and before invoking again
     * any other `SICAD::superimpose()` function, you must invoke `SICAD::releaseContext()`.
     *
     * @note The size of the grid representing the tiled viewports can be accessed through `getTilesRows()` and `getTilesCols()`.
     *
     * @param objpos_map A (tag, pose) container to associate a 7-component `pose`, (x, y, z) position and a (ux, uy, uz, theta) axis-angle orientation, to a mesh with tag 'tag'.
     * @param cam_x (x, y, z) position.
     * @param cam_o (ux, uy, uz, theta) axis-angle orientation.
     * @param pbo_index The index of the PBO where the pixel are stored.
     *
     * @return (true, PBO) upon success, (false, 0) otherswise.
     **/
    virtual bool superimpose(const std::vector<ModelPoseContainer>& objpos_multimap, const double* cam_x, const double* cam_o, const size_t pbo_index);

    /**
     * Render the mesh models in the pose specified in each element of `objpos_multimap` and move the virtual camera in
     * `cam_x` position with orientation `cam_o`. Each group of meshes specified by the elements of `objpos_multimap` are rendered in a
     * different viewport. Each viewport reports the mesh models as they are seen by the virtual camera.
     * The method then stores the pixels of the viewports in the `pbo_index`-th Pixel Buffer Object (PBO) by tiling them in a regular grid.
     *
     * @note The size of the grid representing the tiled viewports can be accessed through `getTilesRows()` and `getTilesCols()`.
     *
     * @note `img` must be of size `cam_width * cam_height`, as specified during object construction, and the
     * `SICAD::setBackgroundOpt(bool show_background)` must have been invoked with `true`.
     *
     * @param objpos_map A (tag, pose) container to associate a 7-component `pose`, (x, y, z) position and a (ux, uy, uz, theta) axis-angle orientation, to a mesh with tag 'tag'.
     * @param cam_x (x, y, z) position.
     * @param cam_o (ux, uy, uz, theta) axis-angle orientation.
     * @param pbo_index The index of the PBO where the pixel are stored.
     * @param img A background image.
     *
     * @return (true, PBO) upon success, (false, 0) otherswise.
     **/
    virtual bool superimpose(const std::vector<ModelPoseContainer>& objpos_multimap, const double* cam_x, const double* cam_o, const size_t pbo_index, const cv::Mat& img);

    /**
     * Make the current thread OpenGL context not current.
     *
     * @note This method must be called only when invoking `SICAD::superimpose()` working on Pixel Buffer Objects (PBO),
     * before invoking again any `SICAD::superimpose()` methods (either the ones using PBOs or not), but after
     * having used the PBO that otherwise cannot be accessed as they are bound to the current thread context.
     */
    virtual void releaseContext() const;

    /**
     * Returns the Pixel Buffer Object (PBO) vector and its size.
     *
     * @note Rendered pixels are stored in the `pbo_index`-th PBO and, in order to use it, the OpenGL context must remain current.
     * As a consequence, once you are done working with the `pbo_index`-th PBO and before invoking again
     * any other `SICAD::superimpose()` function, you must invoke `SICAD::releaseContext()`.
     *
     * @return (PBO base array address, number of PBOs)
     */
    std::pair<const GLuint*, size_t> getPBOs() const;

    /**
     * Returns `pbo_index`-th Pixel Buffer Object (PBO) value.
     *
     * @note Rendered pixels are stored in the `pbo_index`-th PBO and, in order to use it, the OpenGL context must remain current.
     * As a consequence, once you are done working with the `pbo_index`-th PBO and before invoking again
     * any other `SICAD::superimpose()` function, you must invoke `SICAD::releaseContext()`.
     *
     * @return (true, PBO) if `pbo_index` exists, (false, 0) otherwise.
     */
    std::pair<bool, GLuint> getPBO(const size_t pbo_index) const;

    /**
     * Sets the static transformation between the camera coordinate system and the OpenGL camera coordinate system.
     *
     * @param ogl_to_cam A 4-sized vector containing the axis-angle representation of the transformation.
     */
    void setOglToCam(const std::vector<float>& ogl_to_cam);

    bool setProjectionMatrix(const GLsizei cam_width, const GLsizei cam_height, const GLfloat cam_fx, const GLfloat cam_fy, const GLfloat cam_cx, const GLfloat cam_cy);

    bool getBackgroundOpt() const;

    void setBackgroundOpt(bool show_background);

    GLenum getWireframeOpt() const;

    void setWireframeOpt(bool show_mesh_wires);

    void setMipmapsOpt(const MIPMaps& mipmaps);

    MIPMaps getMipmapsOpt() const;

    int getTilesNumber() const;

    int getTilesRows() const;

    int getTilesCols() const;

/* FIXME
 * Change pointer with smartpointers.
 */
private:
    static int class_counter_;

    static GLsizei renderbuffer_size_;

    const std::string log_ID_ = "[ROFT::SICAD]";

    GLFWwindow* window_ = nullptr;

    GLint tiles_num_ = 0;

    GLsizei tiles_cols_ = 0;

    GLsizei tiles_rows_ = 0;

    GLsizei image_width_ = 0;

    GLsizei image_height_ = 0;

    GLfloat cam_fx_;

    GLfloat cam_fy_;

    GLfloat cam_cx_;

    GLfloat cam_cy_;

    glm::mat3 ogl_to_cam_ = glm::mat3(1.0f);

    GLsizei framebuffer_width_ = 0;

    GLsizei framebuffer_height_ = 0;

    GLsizei tile_img_width_ = 0;

    GLsizei tile_img_height_ = 0;

    const GLfloat near_ = 0.001f;

    const GLfloat far_ = 1000.0f;

    std::thread::id main_thread_id_;

    bool show_background_ = false;

    GLenum  show_mesh_mode_ = GL_FILL;

    MIPMaps mesh_mmaps_ = MIPMaps::nearest;

    ROFT::SICADShader* shader_background_ = nullptr;

    ROFT::SICADShader* shader_cad_ = nullptr;

    std::unique_ptr<ROFT::SICADShader> shader_mesh_texture_;

    ROFT::SICADShader* shader_frame_ = nullptr;

    ModelContainer model_obj_;

    GLuint fbo_;

    GLuint texture_color_buffer_;

	GLuint texture_depth_buffer_;

    GLuint texture_depthtest_buffer_;

    GLuint texture_background_;

    GLuint vao_background_;

    GLuint ebo_background_;

    GLuint vbo_background_;

    GLuint vao_frame_;

    GLuint vbo_frame_;

    size_t pbo_number_ = 2;

    GLuint pbo_[2];

    glm::mat4 back_proj_;

    glm::mat4 projection_;

    glm::mat4 getViewTransformationMatrix(const double* cam_x, const double* cam_o);

    void pollOrPostEvent();

    void renderBackground(const cv::Mat& img) const;

    void setWireframe(GLenum mode);

    void factorize_int(const GLsizei area, const GLsizei width_limit, const GLsizei height_limit, GLsizei& width, GLsizei& height);
};

#endif /* ROFT_SUPERIMPOSECAD_H */
