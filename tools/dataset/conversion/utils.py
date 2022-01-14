#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Mar 26 14:08:33 2021

@author: Yuriy Onyshchuk
"""

import os
import pyrender
import trimesh
import numpy as np
import imageio
import json
import cv2
import matplotlib.pyplot as plt
from scipy import io
from pytransform3d.rotations import matrix_from_axis_angle
from struct import iter_unpack
from pyquaternion import Quaternion
import shutil


def axis_angle_to_R(pose):
    T = np.zeros((4,4))
    aa = np.array(pose[3:])

    if np.nonzero(aa)[0].size > 0:
        R = matrix_from_axis_angle(aa)
        T[:3,:3] = R
    
    T[0,3] = pose[0]
    T[1,3] = pose[1]
    T[2,3] = pose[2]
    T[3,3] = 1
    
    return T


def T_to_aa(T_mat):
    aa = np.zeros(7)
    aa[:3] = T_mat[:3,3]
    try:
        q = Quaternion(matrix = T_mat[:3,:3])
    except ValueError:
        U, s, VT = np.linalg.svd(T_mat[:3,:3])
        q = Quaternion(matrix = np.dot(U, VT))
    aa[3:6] = q.axis
    aa[6] = q.angle

    return aa



def nvdu_to_ycbv(nvdu_path, ycbv_models_path, obj_id):
    # Object id to object name
    classes_lookup =\
    {
        '2': '003_cracker_box',
        '3': '004_sugar_box',
        '4': '005_tomato_soup_can',
        '5': '006_mustard_bottle',
        '8': '009_gelatin_box',
        '9': '010_potted_meat_can'
     }

    # Paths
    nvdu_obj_settings_path = nvdu_path + '/nvdu/config/object_settings/_ycb_original.json'
    nvdu_model_path = os.path.join(nvdu_path, 'nvdu/data/ycb/original', classes_lookup[str(obj_id)], 'google_16k/textured.obj')
    ycbv_model_path = os.path.join(ycbv_models_path, classes_lookup[str(obj_id)], 'textured_simple.obj')

    # Meshes
    nvdu_original_mesh = trimesh.load(nvdu_model_path)
    ycbv_mesh = trimesh.load(ycbv_model_path)

    # Relative transformation between NVDU aligned and NVDU original
    transform = json.load(open(nvdu_obj_settings_path))
    fixed_model_transform = np.asarray(transform['exported_objects'][int(obj_id)-1]['fixed_model_transform'], dtype='float32')
    aligned_to_original = np.zeros((4,4))

    # Divide by 100.0 to restore standard units
    for i, row in enumerate(fixed_model_transform):
        aligned_to_original[i,:] =  np.array(row) / 100.

    # Restore the scaler of the transformation to 1
    aligned_to_original[3, 3] = 1.

    # Invert direction of transformation
    aligned_to_original = aligned_to_original.T

    # Relative transformation between NVDU original and YCB-V
    original_to_ycbv = np.zeros((4,4))
    original_to_ycbv[0:3, 0:3] = np.eye(3)
    original_to_ycbv[0:3, 3] = nvdu_original_mesh.centroid - ycbv_mesh.centroid
    original_to_ycbv[3, 3] = 1.

    # Relative transformation between NVDU aligned and YCB-V
    aligned_to_ycbv = np.dot(aligned_to_original, original_to_ycbv)

    return aligned_to_ycbv


def get_K_tracknet(camera_K_txt_path):
    cam_K_file = open(camera_K_txt_path)
    cam_K = np.zeros((3,3))
    i = 0
    for row in cam_K_file:
        cam_K[i] = [float(val) for val in row.split(' ') if val != '\n']
        i += 1
    
    return cam_K


def get_K_synthetic(path):
    cam_file = json.load(open(path))
    cam_K = np.zeros((3,3))
    
    cam_K[0,0] = float(cam_file['fx'])
    cam_K[1,1] = float(cam_file['fy'])
    cam_K[0,2] = float(cam_file['cx'])
    cam_K[1,2] = float(cam_file['cy'])
    cam_K[2,2] = 1
    
    return cam_K


def get_obj_pose_tracknet(pose_path):
    pose_file = open(pose_path)
    pose = np.zeros((4,4))
    i = 0
    for row in pose_file:
        pose[i] = [float(val) for val in row.split(' ') if val != '\n']
        i += 1
    
    return pose


def get_obj_pose_and_K_ycbv(anns_path, obj_id):
    meta_dict = io.loadmat(anns_path)

    pose = np.zeros((4,4))
    ind_mapped = np.where(meta_dict['cls_indexes'] == int(obj_id))[0][0]
    pose[:3] = meta_dict['poses'][:,:,ind_mapped]
    pose[3,3] = 1

    return pose, meta_dict['intrinsic_matrix']


def get_obj_poses_synthetic(pose_path):
    pose_file = open(pose_path)
    poses = dict()
    i = 0
    
    for row in pose_file:
        pose_axis_angle = [float(val.split('\n')[0]) for val in row.split(' ') if val.split('\n')[0] != '']
        # Take only last 7 values, skip the first values related to velocities
        T = axis_angle_to_R(pose_axis_angle[-7:])
        poses[i] = T
        i += 1
    
    return poses


def crop_and_resize(img, data_type='uint8'):
    if len(img.shape) == 3:
        img_crop = np.zeros((720,960,3))
    else:
        img_crop = np.zeros((720,960))
    img_crop = img[:,160:1120]
    img_res = cv2.resize(img_crop, (640, 480), interpolation=cv2.INTER_NEAREST)
    
    return np.array(img_res, dtype=data_type)


def pad_and_resize(img, data_type='uint8', depth_factor=0):
    if len(img.shape) == 3:
        img_pad = np.zeros((960,1280,3))
    else:
        img_pad = np.zeros((960,1280))

    if depth_factor != 0:
        img_pad[:120] = 10 * depth_factor
        img_pad[840:] = 10 * depth_factor

    img_pad[120:840] = img
    img_res = cv2.resize(img_pad, (640, 480), interpolation=cv2.INTER_NEAREST)
    
    return np.array(img_res, dtype=data_type)


def convert_depth(path, in_format='synth', factor=1000):
    with open(path, 'rb') as dat:
        # This will give data as a 1D array
        data = list(iter_unpack('f', dat.read()))
    if in_format == 'synth' or in_format == 'real':
        depth = (np.array(data[4:])*factor).reshape(720, 1280)
        depth = pad_and_resize(depth, 'uint16', factor)
    elif in_format == 'ho3d':
        depth = (np.array(data[4:])*factor).reshape(480, 640)
        depth = np.array(np.where(depth==0, 10*factor, depth), dtype='uint16')
    
    return depth


def process_image(synth_img_path, tracknet_img_path, in_format):
    if in_format == 'synth' or in_format == 'real':
        synth_img = cv2.imread(synth_img_path)
        resized_img = pad_and_resize(synth_img)
        cv2.imwrite(tracknet_img_path, resized_img)
    elif in_format == 'ho3d':
        shutil.copy(synth_img_path, tracknet_img_path)


def convert_mask(path, obj_id, in_format='synth'):
    mask = imageio.imread(path)
    if in_format == 'synth' or in_format == 'real':
        mask = pad_and_resize(mask)
    converted = np.where(mask == 255, int(obj_id), 0)
    
    where = np.array(np.where(converted))
    x1, y1 = np.amin(where, axis=1) # top-left, row-column
    x2, y2 = np.amax(where, axis=1) # bottom-right, row-column
    bbox = f' {y1} {x1} {y2} {x2}\n' # column-row, top-left, bottom-right
    
    return converted, bbox


def render_pose(model_path, cam_K, obj_pose):
    os.environ['PYOPENGL_PLATFORM'] = 'egl'
    
    trimesh_model = trimesh.load(model_path)
    mesh = pyrender.Mesh.from_trimesh(trimesh_model)
    scene = pyrender.Scene()
    
    fx, fy, cx, cy = cam_K[0,0], cam_K[1,1], cam_K[0,2], cam_K[1,2]
    camera = pyrender.IntrinsicsCamera(fx, fy, cx, cy)
    cam_pose = np.identity(4)
    cam_pose[1,1] = 0
    cam_pose[2,2] = 0
    cam_pose[1,2] = 1
    cam_pose[2,1] = 1
    
    scene.add(mesh, pose=obj_pose)
    scene.add(camera, pose=cam_pose)

    light = pyrender.SpotLight(color=np.ones(3), intensity=10.0,
                                innerConeAngle=np.pi/16.0,
                                outerConeAngle=np.pi/6.0)
    scene.add(light, pose=cam_pose)

    r = pyrender.OffscreenRenderer(viewport_width=640, viewport_height=480, 
                                   point_size=1.0)
    color, depth = r.render(scene)
    r.delete()
    
    return color, depth


def vis_rendering(color_img, depth_img):
    plt.figure()
    
    plt.subplot(1,2,1)
    plt.axis('off')
    plt.imshow(color_img)
    
    plt.subplot(1,2,2)
    plt.axis('off')
    plt.imshow(depth_img, cmap=plt.cm.gray_r)
    
    plt.show()


def vis_overlap_render(orig_img, color, ren_depth):
    img_overlapped = np.zeros_like(orig_img)
    grayscale = rgb2gray(orig_img)
    for i in range(3):
        img_overlapped[:,:,i] = np.where(ren_depth != 0, color[:,:,i], grayscale)
    
    plt.figure()
    plt.imshow(img_overlapped)


def vis_overlap_mask(orig_img, ren_depth):
    img_overlapped = np.zeros_like(orig_img)
    for i in range(3):
        img_overlapped[:,:,i] = np.where(ren_depth != 0, orig_img[:,:,i], 0)
    
    plt.figure()
    plt.imshow(img_overlapped)


def zero_pad(inp, out_dim):
    zeros = '0' * out_dim
    
    return (zeros + str(inp))[-out_dim:]


def matrix_to_txt(matrix, out_path, velNaN=False):
    file = open(out_path, 'w')
    for row in matrix:
        if velNaN:
            file.write('NaN ' * 6)
        for val in row:
            file.write(str(val))
            file.write(' ')
        file.write('\n')      
    file.close()
    
    
def rgb2gray(rgb):

    r, g, b = rgb[:,:,0], rgb[:,:,1], rgb[:,:,2]
    gray = 0.2989 * r + 0.5870 * g + 0.1140 * b

    return gray