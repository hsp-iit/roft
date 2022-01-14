#===============================================================================
#
# Copyright (C) 2022 Istituto Italiano di Tecnologia (IIT)
#
# This software may be modified and distributed under the terms of the
# GPL-2+ license. See the accompanying LICENSE file for details.
#
#===============================================================================
import ho3d_utils
import os
import cv2


def configure(obj_id):
    cfg = dict()

    cfg['ho3d_root'] = '?/HO3D_v2'
    cfg['ho3d_synthetic_root'] = '?/ho3d_synthetic_format'

    classes_lookup = {'2': '003_cracker_box', '3': '004_sugar_box', '4': '005_tomato_soup_can',
                      '5': '006_mustard_bottle', '8': '009_gelatin_box', '9': '010_potted_meat_can'}

    cfg['obj_id']     = str(obj_id)
    cfg['obj_name']   = classes_lookup[cfg['obj_id']]

    return cfg


def convert_ho3d_seq(cfg, in_dir, seq_id=0):

    print(f'Converting the directory: {in_dir}')

    seq_root = os.path.join(cfg['ho3d_synthetic_root'], cfg['obj_name']+'_'+str(seq_id))
    color_dir = os.path.join(seq_root, 'rgb')
    depth_dir = os.path.join(seq_root, 'depth')
    pose_dir  = os.path.join(seq_root, 'gt')
    mask_dir  = os.path.join(seq_root, 'masks/gt')
    cam_K_out = os.path.join(seq_root, 'cam_K.json')

    os.makedirs(cfg['ho3d_synthetic_root'], exist_ok=True)
    os.makedirs(seq_root, exist_ok=True)
    os.makedirs(color_dir, exist_ok=True)
    os.makedirs(depth_dir, exist_ok=True)
    os.makedirs(pose_dir, exist_ok=True)

    ho3d_imgs_dir = os.path.join(in_dir, 'rgb')
    ho3d_imgs = [img_name for img_name in os.listdir(ho3d_imgs_dir)
                  if img_name.endswith('.png') or img_name.endswith('.jpg')]
    for img_name in ho3d_imgs:
        ho3d_img_path = os.path.join(ho3d_imgs_dir, img_name)
        synth_img_path = os.path.join(color_dir, str(int(img_name.split('.')[0])) + '.png')
        rgb_img = cv2.imread(ho3d_img_path)[:,:,:3]
        cv2.imwrite(synth_img_path, rgb_img)

    ho3d_depth_dir = os.path.join(in_dir, 'depth')
    ho3d_depth = [depth_name for depth_name in os.listdir(ho3d_depth_dir)
                  if depth_name.endswith('.png')]
    for depth_name in ho3d_depth:
        ho3d_depth_path = os.path.join(ho3d_depth_dir, depth_name)
        synth_depth_path = os.path.join(depth_dir, str(int(depth_name.split('.')[0])) + '.float')
        depth_img = ho3d_utils.decode_depth_img(ho3d_depth_path)
        ho3d_utils.write_depth_float(synth_depth_path, depth_img)

    ho3d_mask_dir = os.path.join(in_dir, 'seg')
    if os.path.exists(ho3d_mask_dir):
        os.makedirs(mask_dir, exist_ok=True)
        ho3d_mask = [mask_name for mask_name in os.listdir(ho3d_mask_dir)
                      if mask_name.endswith('.png') or mask_name.endswith('.jpg')]
        for mask_name in ho3d_mask:
            ho3d_mask_path = os.path.join(ho3d_mask_dir, mask_name)
            synth_mask_path = os.path.join(mask_dir,cfg['obj_name']  + '_' + str(int(mask_name.split('.')[0])) + '.png')
            ho3d_utils.write_binary_mask(ho3d_mask_path, synth_mask_path)

    ho3d_meta_dir = os.path.join(in_dir, 'meta')
    ho3d_metas = sorted([pose_name for pose_name in os.listdir(ho3d_meta_dir)
                  if pose_name.endswith('.pkl')])
    synth_poses = open(os.path.join(pose_dir, 'poses.txt'), 'w')
    for meta_name in ho3d_metas:
        ho3d_meta_path = os.path.join(ho3d_meta_dir, meta_name)
        pose_T = ho3d_utils.get_pose(ho3d_meta_path, cfg['obj_id'])
        pose_aa = ho3d_utils.T_to_aa(pose_T)
        pose_aa_str = ''
        for val in pose_aa:
            pose_aa_str += str(val) + ' '
        synth_poses.write(pose_aa_str + '\n')
    synth_poses.close()

    ho3d_utils.write_cam_K(cam_K_out, ho3d_meta_path)


def ho3d_to_synthetic():

    mapping = {'2' : {'train' : ['MC'], 'eval': [] },
               '3' : {'train': ['ShSu', 'SiS', 'SS'], 'eval': [] },
               '5' : {'train' : ['SM'], 'eval': [] },
               '9' : {'train' : ['GPMF'], 'eval': ['MPM'] }
               }

    discarded = ['MC4', 'MC6', 'ShSu10', 'SS1', 'SS2', 'GPMF10', 'GPMF11', 'GPMF12', 'GPMF13', 'GPMF14']

    for obj in mapping:
        cfg = configure(obj)

        train_dir = os.path.join(cfg['ho3d_root'], 'train')
        seq_id = 0
        for train_abbr in mapping[obj]['train']:
            train_dirs = sorted([d for d in os.listdir(train_dir) if d.startswith(train_abbr)
                                  and d.split(train_abbr)[1][0].isdigit() and d not in discarded])
            for d in train_dirs:
                seq_dir = os.path.join(train_dir, d)
                convert_ho3d_seq(cfg, seq_dir, seq_id)
                seq_id += 1

        eval_dir = os.path.join(cfg['ho3d_root'], 'evaluation')
        seq_id = 100
        for val_abbr in mapping[obj]['eval']:
            val_dirs = sorted([d for d in os.listdir(eval_dir) if d.startswith(val_abbr)
                        and d.split(val_abbr)[1][0].isdigit() and d not in discarded])
            for d in val_dirs:
                seq_dir = os.path.join(eval_dir, d)
                convert_ho3d_seq(cfg, seq_dir, seq_id)
                seq_id += 1


if __name__ == '__main__':
    ho3d_to_synthetic()
