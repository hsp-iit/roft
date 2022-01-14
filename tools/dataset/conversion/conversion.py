import os
import numpy as np
from scipy import io
import utils
import argparse
import cv2
import ho3d_utils


### Configure the script with paths needed for successful conversion
class Cfg:

    def __init__(self):
        self.nvdu_path    = './tools/third_party/Dataset_Utilities/'
        self.ycbv_models_dir   = './YCB_Video_Models/'
        self.model_name = 'textured_simple.obj'

        self.PoseRBPF_root = '?/PoseRBPF'

        self.classes_lookup = {'2': '003_cracker_box', '3': '004_sugar_box', '4': '005_tomato_soup_can',
                          '5': '006_mustard_bottle', '8': '009_gelatin_box', '9': '010_potted_meat_can'}

        self.target_img_width  = 640
        self.target_img_height = 480


    def configure_synthetic(self, obj_id):
        self.synthetic_root = '?/synthetic-ycb-video-dataset'
        self.synthetic_tracknet_root = '?/synthetic_tracknet_format'
        self.synthetic_ycbv_root     = '?/synthetic_ycbv_format'

        self.obj_id     = str(obj_id)
        self.obj_name   = self.classes_lookup[self.obj_id]

        self.gt = True

        self.rbpf_seq_id = 10


    def configure_ho3d(self, obj_id):
        self.synthetic_root = '?/ho3d_synthetic_format'
        self.synthetic_tracknet_root = '?/ho3d_tracknet_format'
        self.synthetic_ycbv_root     = '?/ho3d_ycbv_format'

        self.obj_id     = str(obj_id)
        self.obj_name   = self.classes_lookup[self.obj_id]

        self.gt = True

        self.rbpf_seq_id = 20


    def configure_real(self, obj_id):
        self.synthetic_root = '?/synthetic_real'
        self.synthetic_tracknet_root = '?/real_tracknet_format'
        self.synthetic_ycbv_root     = '?/real_ycbv_format'

        self.obj_id     = str(obj_id)
        self.obj_name   = self.classes_lookup[self.obj_id]

        self.gt = False

        self.rbpf_seq_id = 30



### Convert the Synthetic dataset to the modified YCB-Video format, required by the se3-TrackNet
### Use zero-padding to resize the data
def synthetic_to_tracknet(in_format='synth', dope=False):
    obj_ids = ['2', '3', '4', '5', '8', '9']
    seq_id = 1

    config = Cfg()
    for obj_id in obj_ids:
        if in_format == 'synth':
            config.configure_synthetic(obj_id)
        elif in_format == 'ho3d':
            config.configure_ho3d(obj_id)
        elif in_format == 'real':
            config.configure_real(obj_id)

        scenes = sorted([scene for scene in os.listdir(config.synthetic_root)
                  if scene.startswith(config.obj_name)
                  and os.path.isdir(os.path.join(config.synthetic_root, scene))])
        for scene in scenes:
            in_root = os.path.join(config.synthetic_root, scene)
            print("Converting scene: ", scene)

            seq_root = os.path.join(config.synthetic_tracknet_root, utils.zero_pad(seq_id, 4))
            color_dir = os.path.join(seq_root, 'color')
            depth_dir = os.path.join(seq_root, 'depth_filled')
            pose_dir  = os.path.join(seq_root, 'pose_gt', config.obj_id)
            mask_dir  = os.path.join(seq_root, 'seg')
            cam_K_out = os.path.join(seq_root, 'cam_K.txt')

            os.makedirs(config.synthetic_tracknet_root, exist_ok=True)
            os.makedirs(seq_root, exist_ok=True)
            os.makedirs(color_dir, exist_ok=True)
            os.makedirs(depth_dir, exist_ok=True)
            os.makedirs(pose_dir, exist_ok=True)
            os.makedirs(mask_dir, exist_ok=True)

            cam_K = utils.get_K_synthetic(os.path.join(in_root, 'cam_K.json'))
            utils.matrix_to_txt(cam_K, cam_K_out)

            synth_imgs_dir = os.path.join(in_root, 'rgb')
            synth_imgs = [img_name for img_name in os.listdir(synth_imgs_dir)
                          if img_name.endswith('.png') or img_name.endswith('.jpg')]
            for img_name in synth_imgs:
                synth_img_path = os.path.join(synth_imgs_dir, img_name)
                tracknet_img_path = os.path.join(color_dir, utils.zero_pad(img_name.split('.')[0], 6) + '.png')
                utils.process_image(synth_img_path, tracknet_img_path, in_format)

            synth_depth_dir = os.path.join(in_root, 'depth')
            synth_depth = [depth_name for depth_name in os.listdir(synth_depth_dir)
                          if depth_name.endswith('.float')]
            for depth_name in synth_depth:
                synth_depth_path = os.path.join(synth_depth_dir, depth_name)
                tracknet_depth_path = os.path.join(depth_dir, utils.zero_pad(depth_name.split('.')[0], 7) + '.png')
                depth_img = utils.convert_depth(synth_depth_path, in_format=in_format)
                cv2.imwrite(tracknet_depth_path, depth_img)

            synth_mask_dir = os.path.join(in_root, 'masks/gt')
            if os.path.exists(synth_mask_dir):
                synth_mask = [mask_name for mask_name in os.listdir(synth_mask_dir)
                              if mask_name.endswith('.png')]
                for mask_name in synth_mask:
                    synth_mask_path = os.path.join(synth_mask_dir, mask_name)
                    tracknet_mask_path = os.path.join(mask_dir, utils.zero_pad(mask_name.split('.')[0].split('_')[-1], 6) + '.png')
                    mask_img, _ = utils.convert_mask(synth_mask_path, config.obj_id, in_format=in_format)
                    cv2.imwrite(tracknet_mask_path, np.array(mask_img, dtype='uint8'))

            if in_format == 'synth' or dope:
                aligned_to_ycbv = utils.nvdu_to_ycbv(config.nvdu_path, config.ycbv_models_dir, obj_id)

            if config.gt:
                poses_path = os.path.join(in_root, 'gt/poses.txt')
                poses = utils.get_obj_poses_synthetic(poses_path)
                for pose_id, out_pose in poses.items():
                    out_pose_path = os.path.join(pose_dir, utils.zero_pad(pose_id, 6) + '.txt')
                    if in_format == 'synth':
                        out_pose = np.dot(out_pose, aligned_to_ycbv)
                    utils.matrix_to_txt(out_pose, out_pose_path)

            if dope:
                dope_dir = os.path.join(seq_root, 'dope', config.obj_id)
                os.makedirs(dope_dir, exist_ok=True)
                dope_path  = os.path.join(in_root, 'dope/poses.txt')
                dope_poses = utils.get_obj_poses_synthetic(dope_path)
                for dope_pose_id, dope_pose in dope_poses.items():
                    out_dope_pose_path = os.path.join(dope_dir, utils.zero_pad(dope_pose_id, 6) + '.txt')
                    dope_pose = np.dot(dope_pose, aligned_to_ycbv)
                    utils.matrix_to_txt(dope_pose, out_dope_pose_path)

            seq_id += 1


### Convert the Synthetic dataset to the modified YCB-Video format, required by the PoseRBPF
### Use zero-padding to resize the data
def synthetic_to_ycbv(in_format='synth', dope=False):
    obj_ids = ['2', '3', '4', '5', '8', '9']
    seq_id = 1
    factor_depth = 10000

    # Missing: center, rotation_translation_matrix, vertmap
    mat_dict = dict()
    mat_dict['__header__'] = b'MATLAB 5.0 MAT-file, Platform: GLNXA64, Created on: Mon Apr 12 10:42:04 2021'
    mat_dict['__globals__'] = []
    mat_dict['__version__'] = '1.0'
    mat_dict['factor_depth'] = [[factor_depth]]

    config = Cfg()
    for obj_id in obj_ids:
        if in_format == 'synth':
            config.configure_synthetic(obj_id)
        elif in_format == 'ho3d':
            config.configure_ho3d(obj_id)
        elif in_format == 'real':
            config.configure_real(obj_id)

        if not os.path.exists(config.synthetic_ycbv_root):
            os.makedirs(config.synthetic_ycbv_root)
            keyframes_dir = os.path.join(config.synthetic_ycbv_root, 'keyframes')
            os.makedirs(keyframes_dir)
            keyframes_file = open(os.path.join(keyframes_dir, 'keyframe.txt'), 'w')
            keyframes_file.close()

        scenes = sorted([scene for scene in os.listdir(config.synthetic_root)
                  if scene.startswith(config.obj_name)
                  and os.path.isdir(os.path.join(config.synthetic_root, scene))])
        obj_seq_id = 0
        for scene in scenes:
            in_root = os.path.join(config.synthetic_root, scene)
            print("Converting scene: ", scene)

            seq_root = os.path.join(config.synthetic_ycbv_root, utils.zero_pad(seq_id, 4))
            os.makedirs(seq_root, exist_ok=True)

            cam_K = utils.get_K_synthetic(os.path.join(in_root, 'cam_K.json'))
            mat_dict['intrinsic_matrix'] = cam_K
            mat_dict['cls_indexes'] = [[int(obj_id)]]

            synth_imgs_dir = os.path.join(in_root, 'rgb')
            synth_imgs = [img_name for img_name in os.listdir(synth_imgs_dir)
                          if img_name.endswith('.png') or img_name.endswith('.jpg')]
            for img_name in synth_imgs:
                synth_img_path = os.path.join(synth_imgs_dir, img_name)
                ycbv_img_path = os.path.join(seq_root, utils.zero_pad(int(img_name.split('.')[0])+1, 6) + '-color.png')
                utils.process_image(synth_img_path, ycbv_img_path, in_format)

            synth_depth_dir = os.path.join(in_root, 'depth')
            synth_depth = [depth_name for depth_name in os.listdir(synth_depth_dir)
                          if depth_name.endswith('.float')]
            for depth_name in synth_depth:
                synth_depth_path = os.path.join(synth_depth_dir, depth_name)
                ycbv_depth_path = os.path.join(seq_root, utils.zero_pad(int(depth_name.split('.')[0])+1, 6) + '-depth.png')
                depth_img = utils.convert_depth(synth_depth_path, in_format=in_format, factor=factor_depth)
                cv2.imwrite(ycbv_depth_path, depth_img)

            synth_mask_dir = os.path.join(in_root, 'masks/gt')
            if os.path.exists(synth_mask_dir):
                synth_mask = [mask_name for mask_name in os.listdir(synth_mask_dir)
                              if mask_name.endswith('.png')]
                for mask_name in synth_mask:
                    synth_mask_path = os.path.join(synth_mask_dir, mask_name)
                    ycbv_mask_path = os.path.join(seq_root, utils.zero_pad(int(mask_name.split('.')[0].split('_')[-1])+1, 6) + '-label.png')
                    mask_img, bbox = utils.convert_mask(synth_mask_path, config.obj_id, in_format=in_format)
                    cv2.imwrite(ycbv_mask_path, np.array(mask_img, dtype='uint8'))

                    # Note: BBox is computed based on the object mask. Zero-padding causes mismatch w.r.t. the YCB-V since YCB-V bbox accounts for occlusions
                    ycbv_bbox_path = os.path.join(seq_root, utils.zero_pad(int(mask_name.split('.')[0].split('_')[-1])+1, 6) + '-box.txt')
                    bbox_str = config.obj_name + bbox
                    file = open(ycbv_bbox_path, 'w')
                    file.write(bbox_str)
                    file.close()

            if in_format == 'synth' or dope:
                aligned_to_ycbv = utils.nvdu_to_ycbv(config.nvdu_path, config.ycbv_models_dir, obj_id)

            if dope:
                dope_path  = os.path.join(in_root, 'dope/poses.txt')
                dope_poses = utils.get_obj_poses_synthetic(dope_path)

            if config.gt:
                poses_path = os.path.join(in_root, 'gt/poses.txt')
                poses = utils.get_obj_poses_synthetic(poses_path)

            for pose_id in range(len(synth_imgs)):
                out_pose_path = os.path.join(seq_root, utils.zero_pad(int(pose_id)+1, 6) + '-meta.mat')

                if in_format == 'synth' and config.gt:
                    out_pose = np.dot(poses[pose_id], aligned_to_ycbv)

                if dope:
                    dope_pose = np.dot(dope_poses[pose_id], aligned_to_ycbv)
                    mat_dict['dope_poses'] = np.reshape(dope_pose[:3], (3,4,1))

                if config.gt:
                    mat_dict['poses'] = np.reshape(out_pose[:3], (3,4,1))
                io.savemat(out_pose_path, mat_dict)

                if not os.path.exists(synth_mask_dir):
                    # Note: BBox is computed based on the object pose.
                    ycbv_bbox_path = os.path.join(seq_root, utils.zero_pad(int(pose_id)+1, 6) + '-box.txt')
                    model_path = os.path.join(config.ycbv_models_dir, config.obj_name, config.model_name)
                    if config.gt:
                        _, depth = utils.render_pose(model_path, cam_K, out_pose)
                    else:
                        try:
                            _, depth = utils.render_pose(model_path, cam_K, dope_pose)
                            bbox = ho3d_utils.produce_bbox_txt(depth)
                        except:
                            print('Invalid DOPE pose. Providing entire image as BBox.')
                            bbox = f' 0 0 {config.target_img_width} {config.target_img_height}\n'
                    bbox_str = config.obj_name + bbox
                    file = open(ycbv_bbox_path, 'w')
                    file.write(bbox_str)
                    file.close()

            poserbpf_ycb_dir = os.path.join(config.PoseRBPF_root, 'datasets/YCB')
            obj_dir = os.path.join(poserbpf_ycb_dir, config.obj_name)
            poserbpf_seq_txt = open(os.path.join(obj_dir, 'seq' + str(config.rbpf_seq_id + obj_seq_id) + '.txt'), 'w')
            poserbpf_seq_txt.write(utils.zero_pad(seq_id, 4) + '/000001\n')
            poserbpf_seq_txt.write(utils.zero_pad(seq_id, 4) + f'/{utils.zero_pad(int(pose_id)+1, 6)}\n')
            poserbpf_seq_txt.close()

            seq_id += 1
            obj_seq_id += 1


def boolean_string(s):
    if s not in {'False', 'True', True, False}:
        raise ValueError('Not a valid boolean string')
    return s == 'True' or s == True


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Convert the Synthetic dataset format to the YCB-Video format: original or tracknet format. \
                                     Usage example: python conversion.py --in_format ("synth", "real" or "ho3d") --out_format ("tracknet" or "ycbv") --dope True (if applicable)')
    parser.add_argument("--in_format", type=str, default="ho3d")
    parser.add_argument("--out_format", type=str, default="ycbv")
    parser.add_argument("--dope", type=bool, default=False)
    args = parser.parse_args()
    args.dope = boolean_string(args.dope)

    globals()['synthetic_to_' + args.out_format](args.in_format, args.dope)
