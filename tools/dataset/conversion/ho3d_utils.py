import numpy as np
import imageio
import json
import cv2
import struct
from pyquaternion import Quaternion


def get_pose(anns_path, obj_id):
    meta_dict = np.load(anns_path, allow_pickle=True)

    pose = np.zeros((4,4))
    pose[3,3] = 1

    """
    Conversion taken from the Function: convertPoseOpenDR_DIRT(rot, trans) of:
        https://github.com/shreyashampali/HOnnotate/blob/d94f6b7e1c09a8c7229799a0d48fdc5f2b416780/optimization/ghope/utils.py#L579
    """
    coordChangMat = np.array([[1., 0., 0.], [0., -1., 0.], [0., 0., -1.]])
    pose[:3,:3] = coordChangMat.dot(cv2.Rodrigues(meta_dict['objRot'])[0])
    pose[:3,3] = meta_dict['objTrans'].dot(coordChangMat.T)

    return pose


def decode_depth_img(inFileName):
    '''
    Decode the depth image to depth map in meters
    :param inFileName: input file name
    :return: depth map (float) in meters
    The function taken from:
        https://github.com/shreyashampali/ho3d/blob/466b58262b2cec85358ccfc8443ff7ced7366df6/utils/vis_utils.py#L380
    '''
    depth_scale = 0.00012498664727900177
    depth_img = cv2.imread(inFileName)
    dpt = depth_img[:, :, 2] + depth_img[:, :, 1] * 256
    dpt = dpt * depth_scale

    return dpt


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


def read_depth_float(path):
    with open(path, 'rb') as dat:
        # This will give data as a 1D array
        data = list(struct.iter_unpack('f', dat.read()))
    depth = (np.array(data[4:])).reshape(480, 640)

    return depth


def write_depth_float(path, depth):
    f = open(path,'wb')
    f.write(struct.pack('=Q', depth.shape[1]))
    f.write(struct.pack('=Q', depth.shape[0]))
    f.write(depth.astype('float32', order='C').tobytes())
    f.close()


def write_depth_png(path, depth, factor=1000):
    imageio.imwrite(path, np.array(depth*factor, dtype='uint16'))


def write_binary_mask(in_path, out_path):
    mask = imageio.imread(in_path)
    mask = np.where(mask[:,:,2]>150, 255, 0)
    full_mask = cv2.resize(mask, (640, 480), interpolation=cv2.INTER_NEAREST)
    imageio.imwrite(out_path, np.array(full_mask, dtype='uint8'))


def write_cam_K(out_name, meta_path):
    meta_dict = np.load(meta_path, allow_pickle=True)
    cam_K = meta_dict['camMat']

    cam_dict = {
                "name": "Camera (640x480)",
                "width": 640,
                "height": 480,
                "fx": str(cam_K[0,0]),
                "fy": str(cam_K[1,1]),
                "cx": str(cam_K[0,2]),
                "cy": str(cam_K[1,2])
                }

    out_file = open(out_name, 'w')
    json.dump(cam_dict, out_file, indent=1)
    out_file.close()


def produce_bbox_txt(depth):

    where = np.array(np.where(depth))
    x1, y1 = np.amin(where, axis=1) # top-left, row-column
    x2, y2 = np.amax(where, axis=1) # bottom-right, row-column
    bbox = f' {y1} {x1} {y2} {x2}\n' # column-row, top-left, bottom-right

    return bbox
