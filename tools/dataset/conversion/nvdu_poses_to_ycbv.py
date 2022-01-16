#===============================================================================
#
# Copyright (C) 2022 Istituto Italiano di Tecnologia (IIT)
#
# This software may be modified and distributed under the terms of the
# GPL-2+ license. See the accompanying LICENSE file for details.
#
#===============================================================================

import utils
import argparse
import numpy
import sys


def configure(args):
    cfg = dict()
    cfg['nvdu_path'] = './tools/third_party/Dataset_Utilities/'
    cfg['ycbv_models_path'] = './YCB_Video_Models/'

    cfg['out_path']  = args.out_path
    cfg['in_path'] = args.in_path

    cfg['obj_id'] = args.obj_id

    return cfg


def main():
    parser = argparse.ArgumentParser(description='Convert predicted poses in order to refer to the YCB-Video models instead of the NVDU models.')
    parser.add_argument("--out_path", type=str, default="out.txt")
    parser.add_argument("--in_path", type=str)
    parser.add_argument("--obj_id", type=int)
    parser.add_argument("--format", type=str, default='gt', help='Input format: "gt" or "pred"')
    args = parser.parse_args()

    cfg = configure(args)

    nvdu_aligned_to_ycbv = utils.nvdu_to_ycbv(cfg['nvdu_path'], cfg['ycbv_models_path'], cfg['obj_id'])
    u, s, vh = numpy.linalg.svd(nvdu_aligned_to_ycbv[0:3, 0:3], full_matrices=False)
    nvdu_aligned_to_ycbv[0:3, 0:3] = u @ vh

    #     ycbv_T = numpy.dot(nvdu_T, nvdu_aligned_to_ycbv)

    frames_in = open(args.in_path)

    with open(args.out_path, 'w') as f:
            for line in frames_in:

                items = line.split(' ')
                counter = int(items[-1].rstrip('\n'))

                x = float(items[0])
                y = float(items[1])
                z = float(items[2])
                axis_x = float(items[3])
                axis_y = float(items[4])
                axis_z = float(items[5])
                angle = float(items[6])

                rot = Quaternion(axis=[axis_x, axis_y, axis_z], angle = angle).transformation_matrix
                rot[0, 3] = x
                rot[1, 3] = y
                rot[2, 3] = z

                T = rot @ nvdu_aligned_to_ycbv
                q = Quaternion(matrix = T[0:3, 0:3])

                string = format(T[0, 3], '.6f') + ' ' +\
                         format(T[1, 3], '.6f') + ' ' +\
                         format(T[2, 3], '.6f') + ' ' +\
                         format(q.axis[0], '.6f') + ' ' +\
                         format(q.axis[1], '.6f') + ' ' +\
                         format(q.axis[2], '.6f') + ' ' +\
                         format(q.angle, '.6f') + ' ' +\
                         '0.0 0.0 0.0 0.0 0.0 0.0 ' + str(counter) + '\n'
                f.write(string)



if __name__ == '__main__':
    main()
