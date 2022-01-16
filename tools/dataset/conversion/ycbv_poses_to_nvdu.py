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
    cfg['ycbv_models_path'] = './YCB_Video_Models/models/'

    cfg['out_path']  = args.out_path
    cfg['in_path'] = args.in_path

    cfg['obj_id'] = args.obj_id

    return cfg


def main():
    parser = argparse.ArgumentParser(description='Convert predicted poses in order to refer to the NVDU models instead of the YCB-Video models.')
    parser.add_argument("--out_path", type=str, default="out.txt")
    parser.add_argument("--in_path", type=str)
    parser.add_argument("--obj_id", type=int)
    parser.add_argument("--format", type=str, default='gt', help='Input format: "gt" or "pred"')
    args = parser.parse_args()

    cfg = configure(args)

    if args.format=='gt':
        velNaN = False
    elif args.format=='pred':
        velNaN = True
    else:
        print(f'Input format "{args.format}" not supported.')
        sys.exit(1)

    nvdu_aligned_to_ycbv = utils.nvdu_to_ycbv(cfg['nvdu_path'], cfg['ycbv_models_path'], cfg['obj_id'])
    poses = utils.get_obj_poses_synthetic(cfg['in_path'])
    nvdu_aa = list()
    for i, ycbv_T in poses.items():
        nvdu_T = numpy.dot(ycbv_T, numpy.linalg.inv(nvdu_aligned_to_ycbv))
        nvdu_aa.append(utils.T_to_aa(nvdu_T))

    utils.matrix_to_txt(nvdu_aa, cfg['out_path'], velNaN)


if __name__ == '__main__':
    main()
