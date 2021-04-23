#===============================================================================
#
# Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT)
#
# This software may be modified and distributed under the terms of the
# GPL-2+ license. See the accompanying LICENSE file for details.
#
#===============================================================================

import cv2
import glob
import numpy
import os
import sys
from tqdm import tqdm

masks_path = str(sys.argv[1])
ho3d_path = str(sys.argv[2])
mapping =\
{
    'MC1' : '003_cracker_box_0',
    'MC2' : '003_cracker_box_1',
    'MC5' : '003_cracker_box_2',
    'ShSu12' : '004_sugar_box_0',
    'ShSu13' : '004_sugar_box_1',
    'ShSu14' : '004_sugar_box_2',
    'SiS1' : '004_sugar_box_3',
    'SS3' : '004_sugar_box_4',
    'SM2' : '006_mustard_bottle_0',
    'SM3' : '006_mustard_bottle_1',
    'SM4' : '006_mustard_bottle_2',
    'SM5' : '006_mustard_bottle_3',
    'MPM10' : '010_potted_meat_can_100',
    'MPM11' : '010_potted_meat_can_101',
    'MPM12' : '010_potted_meat_can_102',
    'MPM13' : '010_potted_meat_can_103',
    'MPM14' : '010_potted_meat_can_104'
}


for source_name in mapping:
    target_name = mapping[source_name]
    object_name = '_'.join(target_name.split('_')[:-1])

    source_path = os.path.join(masks_path, source_name)
    target_path = os.path.join(ho3d_path, target_name, 'masks', 'mrcnn_ho3d')

    os.makedirs(target_path, exist_ok = True)

    # For each frame
    for i in tqdm(range(len(glob.glob(os.path.join(ho3d_path, target_name, 'rgb', '*.png'))))):
        frame_instances = []

        instance_idx = 0

        # For each instance
        while True:
            file_path = os.path.join(source_path, str(i).zfill(4) + '_' + object_name + '_' + str(instance_idx) + '.png')
            mask = cv2.imread(file_path, cv2.IMREAD_GRAYSCALE)

            # The instances are over for this frame (or there are none)
            if mask is None:
                break

            frame_instances.append(mask)

            instance_idx += 1

        output_mask = numpy.zeros((480, 640), dtype = numpy.uint8)
        for instance in frame_instances:
            output_mask = cv2.add(output_mask, instance)
        cv2.imwrite(os.path.join(target_path, object_name + '_' + str(i) + '.png'), output_mask)
