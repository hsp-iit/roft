#===============================================================================
#
# Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT)
#
# This software may be modified and distributed under the terms of the
# GPL-2+ license. See the accompanying LICENSE file for details.
#
#===============================================================================

HO3D_PATH='./dataset/ho3d/'

for object_name in `cat config/classes_ho3d.txt`
do
    for sequence_name in `ls -d $HO3D_PATH/$object_name*`; do
        sequence_number=`echo ${sequence_name##*_}`
        bash test/test_full_gt_mask_gt_pose_ho3d.sh $object_name $sequence_number
    done
done
