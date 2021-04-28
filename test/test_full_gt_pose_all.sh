#===============================================================================
#
# Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT)
#
# This software may be modified and distributed under the terms of the
# GPL-2+ license. See the accompanying LICENSE file for details.
#
#===============================================================================

for object_name in `cat config/classes.txt`
do
    bash test/test_full_gt_pose.sh $object_name
done
