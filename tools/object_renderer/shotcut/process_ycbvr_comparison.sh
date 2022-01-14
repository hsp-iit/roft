#===============================================================================
#
# Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT)
#
# This software may be modified and distributed under the terms of the
# GPL-2+ license. See the accompanying LICENSE file for details.
#
#===============================================================================

REPOSITORY_PATH=$PWD
mkdir -p tools/object_renderer/shotcut/tmp

for object_name in 003_cracker_box 006_mustard_bottle; do
    for name in se3tracknet poserbpf ours; do
        ln -sfT $REPOSITORY_PATH/evaluation_output/exp_ycbvr_real/$name/$object_name/sequence_0/output.mp4 $REPOSITORY_PATH/tools/object_renderer/shotcut/tmp/$name.mp4;
    done
    shotcut tools/object_renderer/shotcut/project_comparison.mlt
done
