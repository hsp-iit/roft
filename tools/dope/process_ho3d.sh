#===============================================================================
#
# Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT)
#
# This software may be modified and distributed under the terms of the
# GPL-2+ license. See the accompanying LICENSE file for details.
#
#===============================================================================

HO3D_PATH=`cat ./config/ho3d_location`
DOPE_PATH='./tools/third_party/Deep_Object_Pose'
DOPE_VENV_PATH='./dope_env'
GPU_ID=$1

. $DOPE_VENV_PATH/bin/activate
for OBJECT_NAME in 003_cracker_box 004_sugar_box 006_mustard_bottle 010_potted_meat_can; do
    python ./tools/dope/inference.py --dataset-name ho3d --dataset-path $HO3D_PATH --dope-path $DOPE_PATH --gpu-id $GPU_ID --output-path $HO3D_PATH --object-name $OBJECT_NAME --rgb-format png
done
