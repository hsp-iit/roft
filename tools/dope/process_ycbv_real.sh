#===============================================================================
#
# Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT)
#
# This software may be modified and distributed under the terms of the
# GPL-2+ license. See the accompanying LICENSE file for details.
#
#===============================================================================

YCBVSYN_PATH='./dataset/fast-ycb'
DOPE_PATH=`cat ./config/dope_location`
DOPE_VENV_PATH=`cat ./config/dope_venv_location`
GPU_ID=$1

. $DOPE_VENV_PATH/bin/activate
for OBJECT_NAME in 003_cracker_box 006_mustard_bottle; do
    python ./tools/dope/inference.py --dataset-name ycbv_real --dataset-path $YCBVSYN_PATH --dope-path $DOPE_PATH --gpu-id $GPU_ID --output-path $YCBVSYN_PATH --object-name $OBJECT_NAME --rgb-format png
done
