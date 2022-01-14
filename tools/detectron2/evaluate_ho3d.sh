#===============================================================================
#
# Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT)
#
# This software may be modified and distributed under the terms of the
# GPL-2+ license. See the accompanying LICENSE file for details.
#
#===============================================================================

DETECTRON2_VENV_PATH='./tools/third_party/detectron2/env'
GPU_ID=$1

. $DETECTRON2_VENV_PATH/bin/activate
python ./tools/detectron2/evaluate.py --dataset-name ho3d --gpu-id $GPU_ID
