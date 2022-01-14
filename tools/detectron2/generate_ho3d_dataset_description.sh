#===============================================================================
#
# Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT)
#
# This software may be modified and distributed under the terms of the
# GPL-2+ license. See the accompanying LICENSE file for details.
#
#===============================================================================

HO3D_PATH='./dataset/ho3d'
DETECTRON2_VENV_PATH='./tools/third_party/detectron2/env'

. $DETECTRON2_VENV_PATH/bin/activate
python ./tools/detectron2/dataset.py --dataset-name ho3d --dataset-path $HO3D_PATH --generate "true"
