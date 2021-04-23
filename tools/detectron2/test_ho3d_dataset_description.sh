#===============================================================================
#
# Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT)
#
# This software may be modified and distributed under the terms of the
# GPL-2+ license. See the accompanying LICENSE file for details.
#
#===============================================================================

DETECTRON2_VENV_PATH=`cat ./config/detectron2_venv_location`

. $DETECTRON2_VENV_PATH/bin/activate
python ./tools/detectron2/dataset_tester.py --dataset-name ho3d
