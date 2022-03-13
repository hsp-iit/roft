#===============================================================================
#
# Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT)
#
# This software may be modified and distributed under the terms of the
# GPL-2+ license. See the accompanying LICENSE file for details.
#
#===============================================================================

YCBVSYN_PATH='./dataset/fast-ycb'

for OBJECT_NAME in 003_cracker_box_real 006_mustard_bottle_real; do
    OUTPUT_PATH=$YCBVSYN_PATH/$OBJECT_NAME/optical_flow/nvof_1_slow
    mkdir -p $OUTPUT_PATH
    ./build/bin/ROFT-of-dumper $YCBVSYN_PATH/$OBJECT_NAME txt png 0 0 1280 720 $OUTPUT_PATH
done
