#===============================================================================
#
# Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT)
#
# This software may be modified and distributed under the terms of the
# GPL-2+ license. See the accompanying LICENSE file for details.
#
#===============================================================================

YCBVSYN_PATH='./dataset/fast-ycb'

for OBJECT_NAME in 003_cracker_box 004_sugar_box 005_tomato_soup_can 006_mustard_bottle 009_gelatin_box 010_potted_meat_can; do
    OUTPUT_PATH=$YCBVSYN_PATH/object_motion/$OBJECT_NAME/optical_flow/nvof_1_slow
    mkdir -p $OUTPUT_PATH
    ./build/bin/6d-of-tracking-of-dumper $YCBVSYN_PATH/object_motion/$OBJECT_NAME txt png 0 0 1280 720 $OUTPUT_PATH
done
