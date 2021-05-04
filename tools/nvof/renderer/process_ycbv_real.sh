#===============================================================================
#
# Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT)
#
# This software may be modified and distributed under the terms of the
# GPL-2+ license. See the accompanying LICENSE file for details.
#
#===============================================================================

YCBVSYN_PATH=`cat ./config/ycbv_synthetic_location`

for OBJECT_NAME in 003_cracker_box_real 006_mustard_bottle_real; do
    OUTPUT_PATH=$YCBVSYN_PATH/object_motion/$OBJECT_NAME/optical_flow/nvof_1_slow/debugging
    mkdir -p $OUTPUT_PATH
    ./build/bin/6d-of-tracking-of-renderer $YCBVSYN_PATH/object_motion/$OBJECT_NAME nvof_1_slow 1 0 1280 720 $OUTPUT_PATH
done
