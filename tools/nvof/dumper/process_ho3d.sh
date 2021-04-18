#===============================================================================
#
# Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT)
#
# This software may be modified and distributed under the terms of the
# GPL-2+ license. See the accompanying LICENSE file for details.
#
#===============================================================================

HO3D_PATH=?

for SEQ_NAME in `ls $HO3D_PATH`; do
    OUTPUT_PATH=$HO3D_PATH/$SEQ_NAME/optical_flow/nvof_1_slow
    mkdir -p $OUTPUT_PATH
    ./build/bin/6d-of-tracking-of-dumper $HO3D_PATH/$SEQ_NAME txt png 0 0 640 480 $OUTPUT_PATH
done
