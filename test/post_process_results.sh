#===============================================================================
#
# Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT)
#
# This software may be modified and distributed under the terms of the
# GPL-2+ license. See the accompanying LICENSE file for details.
#
#===============================================================================

CONVERSION_TOOL=`cat ./config/conversion_tool_location`/Conversion/nvdu_poses_to_ycbv.py
YCBV_SYN_PATH=`cat ./config/ycbv_synthetic_location`
OBJECT_NAME=$1
INPUT_PATH=$2

OBJECT_ID=""
if [ $OBJECT_NAME == "003_cracker_box" ]; then
    OBJECT_ID="2";
elif [ $OBJECT_NAME == "004_sugar_box" ]; then
    OBJECT_ID="3"
elif [ $OBJECT_NAME == "005_tomato_soup_can" ]; then
    OBJECT_ID="4"
elif [ $OBJECT_NAME == "006_mustard_bottle" ]; then
    OBJECT_ID="5"
elif [ $OBJECT_NAME == "009_gelatin_box" ]; then
    OBJECT_ID="8"
elif [ $OBJECT_NAME == "010_potted_meat_can" ]; then
    OBJECT_ID="9"
fi

python $CONVERSION_TOOL\
       --format gt\
       --obj_id $OBJECT_ID\
       --in_path $YCBV_SYN_PATH/object_motion/$OBJECT_NAME/gt/poses.txt\
       --out_path $YCBV_SYN_PATH/object_motion/$OBJECT_NAME/gt/poses_ycb.txt

rm -f $INPUT_PATH/pose_estimate_ycb.txt
python $CONVERSION_TOOL\
       --format pred\
       --obj_id $OBJECT_ID\
       --in_path $INPUT_PATH/pose_estimate.txt\
       --out_path $INPUT_PATH/pose_estimate_ycb.txt
