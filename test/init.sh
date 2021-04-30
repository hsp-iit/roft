#===============================================================================
#
# Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT)
#
# This software may be modified and distributed under the terms of the
# GPL-2+ license. See the accompanying LICENSE file for details.
#
#===============================================================================

CONVERSION_TOOL_PATH=`cat ./config/conversion_tool_location`/Conversion/
HO3D_PATH=`cat ./config/ho3d_location`
YCBV_SYN_PATH=`cat ./config/ycbv_synthetic_location`

for object_name in `cat config/classes.txt`
do
    OBJECT_ID=""
    if [ $object_name == "003_cracker_box" ]; then
        OBJECT_ID="2";
    elif [ $object_name == "004_sugar_box" ]; then
        OBJECT_ID="3"
    elif [ $object_name == "005_tomato_soup_can" ]; then
        OBJECT_ID="4"
    elif [ $object_name == "006_mustard_bottle" ]; then
        OBJECT_ID="5"
    elif [ $object_name == "009_gelatin_box" ]; then
        OBJECT_ID="8"
    elif [ $object_name == "010_potted_meat_can" ]; then
        OBJECT_ID="9"
    fi

    echo "[ycbv_synthetic] Converting ground truth to YCB-Video format for sequence "$object_name
    python $CONVERSION_TOOL_PATH/nvdu_poses_to_ycbv.py\
           --format gt\
           --obj_id $OBJECT_ID\
           --in_path $YCBV_SYN_PATH/object_motion/$object_name/gt/poses.txt\
           --out_path $YCBV_SYN_PATH/object_motion/$object_name/gt/poses_ycb.txt

    echo "[ycbv_synthetic] Converting DOPE poses to YCB-Video format for sequence "$object_name
    python $CONVERSION_TOOL_PATH/nvdu_poses_to_ycbv.py\
           --format gt\
           --obj_id $OBJECT_ID\
           --in_path $YCBV_SYN_PATH/object_motion/$object_name/dope/poses.txt\
           --out_path $YCBV_SYN_PATH/object_motion/$object_name/dope/poses_ycb.txt
done

for object_name in `cat config/classes_ho3d.txt`
do
    OBJECT_ID=""
    if [ $object_name == "003_cracker_box" ]; then
        OBJECT_ID="2";
    elif [ $object_name == "004_sugar_box" ]; then
        OBJECT_ID="3"
    elif [ $object_name == "005_tomato_soup_can" ]; then
        OBJECT_ID="4"
    elif [ $object_name == "006_mustard_bottle" ]; then
        OBJECT_ID="5"
    elif [ $object_name == "009_gelatin_box" ]; then
        OBJECT_ID="8"
    elif [ $object_name == "010_potted_meat_can" ]; then
        OBJECT_ID="9"
    fi

    for sequence in $HO3D_PATH/$object_name*; do
        echo "[ho3d] Converting ground truth to NVDU format for sequence "$sequence
        python $CONVERSION_TOOL_PATH/ycbv_poses_to_nvdu.py\
               --format gt\
               --obj_id $OBJECT_ID\
               --in_path $sequence/gt/poses.txt\
               --out_path $sequence/gt/poses_nvdu.txt

        echo "[ho3d] Converting DOPE poses to YCB-Video format for sequence "$sequence
        python $CONVERSION_TOOL_PATH/nvdu_poses_to_ycbv.py\
               --format gt\
               --obj_id $OBJECT_ID\
               --in_path $sequence/dope/poses.txt\
               --out_path $sequence/dope/poses_ycb.txt
    done
done
