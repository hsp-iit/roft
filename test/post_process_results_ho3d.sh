#===============================================================================
#
# Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT)
#
# This software may be modified and distributed under the terms of the
# GPL-2+ license. See the accompanying LICENSE file for details.
#
#===============================================================================

CONVERSION_TOOL=`cat ./config/conversion_tool_location`/Conversion/nvdu_poses_to_ycbv.py
HO3D_PATH=`cat ./config/ho3d_location`

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

    for sequence_name in `ls -d $HO3D_PATH/$object_name*`; do
        sequence_number=`echo ${sequence_name##*_}`

        for folder in `ls ./results/ours/ho3d/`; do
            IN_FILE_PATH=./results/ours/ho3d/$folder/"$object_name"_"$sequence_number"/pose_estimate.txt
            OUT_FILE_PATH=./results/ours/ho3d/$folder/"$object_name"_"$sequence_number"/pose_estimate_ycb.txt
            echo "Processing  "$IN_FILE_PATH
            rm -f $OUT_FILE_PATH
            python $CONVERSION_TOOL\
                   --format pred\
                   --obj_id $OBJECT_ID\
                   --in_path $IN_FILE_PATH\
                   --out_path $OUT_FILE_PATH

            # Pad results with invalid poses if we run from sequeunces, like HO-3D 006_mustard_bottle_2,
            # where DOPE measurements are invalid from the first frame
            python ./tools/dataset/results_padding/pad_results.py $OUT_FILE_PATH $object_name $sequence_number
        done
    done
done
