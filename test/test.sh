#===============================================================================
#
# Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT)
#
# This software may be modified and distributed under the terms of the
# GPL-2+ license. See the accompanying LICENSE file for details.
#
#===============================================================================

if [ -f roft_env/bin/activate ]; then
   . roft_env/bin/activate
fi

# arguments
OBJECT_NAME=$1
GT_MASK=$2
GT_OF=$3
GT_POSE=$4
NO_OUT_REJ=$5
NO_POSE_SYNC=$6
NO_FLOW_AID=$7
NO_POSE=$8
NO_VEL=$9

# paths and names
EXECUTABLE=ROFT-tracker
FASTYCB_PATH='./dataset/fast-ycb'
CONFIG_ROOT_PATH=./config/
OBJECT_ROOT_PATH=$FASTYCB_PATH/$OBJECT_NAME/
OUTPUT_ROOT_PATH=./results/ROFT_results/fastycb/

MODEL_NAME=$OBJECT_NAME
if [ "$MODEL_NAME" == "003_cracker_box_real" ]; then
    MODEL_NAME=003_cracker_box
    OUTPUT_ROOT_PATH=./results/ROFT_results/fastycb_qual/
elif [ "$MODEL_NAME" == "006_mustard_bottle_real" ]; then
    MODEL_NAME=006_mustard_bottle
    OUTPUT_ROOT_PATH=./results/ROFT_results/fastycb_qual/
fi

# camera intrinsics
FX=`cat $OBJECT_ROOT_PATH/cam_K.json | head -n 5 | tail -n 1`
FX=`echo $FX | xargs`
FX=`echo ${FX##*:}`
FX=${FX::-1}

FY=`cat $OBJECT_ROOT_PATH/cam_K.json | head -n 6 | tail -n 1`
FY=`echo $FY | xargs`
FY=`echo ${FY##*:}`
FY=${FY::-1}

CX=`cat $OBJECT_ROOT_PATH/cam_K.json | head -n 7 | tail -n 1`
CX=`echo $CX | xargs`
CX=`echo ${CX##*:}`
CX=${CX::-1}

CY=`cat $OBJECT_ROOT_PATH/cam_K.json | head -n 8 | tail -n 1`
CY=`echo $CY | xargs`
CY=`echo ${CY##*:}`

# configuration matrix
MASK_SET="mrcnn_ycbv_bop_pbr"
OF_SET="nvof_1_slow"
POSE_SET="dope"
USE_OUTREJ="true"
USE_POSE_RESYNC="true"
USE_FLOW_AIDED="true"
USE_POSE_MEASUREMENT="true"
USE_VEL_MEASUREMENT="true"
SIGMA_ANG_VEL="1.0,1.0,1.0"
P_COV_Q="0.0001,0.0001,0.0001"

LOG_POSTFIX="full"
if [ "$GT_MASK" == "true" ]; then
    MASK_SET="gt"
fi
LOG_POSTFIX=${LOG_POSTFIX}"_mask_"${MASK_SET}

if [ "$GT_OF" == "true" ]; then
    OF_SET="gt"
fi
LOG_POSTFIX=${LOG_POSTFIX}"_of_"${OF_SET}

if [ "$GT_POSE" == "true" ]; then
    POSE_SET="gt"
fi
LOG_POSTFIX=${LOG_POSTFIX}"_pose_"${POSE_SET}

if [ "$NO_OUT_REJ" == "true" ]; then
    LOG_POSTFIX=${LOG_POSTFIX}"_no_outrej"
    USE_OUTREJ="false"
fi

if [ "$NO_POSE_SYNC" == "true" ]; then
    LOG_POSTFIX=${LOG_POSTFIX}"_no_posesync"
    USE_POSE_RESYNC="false"
fi

if [ "$NO_FLOW_AID" == "true" ]; then
    LOG_POSTFIX=${LOG_POSTFIX}"_no_flowaid"
    USE_FLOW_AIDED="false"
fi

if [ "$NO_VEL" == "true" ]; then
    LOG_POSTFIX=${LOG_POSTFIX}"_no_velocity"
    USE_VEL_MEASUREMENT="false"
    USE_OUTREJ="false"
    USE_POSE_RESYNC="false"
    SIGMA_ANG_VEL="0.01,0.01,0.01"
    P_COV_Q="0.01,0.01,0.01"
fi

if [ "$NO_POSE" == "true" ]; then
    LOG_POSTFIX=${LOG_POSTFIX}"_no_pose"
    USE_POSE_MEASUREMENT="false"
    USE_OUTREJ="false"
    USE_POSE_RESYNC="false"
fi

INITIAL_POSE_STRING=`head -n 1 $OBJECT_ROOT_PATH/dope/poses.txt`
INITIAL_POSE_ARRAY=($INITIAL_POSE_STRING)
INITIAL_POSITION=`echo "${INITIAL_POSE_ARRAY[@]:0:3}" | sed "s/ /,/g"`
INITIAL_ORIENTATION=`echo "${INITIAL_POSE_ARRAY[@]:3:7}" | sed "s/ /,/g"`

OUTPUT_PATH=$OUTPUT_ROOT_PATH"$LOG_POSTFIX"/"$OBJECT_NAME"/
echo $OUTPUT_PATH
mkdir -p $OUTPUT_PATH
mkdir -p $OUTPUT_PATH/segmentation
mkdir -p $OUTPUT_PATH/segmentation_refined
rm -f $OUTPUT_PATH/*.txt
rm -f $OUTPUT_PATH/segmentation/*.png
rm -f $OUTPUT_PATH/segmentation_refined/*.png

# sudo cpupower frequency-set --governor performance
$EXECUTABLE --from $CONFIG_ROOT_PATH/config_fast_ycb.cfg\
            --camera_dataset::fx $FX\
            --camera_dataset::fy $FY\
            --camera_dataset::cx $CX\
            --camera_dataset::cy $CY\
            --camera_dataset::path $OBJECT_ROOT_PATH\
            --initial_condition::pose::x "$INITIAL_POSITION"\
            --initial_condition::pose::axis_angle "$INITIAL_ORIENTATION"\
            --kinematic_model::pose::sigma_angular "$SIGMA_ANG_VEL"\
            --log::path $OUTPUT_PATH\
            --log::enable_segmentation false\
            --measurement_model::pose::cov_q "$P_COV_Q"\
            --measurement_model::use_pose $USE_POSE_MEASUREMENT\
            --measurement_model::use_pose_resync $USE_POSE_RESYNC\
            --measurement_model::use_velocity $USE_VEL_MEASUREMENT\
            --model::name $MODEL_NAME\
            --optical_flow_dataset::path $OBJECT_ROOT_PATH\
            --optical_flow_dataset::set $OF_SET/\
            --outlier_rejection::enable $USE_OUTREJ\
            --pose_dataset::path $OBJECT_ROOT_PATH/$POSE_SET/poses.txt\
            --segmentation_dataset::flow_aided $USE_FLOW_AIDED\
            --segmentation_dataset::path $OBJECT_ROOT_PATH\
            --segmentation_dataset::set $MASK_SET
# sudo cpupower frequency-set --governor powersave

# Convert to YCB-V reference frame
bash ./test/post_process_results.sh $MODEL_NAME $OUTPUT_PATH

if [ -f roft_env/bin/activate ]; then
   deactivate
fi
