#===============================================================================
#
# Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT)
#
# This software may be modified and distributed under the terms of the
# GPL-2+ license. See the accompanying LICENSE file for details.
#
#===============================================================================

# arguments
OBJECT_NAME=$1
SEQ_NAME=$2
GT_MASK=$3
GT_OF=$4
GT_POSE=$5
NO_OUT_REJ=$6
NO_POSE_SYNC=$7
NO_FLOW_AID=$8
NO_POSE=$9
NO_VEL=$10
HO3D_MASK=${11}

# paths and names
EXECUTABLE=robmo-misc-object-tracker-of
HO3D_PATH='./dataset/ho3d/'
CONFIG_ROOT_PATH=./config/
OBJECT_ROOT_PATH=$HO3D_PATH/${OBJECT_NAME}_${SEQ_NAME}/
OUTPUT_ROOT_PATH=./results/ours/ho3d/

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
#CY=${CY::-1}

# configuration matrix
MASK_SET="mrcnn_ycbv_bop_pbr"
OF_SET="nvof_2_slow"
POSE_SET="dope"
POSE_POSTFIX=""
USE_OUTREJ="true"
USE_POSE_RESYNC="true"
USE_FLOW_AIDED="true"
USE_POSE_MEASUREMENT="true"
USE_VEL_MEASUREMENT="true"
SIGMA_ANG_VEL="(1.0, 1.0, 1.0)"
P_COV_Q="(0.0001, 0.0001, 0.0001)"

INITIAL_POSE_STRING=`python ./tools/dataset/dope_pose_finder/pose_finder.py $OBJECT_ROOT_PATH/dope/poses.txt 5` # 5 is the FPS used for DOPE
INITIAL_POSE_ARRAY=($INITIAL_POSE_STRING)
INITIAL_INDEX="${INITIAL_POSE_ARRAY[@]:0:1}"
INITIAL_POSITION="${INITIAL_POSE_ARRAY[@]:1:3}"
INITIAL_ORIENTATION="${INITIAL_POSE_ARRAY[@]:4}"

LOG_POSTFIX="full"
if [ "$GT_MASK" == "true" ]; then
    MASK_SET="gt"
fi
if [ "$HO3D_MASK" == "true" ]; then
    MASK_SET="mrcnn_ho3d"
fi
LOG_POSTFIX=${LOG_POSTFIX}"_mask_"${MASK_SET}

if [ "$GT_OF" == "true" ]; then
    OF_SET="gt"
fi
LOG_POSTFIX=${LOG_POSTFIX}"_of_"${OF_SET}

if [ "$GT_POSE" == "true" ]; then
    POSE_SET="gt"
    POSE_POSTIX="_nvdu"

    INITIAL_POSE_STRING=`head $OBJECT_ROOT_PATH/gt/poses_nvdu.txt -n 1`
    INITIAL_POSE_ARRAY=($INITIAL_POSE_STRING)
    INITIAL_INDEX="0"
    INITIAL_POSITION="${INITIAL_POSE_ARRAY[@]:0:3}"
    INITIAL_ORIENTATION="${INITIAL_POSE_ARRAY[@]:3}"
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
    SIGMA_ANG_VEL="(0.01, 0.01, 0.01)"
    P_COV_Q="(0.01, 0.01, 0.01)"
fi

if [ "$NO_POSE" == "true" ]; then
    LOG_POSTFIX=${LOG_POSTFIX}"_no_pose"
    USE_POSE_MEASUREMENT="false"
    USE_OUTREJ="false"
    USE_POSE_RESYNC="false"
fi

OUTPUT_PATH=$OUTPUT_ROOT_PATH"$LOG_POSTFIX"/"$OBJECT_NAME"_"$SEQ_NAME"/
mkdir -p $OUTPUT_PATH
rm -f $OUTPUT_PATH/*.txt

$EXECUTABLE --from $CONFIG_ROOT_PATH/config_ho3d.ini\
            --CAMERA::fx $FX\
            --CAMERA::fy $FY\
            --CAMERA::cx $CX\
            --CAMERA::cy $CY\
            --CAMERA_DATASET::path $OBJECT_ROOT_PATH\
            --CAMERA_DATASET::index_offset $INITIAL_INDEX\
            --INITIAL_CONDITION::p_x_0 "($INITIAL_POSITION)"\
            --INITIAL_CONDITION::p_axis_angle_0 "($INITIAL_ORIENTATION)"\
            --KINEMATIC_MODEL::sigma_ang_vel "$SIGMA_ANG_VEL"\
            --LOG::absolute_log_path $OUTPUT_PATH\
            --MEASUREMENT_MODEL::p_cov_q "$P_COV_Q"\
            --MEASUREMENT_MODEL::use_pose_measurement $USE_POSE_MEASUREMENT\
            --MEASUREMENT_MODEL::use_vel_measurement $USE_VEL_MEASUREMENT\
            --MEASUREMENT_MODEL::use_pose_resync $USE_POSE_RESYNC\
            --MODEL::name $OBJECT_NAME\
            --OPTICAL_FLOW_DATASET::path $OBJECT_ROOT_PATH\
            --OPTICAL_FLOW_DATASET::set $OF_SET/\
            --OPTICAL_FLOW_DATASET::index_offset $INITIAL_INDEX\
            --POSE_DATASET::path $OBJECT_ROOT_PATH/$POSE_SET/poses$POSE_POSTIX.txt\
            --POSE_DATASET::skip_rows $INITIAL_INDEX\
            --SEGMENTATION::flow_aided $USE_FLOW_AIDED\
            --SEGMENTATION_DATASET::path $OBJECT_ROOT_PATH\
            --SEGMENTATION_DATASET::set $MASK_SET\
            --SEGMENTATION_DATASET::index_offset $INITIAL_INDEX

# Convert to YCB-V reference frame
bash ./test/post_process_results_ho3d.sh $OBJECT_NAME $SEQ_NAME $OUTPUT_PATH
