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
GT_MASK=$2
GT_OF=$3
GT_POSE=$4
NO_OUT_REJ=$5
NO_POSE_SYNC=$6
NO_FLOW_AID=$7
ONLY_POSE=$8
ONLY_VEL=$9

# paths and names
EXECUTABLE=?
YCBV_SYN_PATH=?
CONFIG_ROOT_PATH=./config/
OBJECT_ROOT_PATH=$YCBV_SYN_PATH/$OBJECT_NAME/
OUTPUT_ROOT_PATH=./results/ours/ycbv_synthetic/

# configuration matrix
MASK_SET="mrcnn"
OF_SET="nvof/"
POSE_SET="dope/"
USE_OUTREJ="true"
USE_POSE_RESYNC="true"
USE_FLOW_AIDED="true"
USE_POSE_MEASUREMENT="true"
USE_VEL_MEASUREMENT="true"
SIGMA_ANG_VEL="(1.0, 1.0, 1.0)"
P_COV_Q="(0.0001, 0.0001, 0.0001)"

LOG_POSTFIX="full"
if [ "$GT_MASK" == "true" ]; then
    MASK_SET="gt"
    LOG_POSTFIX=${LOG_POSTFIX}"_gt_mask"
fi
if [ "$GT_OF" == "true" ]; then
    OF_SET="gt/"
    LOG_POSTFIX=${LOG_POSTFIX}"_gt_of"
fi
if [ "$GT_POSE" == "true" ]; then
    POSE_SET="gt/"
    LOG_POSTFIX=${LOG_POSTFIX}"_gt_pose"
fi
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
if [ "$ONLY_POSE" == "true" ]; then
    LOG_POSTFIX=${LOG_POSTFIX}"_only_pose"
    USE_VEL_MEASUREMENT="false"
    USE_OUTREJ="false"
    USE_POSE_RESYNC="false"
    SIGMA_ANG_VEL="(0.01, 0.01, 0.01)"
    P_COV_Q="(0.01, 0.01, 0.01)"
fi
if [ "$ONLY_VEL" == "true" ]; then
    LOG_POSTFIX=${LOG_POSTFIX}"_only_vel"
    USE_POSE_MEASUREMENT="false"
    USE_OUTREJ="false"
    USE_POSE_RESYNC="false"
fi

INITIAL_POSE_STRING=`head -n 1 $OBJECT_ROOT_PATH/dope/poses.txt`
INITIAL_POSE_ARRAY=($INITIAL_POSE_STRING)
INITIAL_POSITION="${INITIAL_POSE_ARRAY[@]:0:3}"
INITIAL_ORIENTATION="${INITIAL_POSE_ARRAY[@]:3:7}"

# cd $YCBV_SYN_PATH/$OBJECT_NAME
OUTPUT_PATH=$OUTPUT_ROOT_PATH"$LOG_POSTFIX"/"$OBJECT_NAME"/
echo $OUTPUT_PATH
mkdir -p $OUTPUT_PATH
rm -f $OUTPUT_PATH/*.txt

$EXECUTABLE --from $CONFIG_ROOT_PATH/config_ycbv_syn.ini\
            --CAMERA_DATASET::path $OBJECT_ROOT_PATH\
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
            --OPTICAL_FLOW_DATASET::set $OF_SET\
            --POSE_DATASET::path $OBJECT_ROOT_PATH/$POSE_SET/poses.txt\
            --SEGMENTATION::flow_aided $USE_FLOW_AIDED\
            --SEGMENTATION_DATASET::path $OBJECT_ROOT_PATH\
            --SEGMENTATION_DATASET::set $MASK_SET
