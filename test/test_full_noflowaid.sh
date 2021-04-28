#===============================================================================
#
# Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT)
#
# This software may be modified and distributed under the terms of the
# GPL-2+ license. See the accompanying LICENSE file for details.
#
#===============================================================================

OBJECT_NAME=$1
GT_MASK="false"
GT_OF="false"
GT_POSE="false"
NO_OUT_REJ="false"
NO_POSE_SYNC="false"
NO_FLOW_AID="true"
NO_POSE="false"
NO_VEL="false"

bash test/test.sh $OBJECT_NAME $GT_MASK $GT_OF $GT_POSE $NO_OUT_REJ $NO_POSE_SYNC $NO_FLOW_AID $NO_POSE $NO_VEL
