#===============================================================================
#
# Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT)
#
# This software may be modified and distributed under the terms of the
# GPL-2+ license. See the accompanying LICENSE file for details.
#
#===============================================================================

import os
import sys

sequence_path = str(sys.argv[1])
fps = 30.0

num_lines = sum(1 for line in open(os.path.join(sequence_path, 'gt', 'poses.txt')))

f = open(os.path.join(sequence_path, 'data.txt'), 'w')

for i in range(num_lines):
    stamp_rgb = (1.0 / fps) * i
    stamp_depth = stamp_rgb
    camera_pose = '0.0 0.0 0.0 1.0 0.0 0.0 0.0'
    f.write(str(stamp_rgb) + ' ' + str(stamp_depth) + ' ' + camera_pose + '\n')

f.close()
