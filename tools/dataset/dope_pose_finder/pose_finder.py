#===============================================================================
#
# Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT)
#
# This software may be modified and distributed under the terms of the
# GPL-2+ license. See the accompanying LICENSE file for details.
#
#===============================================================================

import sys

# Open file
file_path = str(sys.argv[1])
fps = int(sys.argv[2])
fps_steps = (1.0 / fps) / (1 / 30.0)
f = open(file_path, 'r')

# Read all lines
lines = f.readlines()
f.close()

# Find the firstp pose different from the invalid pose
invalid_pose = ('0.0 ' * 7)[:-1]

for i, line in enumerate(lines):
    line = line.rstrip()
    if (line != invalid_pose) and (i % fps_steps == 0):
        # The starting frame should be i + 6 if i is not 0
        if i != 0:
            i += 6
        print(str(i) + ' ' + line)
        exit(0)
