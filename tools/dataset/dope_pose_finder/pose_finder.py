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
f = open(file_path, 'r')

# Read all lines
lines = f.readlines()
f.close()

# Find the firstp pose different from the invalid pose
invalid_pose = ('0.0 ' * 7)[:-1]

for line in lines:
    line = line.rstrip()
    if line != invalid_pose:
        print(line)
        exit(0)
