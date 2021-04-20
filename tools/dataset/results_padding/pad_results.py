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
object_name = str(sys.argv[2])
sequence_name = str(sys.argv[3])
video_name = object_name + '_'  + sequence_name

# For these sequences DOPE predictions are not available starting from the first frame
# Hence, we zeropad the results at the beginning until we receive the first valid DOPE prediction
# and only then we start the tracking process
padding_list =\
{
    '006_mustard_bottle_2' : { 'padding' : 72, 'target_size' : 880 }
}

if video_name in padding_list:
    f = open(file_path, 'r+')

    lines = f.readlines()
    if len(lines) == padding_list[video_name]['target_size']:
        # nothing to do, the results have been already padded
        f.close()
        exit(0)

    f.seek(0, 0)
    for i in range(padding_list[video_name]['padding']):
        f.write('0.0 ' * 13 + '\n')
    for line in lines:
        f.write(line)

    f.close()
