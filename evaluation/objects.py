#===============================================================================
#
# Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT)
#
# This software may be modified and distributed under the terms of the
# GPL-2+ license. See the accompanying LICENSE file for details.
#
#===============================================================================

class Objects():

    def __init__(self):
        """Constructor"""

        self.objects = {}

        self.objects['models'] =\
        [
            '003_cracker_box',
            '004_sugar_box',
            '005_tomato_soup_can',
            '006_mustard_bottle',
            '009_gelatin_box',
            '010_potted_meat_can'
        ]

        self.objects['ycbv_synthetic'] =\
        [
            '003_cracker_box',
            '004_sugar_box',
            '005_tomato_soup_can',
            '006_mustard_bottle',
            '009_gelatin_box',
            '010_potted_meat_can',
            'ALL'
        ]

        self.objects['ycbv_real'] =\
        [
            '003_cracker_box',
            '006_mustard_bottle'
        ]

        self.objects['ho3d'] =\
        [
            '003_cracker_box',
            '004_sugar_box',
            '006_mustard_bottle',
            '010_potted_meat_can',
            'ALL'
        ]
