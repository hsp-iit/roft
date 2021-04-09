#===============================================================================
#
# Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT)
#
# This software may be modified and distributed under the terms of the
# GPL-2+ license. See the accompanying LICENSE file for details.
#
#===============================================================================

import numpy
from objects import Objects


class DataLoader():

    def __init__(self, algorithm_name):
        """Constructor."""

        self.algorithm_name = algorithm_name
        self.load_methods =\
        {
            'gt' : self.load_gt,
            'ours' : self.load_ours,
            'se3tracknet' : self.load_se3_tracknet,
            'poserbpf' : self.load_poserbpf
        }

        root_path = './results/'
        self.paths =\
        {
            'gt' : root_path + 'gt/',
            'ours' : root_path + 'ours/',
            'se3tracknet' : root_path + 'TrackNet_results/',
            'poserbpf' : root_path + 'PoseRBPF_results/'
        }

        self.objects = Objects().objects

        self.data = {}


    def log(self, method, msg, starter = False):
        """Log function."""
        if starter:
            print('DataLoader::' + method + '()')
        print('    info: ' + msg)


    def load(self):
        """Load the data."""
        self.load_methods[self.algorithm_name]()

        return self.data


    def load_generic(self, file_path):

        data = []

        with open(file_path, newline = '') as csv_data:
            for row in csv_data:
                data.append([float(num_string.rstrip()) for num_string in row.rstrip().split(sep = ' ') if num_string != ''])
        return numpy.array(data)


    def load_gt(self):
        """Load gt data using our format."""

        self.log('load_gt', 'loading data from ' + self.paths['gt'], starter = True)

        for object_name in self.objects:
            object_path = self.paths['gt'] + '/' + object_name + '/'

            self.data[object_name] = {}
            self.log('load_gt', 'processing object ' + object_name)

            d = self.load_generic(object_path  + 'poses_ycb.txt')
            self.data[object_name] = d


    def load_ours(self):
        """Load the data using our format."""

        variants = {'full'}
        contents = {'execution_times', 'pose_estimate_ycb', 'velocity_estimate', 'pose_measurements', 'velocity_measurements'}

        self.data = {}

        for variant in variants:
            self.data[variant] = {}

            path = self.paths['ours'] + variant
            self.log('load_ours', 'loading data from ' + path, starter = True)

            for object_name in self.objects:
                object_path = self.paths['ours'] + variant + '/' + object_name + '/'

                self.data[variant][object_name] = {}
                self.log('load_ours', 'processing object ' + object_name)

                for content in contents:
                    d = self.load_generic(object_path + content + '.txt')
                    # This channel contains also a heading 6-sized vector of object velocities
                    if content == 'pose_estimate_ycb':
                        d = d[:, 6 : ]
                    self.data[variant][object_name][content] = d


    def load_se3_tracknet(self):
        """Load the data using our format for se3-TrackNet."""
        pass


    def load_poserbpf(self):
        """Load the data using our format for PoseRBPF."""
        pass
