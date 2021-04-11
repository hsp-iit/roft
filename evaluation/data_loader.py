#===============================================================================
#
# Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT)
#
# This software may be modified and distributed under the terms of the
# GPL-2+ license. See the accompanying LICENSE file for details.
#
#===============================================================================

import copy
import numpy
from objects import Objects


class DataLoader():

    def __init__(self, algorithm):
        """Constructor."""

        self.algorithm = algorithm
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

        self.load_methods[self.algorithm['name']]()

        return self.data


    def load_generic(self, file_path):
        """Load from file."""

        data = []

        with open(file_path, newline = '') as csv_data:
            for row in csv_data:
                data.append([float(num_string.rstrip()) for num_string in row.rstrip().split(sep = ' ') if num_string != ''])
        return numpy.array(data)


    def load_poserbpf_indexes(self, file_path):
        """Load sequence of PoseRBPF indexes."""

        data = []

        with open(file_path, newline = '') as csv_data:
            for row in csv_data:
                data.append(int(row.split(' ')[1].split('/')[1]) - 1)

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

        print('')


    def load_ours(self):
        """Load the data of our algorithm using our format."""

        contents_map =\
        {
            'execution_times' : 'times',
            'pose_estimate_ycb' : 'pose',
            'velocity_estimate' : 'velocity',
            'pose_measurements' : 'pose_meas',
            'velocity_measurements' : 'vel_meas'
        }

        variant = self.algorithm['config']['variant']
        path = self.paths['ours'] + variant
        self.log('load_ours', 'loading data from ' + path, starter = True)

        self.data = {}
        for object_name in self.objects:
            object_path = path + '/' + object_name + '/'

            self.data[object_name] = {}
            self.log('load_ours', 'processing object ' + object_name)

            for content in contents_map:
                d = self.load_generic(object_path + content + '.txt')
                # This channel contains also a heading 6-sized vector of object velocities
                if content == 'pose_estimate_ycb':
                    d = d[:, 6 : ]
                self.data[object_name][contents_map[content]] = d

        print('')


    def load_se3_tracknet(self):
        """Load the data using our format for se3-TrackNet."""

        config = self.algorithm['config']

        contents_map = { 'pred' : 'pose' }
        # For dope, poses used durinig re-initialization are available
        if config['reinit_from'] == 'dope':
            contents_map['reinit_dope'] = 'pose_measurements'

        # Video ids used in se3-tracknet dataset
        video_ids =\
        {
            '003_cracker_box' : '0001',
            '004_sugar_box' : '0002',
            '005_tomato_soup_can' : '0003',
            '006_mustard_bottle' : '0004',
            '009_gelatin_box' : '0005',
            '010_potted_meat_can' : '0006'
        }

        # Compose path to files according to the configuration
        config_string = 'Synthetic_init_'
        if config['reinit']:
            config_string += config['reinit_from'] + '_'

            if config['reinit_from'] == 'gt':
                config_string += '0_'

            config_string += str(config['reinit_fps']) + '_fps'
        else:
            config_string += 'gt_reinit_None'

        path = self.paths['se3tracknet'] + config_string
        self.log('load_se3_tracknet', 'loading data from ' + path, starter = True)

        # Load data for each object
        for object_name in self.objects:
            object_path = path + '/' + object_name + '/' + video_ids[object_name] + '/'

            self.data[object_name] = {}
            self.log('load_ours', 'processing object ' + object_name)

            for content in contents_map:
                d = self.load_generic(object_path + content + '.txt')

                # Repeat dope poses using sample-and-hold approach
                if content == 'reinit_dope':
                    tmp = copy.deepcopy(d)
                    d = []
                    for i in range(tmp.shape[0]):
                        for j in range(int((1.0 / float(config['reinit_fps']) / (1.0 / 30.0)))):
                            d.append(tmp[i, 2 :])
                            if i == (tmp.shape[0] - 1):
                                break

                    d = numpy.array(d)

                self.data[object_name][contents_map[content]] = d

        print('')


    def load_poserbpf(self):
        """Load the data using our format for PoseRBPF."""

        config = self.algorithm['config']

        contents_map = { 'Pose' : 'pose', 'Index' : 'indexes' }
        # For dope, poses used durinig re-initialization are available
        if config['reinit_from'] == 'dope':
            contents_map['reinit_dope'] = 'pose_measurements'

        # Compose path to files according to the configuration
        config_string = str(config['particles']) + '_particles' + '/' + 'synthetic_' + str(config['fps']) + 'fps_reinit_'

        if config['reinit']:
            config_string += config['reinit_from']
        else:
            config_string += 'None'

        path = self.paths['poserbpf'] + config_string
        self.log('load_poserbpf', 'loading data from ' + path, starter = True)

        # Load data for each object
        for object_name in self.objects:
            object_path = path + '/' + object_name + '/seq_10/'

            self.data[object_name] = {}
            self.log('load_poserbpf', 'processing object ' + object_name)

            for content in contents_map:

                if content == 'Index':
                    content_name = 'Pose_' + object_name + '_seq10'
                elif content == 'Pose':
                    content_name = 'Pose_' + object_name + '_seq10_clean'
                else:
                    content_name = content

                file_path = object_path + content_name + '.txt'
                if content == 'Index':
                    d = self.load_poserbpf_indexes(file_path)
                else:
                    d = self.load_generic(file_path)

                # Repeat dope poses using sample-and-hold approach
                if content == 'reinit_dope':
                    tmp = copy.deepcopy(d)
                    d = []
                    for i in range(tmp.shape[0]):
                        for j in range(int((1.0 / float(config['fps']) / (1.0 / 30.0)))):
                            d.append(tmp[i, 2 :])
                            if i == (tmp.shape[0] - 1):
                                break

                    d = numpy.array(d)

                self.data[object_name][contents_map[content]] = d

        print('')
