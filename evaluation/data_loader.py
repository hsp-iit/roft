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
import os
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


    def load_gt_ycbv_synthetic(self):
        """Load gt data using our format for dataset ycbv synthetic."""

        self.data['ycbv_synthetic'] = {}

        dataset_location_file = open('./config/ycbv_synthetic_location', 'r')
        dataset_location = dataset_location_file.read()
        dataset_location_file.close()

        for object_name in self.objects['ycbv_synthetic']:

            if object_name == 'ALL':
                continue

            file_path = os.path.join(dataset_location, 'object_motion', object_name, 'gt', 'poses_ycb.txt')
            self.data['ycbv_synthetic'][object_name] = [self.load_generic(file_path)]


    def load_gt_ho3d(self):
        """Load gt data using our format for dataset ho3d."""

        self.data['ho3d'] = {}

        dataset_location_file = open('./config/ho3d_location', 'r')
        dataset_location = dataset_location_file.read()
        dataset_location_file.close()

        video_ids =\
        {
            '003_cracker_box' : ['0', '1', '2'],
            '004_sugar_box' : ['0', '1', '2', '3', '4'],
            '006_mustard_bottle' : ['0', '1', '2', '3'],
            '010_potted_meat_can' : ['100', '101', '102', '103', '104']
        }

        for object_name in self.objects['ho3d']:

            if object_name == 'ALL':
                continue

            self.data['ho3d'][object_name] = []
            for i, video_id in enumerate(video_ids[object_name]):
                file_path = os.path.join(dataset_location, object_name + '_' + video_id, 'gt', 'poses.txt')

                self.data['ho3d'][object_name].append(self.load_generic(file_path))


    def load_gt(self):
        """Load gt data using our format."""

        self.log('load_gt', 'loading ground truth data', starter = True)

        self.load_gt_ycbv_synthetic()

        self.load_gt_ho3d()

        print('')


    def load_ours(self):
        """Load the data of our algorithm using our format."""

        # Load useful information from the configuration
        config = self.algorithm['config']
        dataset_name = config['dataset']
        excluded_objects = config['excluded_objects']

        contents_map =\
        {
            'execution_times' : 'times',
            'pose_estimate_ycb' : 'pose',
            'velocity_estimate' : 'velocity',
            'pose_measurements' : 'pose_meas',
            'velocity_measurements' : 'vel_meas'
        }

        variant = 'full_' + config['masks_train_set'] + '_' + config['nvof_set']
        if config['gt_masks']:
            variant += '_gt_mask'
        if config['gt_pose']:
            variant += '_gt_pose'

        path = os.path.join(self.paths['ours'], dataset_name, variant)
        self.log('load_ours', 'loading data from ' + path, starter = True)

        # Video ids used in se3-tracknet
        video_ids = {}
        video_ids['ycbv_synthetic'] =\
        {
            '003_cracker_box' : [''],
            '004_sugar_box' : [''],
            '005_tomato_soup_can' : [''],
            '006_mustard_bottle' : [''],
            '009_gelatin_box' : [''],
            '010_potted_meat_can' : ['']
        }
        video_ids['ho3d'] =\
        {
            '003_cracker_box' : ['_0', '_1', '_2'],
            '004_sugar_box' : ['_0', '_1', '_2', '_3', '_4'],
            '006_mustard_bottle' : ['_0', '_1', '_2', '_3'],
            '010_potted_meat_can' : ['_100', '_101', '_102', '_103', '_104']
        }

        self.data = {}
        for object_name in self.objects[dataset_name]:

            if object_name == 'ALL':
                continue

            if object_name in excluded_objects:
                continue

            object_data = []

            for i, video_id in enumerate(video_ids[dataset_name][object_name]):

                object_path = path + '/' + object_name + video_id + '/'

                object_video_data = {}

                # self.log('load_ours', 'processing object ' + object_name)

                for content in contents_map:
                    d = self.load_generic(object_path + content + '.txt')
                    # This channel contains also a heading 6-sized vector of object velocities
                    if content == 'pose_estimate_ycb':
                        d = d[:, 6 : ]

                    object_video_data[contents_map[content]] = d

                object_data.append(object_video_data)

            self.data[object_name] = object_data

        print('')


    def load_se3_tracknet(self):
        """Load the data using our format for se3-TrackNet."""

        config = self.algorithm['config']

        contents_map = { 'pred' : 'pose' }
        dataset_map = { 'ycbv_synthetic' : 'Synthetic', 'ho3d' : 'HO3D' }

        dataset_name = config['dataset']
        se3_dataset_name = dataset_map[config['dataset']]

        # For dope, poses used durinig re-initialization are available
        if config['reinit_from'] == 'dope':
            contents_map['reinit_dope'] = 'pose_measurements'

        # Video ids used in se3-tracknet
        video_ids = {}
        video_ids['ycbv_synthetic'] =\
        {
            '003_cracker_box' : ['0001'],
            '004_sugar_box' : ['0002'],
            '005_tomato_soup_can' : ['0003'],
            '006_mustard_bottle' : ['0004'],
            '009_gelatin_box' : ['0005'],
            '010_potted_meat_can' : ['0006']
        }
        video_ids['ho3d'] =\
        {
            '003_cracker_box' : ['0001', '0002', '0003'],
            '004_sugar_box' : ['0004', '0005', '0006', '0007', '0008'],
            '006_mustard_bottle' : ['0009', '0010', '0011', '0012'],
            '010_potted_meat_can' : ['0013', '0014', '0015', '0016', '0017']
        }

        # Compose path to files according to the configuration
        config_string = se3_dataset_name + '_init_'
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
        for object_name in self.objects[dataset_name]:

            if object_name == 'ALL':
                continue

            object_data = []

            for i, video_id in enumerate(video_ids[dataset_name][object_name]):
                object_path = path + '/' + object_name + '/' + video_id + '/'

                object_video_data = {}
                # self.log('load_ours', 'processing object ' + object_name)

                for content in contents_map:
                    d = self.load_generic(object_path + content + '.txt')

                    # Repeat dope poses using sample-and-hold approach
                    if content == 'reinit_dope':
                        tmp = copy.deepcopy(d)
                        d = []
                        for j in range(tmp.shape[0]):
                            for k in range(int((1.0 / float(config['reinit_fps']) / (1.0 / 30.0)))):
                                d.append(tmp[j, 2 :])
                                if j == (tmp.shape[0] - 1):
                                    break

                    d = numpy.array(d)

                    object_video_data[contents_map[content]] = d

                object_data.append(object_video_data)

            self.data[object_name] = object_data

        print('')


    def load_poserbpf(self):
        """Load the data using our format for PoseRBPF."""

        config = self.algorithm['config']

        contents_map = { 'Pose' : 'pose', 'Index' : 'indexes' }
        dataset_map = { 'ycbv_synthetic' : 'synthetic' , 'ho3d' : 'HO3D'}

        dataset_name = config['dataset']
        poserbpf_dataset_name = dataset_map[config['dataset']]

        # For dope, poses used durinig re-initialization are available
        if config['reinit_from'] == 'dope':
            contents_map['reinit_dope'] = 'pose_measurements'

        # Video ids used in poserbpf
        video_ids = {}
        video_ids['ycbv_synthetic'] =\
        {
            '003_cracker_box' : ['seq_10'],
            '004_sugar_box' : ['seq_10'],
            '005_tomato_soup_can' : ['seq_10'],
            '006_mustard_bottle' : ['seq_10'],
            '009_gelatin_box' : ['seq_10'],
            '010_potted_meat_can' : ['seq_10']
        }
        video_ids['ho3d'] =\
        {
            '003_cracker_box' : ['seq_20', 'seq_21', 'seq_22'],
            '004_sugar_box' : ['seq_20', 'seq_21', 'seq_22', 'seq_23', 'seq_24'],
            '006_mustard_bottle' : ['seq_20', 'seq_21', 'seq_22', 'seq_23'],
            '010_potted_meat_can' : ['seq_20', 'seq_21', 'seq_22', 'seq_23', 'seq_24']
        }

        # Compose path to files according to the configuration
        config_string = str(config['particles']) + '/' + poserbpf_dataset_name + '_' + str(config['fps']) + 'fps_reinit_'

        if config['reinit']:
            config_string += config['reinit_from']
        else:
            config_string += 'None'

        if config['init_from']:
            config_string += config['init_from']

        path = self.paths['poserbpf'] + config_string
        self.log('load_poserbpf', 'loading data from ' + path, starter = True)

        # Load data for each object
        for object_name in self.objects[dataset_name]:

            if object_name == 'ALL':
                continue

            object_data = []

            for i, video_id in enumerate(video_ids[dataset_name][object_name]):

                object_name_in_path = object_name
                if dataset_name == 'ho3d':
                    object_name_in_path += '_' + video_id.split('_')[1]

                object_path = path + '/' + object_name_in_path + '/' + video_id + '/'

                object_video_data = {}
                # self.log('load_poserbpf', 'processing object ' + object_name)

                for content in contents_map:

                    video_id_cut = ''.join(video_id.split('_'))
                    if content == 'Index':
                        content_name = 'Pose_' + object_name + '_' + video_id_cut
                    elif content == 'Pose':
                        content_name = 'Pose_' + object_name + '_' + video_id_cut + '_clean'
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

                    object_video_data[contents_map[content]] = d

                object_data.append(object_video_data)

            self.data[object_name] = object_data

        print('')
