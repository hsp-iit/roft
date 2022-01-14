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
import json
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
            'poserbpf' : self.load_poserbpf,
            'dope' : self.load_dope
        }

        root_path = './results/'
        self.paths =\
        {
            'ours' : root_path + 'ours/',
            'se3tracknet' : root_path + 'TrackNet_results/',
            'poserbpf' : root_path + 'PoseRBPF_results/'
        }

        self.dataset_paths =\
        {
            name : open(os.path.join('./config/', name + '_location'), 'r').readline() for name in ['ho3d', 'ycbv_synthetic']
        }
        self.dataset_paths['ycbv_synthetic'] = os.path.join(self.dataset_paths['ycbv_synthetic'], 'object_motion')
        self.dataset_paths['ycbv_real'] = self.dataset_paths['ycbv_synthetic']
        self.dataset_mesh_paths =\
        {
            'ycbv_synthetic' : './YCB_Video_Models/',
            'ycbv_real' : './YCB_Video_Models/',
            'ho3d' : './YCB_Video_Models/'
        }

        self.dataset_video_ids = {}
        self.dataset_video_ids['ycbv_synthetic'] =\
        {
            '003_cracker_box' : [''],
            '004_sugar_box' : [''],
            '005_tomato_soup_can' : [''],
            '006_mustard_bottle' : [''],
            '009_gelatin_box' : [''],
            '010_potted_meat_can' : ['']
        }
        self.dataset_video_ids['ycbv_real'] =\
        {
            '003_cracker_box' : ['_real'],
            '006_mustard_bottle' : ['_real'],
        }
        self.dataset_video_ids['ho3d'] =\
        {
            '003_cracker_box' : ['_0', '_1', '_2'],
            '004_sugar_box' : ['_0', '_1', '_2', '_3', '_4'],
            '006_mustard_bottle' : ['_0', '_1', '_2', '_3'],
            '010_potted_meat_can' : ['_100', '_101', '_102', '_103', '_104']
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


    def load_camera_intrinsics(self, file_path):
        """Load camera intriniscs from json."""

        f = open(file_path, 'r')
        data = json.load(f)
        f.close()

        # These parameters needs to be floats not strings
        for key in ['fx', 'fy', 'cx', 'cy']:
            data[key] = float(data[key])

        return data


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
        self.data['ycbv_synthetic_velocity'] = {}

        dataset_location_file = open('./config/ycbv_synthetic_location', 'r')
        dataset_location = dataset_location_file.read()
        dataset_location_file.close()

        for object_name in self.objects['ycbv_synthetic']:

            if object_name == 'ALL':
                continue

            file_path = os.path.join(dataset_location, 'object_motion', object_name, 'gt', 'poses_ycb.txt')
            self.data['ycbv_synthetic'][object_name] = [self.load_generic(file_path)]

            # Load ground truth velocities
            vel_file_path = os.path.join(dataset_location, 'object_motion', object_name, 'gt', 'velocities.txt')
            self.data['ycbv_synthetic_velocity'][object_name] = [self.load_generic(vel_file_path)]


    def load_gt_ho3d(self):
        """Load gt data using our format for dataset ho3d."""

        self.data['ho3d'] = {}

        dataset_location_file = open('./config/ho3d_location', 'r')
        dataset_location = dataset_location_file.read()
        dataset_location_file.close()

        video_ids = self.dataset_video_ids['ho3d']

        for object_name in self.objects['ho3d']:

            if object_name == 'ALL':
                continue

            self.data['ho3d'][object_name] = []
            for i, video_id in enumerate(video_ids[object_name]):
                file_path = os.path.join(dataset_location, object_name + video_id, 'gt', 'poses.txt')

                self.data['ho3d'][object_name].append(self.load_generic(file_path))


    def load_gt(self):
        """Load gt data using our format."""

        self.log('load_gt', 'loading ground truth data', starter = True)

        self.load_gt_ycbv_synthetic()

        self.load_gt_ho3d()

        print('')


    def load_ours(self):
        """Load the data of our algorithm using our format."""

        # Load datasets video ids
        video_ids = self.dataset_video_ids

        # Load useful information from the configuration
        config = self.algorithm['config']
        dataset_name = config['dataset']
        excluded_objects = config['excluded_objects']

        contents_map =\
        {
            'execution_times' : 'time',
            'pose_estimate_ycb' : 'pose',
            'velocity_estimate' : 'velocity',
            'pose_measurements' : 'pose_meas',
            'velocity_measurements' : 'vel_meas'
        }

        variant = 'full_mask_' + config['masks_set'] + '_of_' + config['of_set'] + '_pose_' + config['pose_set']
        for variant_name in ['no_outrej', 'no_posesync', 'no_flowaid', 'no_velocity', 'no_pose']:
            if (variant_name in config) and config[variant_name]:
                variant += ('_' + variant_name)

        path = os.path.join(self.paths['ours'], dataset_name, variant)
        self.log('load_ours', 'loading data from ' + path, starter = True)

        self.data = {}
        for object_name in self.objects[dataset_name]:

            if object_name == 'ALL':
                continue

            if object_name in excluded_objects:
                continue

            object_data = []

            for i, video_id in enumerate(video_ids[dataset_name][object_name]):

                object_path = os.path.join(path, object_name + video_id)
                segmentation_path = os.path.join(path, object_name + video_id, 'segmentation')
                rgb_path = os.path.join(self.dataset_paths[dataset_name], object_name + video_id, 'rgb')
                cam_path = os.path.join(self.dataset_paths[dataset_name], object_name + video_id, 'cam_K.json')
                mesh_path = os.path.join(self.dataset_mesh_paths[dataset_name], object_name, 'textured.obj')

                object_video_data = {}

                # self.log('load_ours', 'processing object ' + object_name)

                for content in contents_map:
                    d = self.load_generic(os.path.join(object_path, content + '.txt'))
                    # This channel contains also a heading 6-sized vector of object velocities
                    if content == 'pose_estimate_ycb':
                        d = d[:, 6 : ]

                    object_video_data[contents_map[content]] = d
                object_video_data['rgb_path'] = rgb_path
                object_video_data['cam_intrinsics'] = self.load_camera_intrinsics(cam_path)
                object_video_data['mesh_path'] = mesh_path
                object_video_data['segmentation_path'] = segmentation_path

                object_data.append(object_video_data)

            self.data[object_name] = object_data

        print('')


    def load_se3_tracknet(self):
        """Load the data using our format for se3-TrackNet."""

        config = self.algorithm['config']

        contents_map = { 'pred' : 'pose' }
        dataset_map = { 'ycbv_synthetic' : 'synthetic', 'ho3d' : 'ho3d', 'ycbv_real' : 'real'}

        dataset_name = config['dataset']
        se3_dataset_name = dataset_map[config['dataset']]

        # For dope, poses used durinig re-initialization are available
        if config['reinit_from'] == 'dope':
            contents_map['reinit_dope'] = 'pose_measurements'

        # Video ids used in se3-tracknet
        video_ids = {}
        video_ids['ycbv_real'] =\
        {
            '003_cracker_box' : ['0001'],
            '006_mustard_bottle' : ['0002']
        }
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
        config_string = se3_dataset_name + '_'

        config_string += 'init_'
        if config['init_from']:
            config_string += config['init_from']
        else:
            config_string += 'none'
        config_string += '_'

        config_string += 'reinit_'
        if config['reinit']:
            config_string += config['reinit_from']
        else:
            config_string += 'none'

        if config['reinit']:
            config_string += '_' + str(config['reinit_fps']) + '_fps'

        path = self.paths['se3tracknet'] + config_string
        self.log('load_se3_tracknet', 'loading data from ' + path, starter = True)

        # Load data for each object
        for object_name in self.objects[dataset_name]:

            if object_name == 'ALL':
                continue

            object_data = []

            for i, video_id in enumerate(video_ids[dataset_name][object_name]):
                object_path = os.path.join(path, object_name, video_id)

                # video ids for rgb and cam paths are those of the datasets
                dataset_video_id = self.dataset_video_ids[dataset_name][object_name][i]
                rgb_path = os.path.join(self.dataset_paths[dataset_name], object_name + dataset_video_id, 'rgb')
                cam_path = os.path.join(self.dataset_paths[dataset_name], object_name + dataset_video_id, 'cam_K.json')
                mesh_path = os.path.join(self.dataset_mesh_paths[dataset_name], object_name, 'textured.obj')

                object_video_data = {}
                # self.log('load_ours', 'processing object ' + object_name)

                for content in contents_map:
                    d = self.load_generic(os.path.join(object_path,content + '.txt'))

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
                object_video_data['rgb_path'] = rgb_path
                object_video_data['cam_intrinsics'] = self.load_camera_intrinsics(cam_path)
                object_video_data['mesh_path'] = mesh_path

                object_data.append(object_video_data)

            self.data[object_name] = object_data

        print('')


    def load_poserbpf(self):
        """Load the data using our format for PoseRBPF."""

        config = self.algorithm['config']

        contents_map = { 'Pose' : 'pose', 'Index' : 'indexes' }
        dataset_map = { 'ycbv_synthetic' : 'synthetic' , 'ho3d' : 'ho3d', 'ycbv_real' : 'real'}

        dataset_name = config['dataset']
        poserbpf_dataset_name = dataset_map[config['dataset']]

        # For dope, poses used durinig re-initialization are available
        if config['reinit_from'] == 'dope':
            contents_map['reinit_dope'] = 'pose_measurements'

        # Video ids used in poserbpf
        video_ids = {}
        video_ids['ycbv_real'] =\
        {
            '003_cracker_box' : ['seq_30'],
            '006_mustard_bottle' : ['seq_30']
        }
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
        config_string = str(config['particles']) + '_particles/' + poserbpf_dataset_name + '_'

        config_string += 'init_'
        if config['init_from']:
            config_string += config['init_from']
        else:
            config_string += 'none'
        config_string += '_'

        config_string += 'reinit_'
        if config['reinit']:
            config_string += config['reinit_from']
        else:
            config_string += 'none'

        config_string += '_' + str(config['fps']) + '_fps'

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

                object_path = os.path.join(path, object_name_in_path, video_id)

                # video ids for rgb and cam paths are those of the dataset not of the algorithm
                dataset_video_id = self.dataset_video_ids[dataset_name][object_name][i]
                rgb_path = os.path.join(self.dataset_paths[dataset_name], object_name + dataset_video_id, 'rgb')
                cam_path = os.path.join(self.dataset_paths[dataset_name], object_name + dataset_video_id, 'cam_K.json')
                mesh_path = os.path.join(self.dataset_mesh_paths[dataset_name], object_name, 'textured.obj')

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

                    file_path = os.path.join(object_path, content_name + '.txt')
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
                object_video_data['rgb_path'] = rgb_path
                object_video_data['cam_intrinsics'] = self.load_camera_intrinsics(cam_path)
                object_video_data['mesh_path'] = mesh_path

                object_data.append(object_video_data)

            self.data[object_name] = object_data

        print('')


    def load_dope(self):
        """Load the data using our format for PoseRBPF."""

        config = self.algorithm['config']

        dataset_name = config['dataset']

        # Load datasets video ids
        video_ids = self.dataset_video_ids

        # Load data for each object
        for object_name in self.objects[dataset_name]:

            if object_name == 'ALL':
                continue

            object_data = []

            for i, video_id in enumerate(video_ids[dataset_name][object_name]):

                object_video_data = {}

                file_path = os.path.join(self.dataset_paths[dataset_name], object_name + video_id, 'dope', 'poses_ycb.txt')
                rgb_path = os.path.join(self.dataset_paths[dataset_name], object_name + video_id, 'rgb')
                cam_path = os.path.join(self.dataset_paths[dataset_name], object_name + video_id, 'cam_K.json')
                mesh_path = os.path.join(self.dataset_mesh_paths[dataset_name], object_name, 'textured.obj')

                self.log('load_dope', 'loading data from ' + file_path, starter = True)

                d = self.load_generic(file_path)
                data = []
                indexes = []

                if config['simulate_inference']:
                    # In this case we simulate inference at 5 fps with 200 ms delay for each frame (i.e a 30 fps stream)
                    # If a pose is missing because the detection was missing (coded with all zeros) the most recent is taken instead
                    # This corresponds to a practical robotic scenario where we run DOPE at 5 fps and we need an answer for each frame at 30 fps

                    fps = 5
                    skip_steps = int((1.0 / fps) / (1 / 30.0))

                    # Apply delay
                    data = numpy.pad(d, ((skip_steps, 0), (0, 0)), 'edge')
                    data = data[:d.shape[0], :]

                    # Sample at 5 fps and hold
                    data = data[::skip_steps, :]
                    data = numpy.repeat(data, skip_steps, axis = 0)
                    data = data[: d.shape[0], :]

                    # DOPE frames might be missing from the very beginning
                    i_0 = 0
                    for i in range(data.shape[0]):
                        if (data[i, 0] != 0.0) and (data[i, 1] != 0.0) and (data[i, 2] != 0.0):
                            i_0 = i
                            break
                    indexes = numpy.array(list(range(i_0, data.shape[0])))
                    data = data[i_0:, :]

                    # Substitute missing detections with the latest available value
                    for i in range(1, data.shape[0]):
                        if (data[i, 0] == 0.0) and (data[i, 1] == 0.0) and (data[i, 2] == 0.0):
                            data[i, :] = data[i-1, :]
                else:
                    # In this case we take all the frames discarding those that correspond to missing detection
                    # A list of valid indexes is provides, such that the batch validation machinery can align
                    # the provided data with the ground truth
                    for i in range(d.shape[0]):
                        if (d[i, 0] == 0.0) and (d[i, 1] == 0.0) and (d[i, 2] == 0.0):
                            continue
                        data.append(d[i, :])
                        indexes.append(i)
                    data = numpy.array(data)
                    indexes = numpy.array(indexes)

                object_video_data['pose'] = data
                object_video_data['indexes'] = indexes
                object_video_data['rgb_path'] = rgb_path
                object_video_data['cam_intrinsics'] = self.load_camera_intrinsics(cam_path)
                object_video_data['mesh_path'] = mesh_path

                object_data.append(object_video_data)

            self.data[object_name] = object_data

        print('')
