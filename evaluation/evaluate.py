#===============================================================================
#
# Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT)
#
# This software may be modified and distributed under the terms of the
# GPL-2+ license. See the accompanying LICENSE file for details.
#
#===============================================================================

import argparse
import copy
import glob
import numpy
import os
import sys
from data_loader import DataLoader
from experiments import Experiments
from objects import Objects
from metrics import Metric
from results_renderer import ResultsLaTeXRenderer
from results_renderer import ResultsLaTeXSummaryRenderer
from results_renderer import ResultsMarkdownRenderer
from results_renderer import ResultsMatplotlibRenderer
from results_renderer import ResultsSegmentationVideoRenderer
from results_renderer import ResultsThumbnailRenderer
from results_renderer import ResultsVideoRenderer

class Evaluator():

    def __init__(self, metric_name, experiment_name, output_head, output_path, subset_from, disable_ho3d_padding, expand_if_missing):
        """Constructor.

           metric_name = the desired metric
           experiment_name = one of the experiments defined in experiments.Experiments
           output_head = markdown or latex
           output_path = where to save results
           subset_from = name of the algorithm whose ground truth indexes should be used for the evaluation
           disable_ho3d_padding = whether to handle that DOPE predictions for HO-3D in sequence 006_mustard_bottle_2 are missing starting from the first frame
           expand_if_missing = whether to sample and hold algorithm output if provided frames are less than ground truth frames
        """

        self.data = {}
        self.results = {}
        self.subset_from = subset_from
        self.expand_if_missing = expand_if_missing
        self.metric_name = metric_name
        self.output_head = output_head
        self.output_path = output_path

        # Load object names
        self.objects = Objects().objects
        print('List of objects: ' + str(self.objects))
        print('')

        # Load experiments
        self.experiments = Experiments()
        print('Available experiments:')
        print('')
        print(self.experiments)

        # Load ground truth data
        experiment = self.experiments(experiment_name)
        dataset_name = experiment[0]['config']['dataset']
        self.data['gt'] = DataLoader({'name' : 'gt_' + dataset_name}).load()

        # Initialize metrics
        self.metrics = {}
        self.metric_names = []
        if metric_name == 'add':
            self.metric_names = ['add']
        elif metric_name == 'rmse':
            self.metric_names = ['rmse_cartesian_' + coord for coord in ['x', 'y', 'z']]
            self.metric_names.append('rmse_angular')
        elif metric_name == 'rmse_3d':
            self.metric_names = ['rmse_cartesian_3d']
        elif metric_name == 'rmse_angular':
            self.metric_names = ['rmse_angular']
        elif metric_name == 'rmse_velocity':
            self.metric_names = ['rmse_linear_velocity', 'rmse_angular_velocity']
        elif metric_name == 'max_velocity':
            self.metric_names = ['max_linear_velocity', 'max_angular_velocity']
        elif metric_name == 'mix':
            self.metric_names = ['add', 'rmse_cartesian_3d', 'rmse_angular']
        elif metric_name == 'time':
            self.metric_names = ['time', 'excess_33_ms']

        self.metrics = { name : Metric(name) for name in self.metric_names }

        # Take into account experiments with missing DOPE predictions at the beginning of the scene
        self.ho3d_padding_list = {}
        if not disable_ho3d_padding:
            self.ho3d_padding_list=\
            {
                '006_mustard_bottle' : { 2 : { 'padding' : 72, 'target_size' : 880 } }
            }

        # Process experiments
        if experiment_name is not None:
            self.process_experiment(experiment_name)
        else:
            # Process all the experiments if the user does not specify a specific experiment
            for experiment in self.experiments.experiments:
                self.process_experiment(experiment)

        # Render results
        extension, renders = self.render()

        # Save results
        self.save(renders, extension)


    def process_experiment(self, experiment_name):
        """Process a single experiment."""

        if self.output_head in ['video', 'video-segmentation', 'thumbnail']:
            self.prepare_experiment_for_video(experiment_name)
        else:
            self.evaluate_experiment(experiment_name)


    def prepare_experiment_for_video(self, experiment_name):
        """Process a single experiment for video production."""

        print('Processing experiment: ' + experiment_name)
        print('')

        self.data[experiment_name] = {}
        exp_data = self.data[experiment_name]

        self.results[experiment_name] = {}
        exp_results = self.results[experiment_name]

        # Load experiment
        experiment = self.experiments(experiment_name)

        # Load data for all the algorithms
        for algorithm in experiment:
            loader = DataLoader(algorithm)
            exp_data[algorithm['label']] = loader.load()

        # Process data for each algorithm
        for algorithm in experiment:
            print('Preparing data for  algorithm with label ' + algorithm['label'] + '.')
            exp_results[algorithm['label']] = {}

            dataset_name = algorithm['config']['dataset']

            excluded_objects = algorithm['config']['excluded_objects']

            # For each object
            for object_name in self.objects[dataset_name]:

                print('    processing object ' + object_name)
                exp_results[algorithm['label']][object_name] = []

                if (object_name in excluded_objects) or (object_name == 'ALL'):
                    continue

                # For each sequence belonging to the object
                for i, sequence_data in enumerate(exp_data[algorithm['label']][object_name]):

                    output_sequence_data = {}

                    seq_gt = None
                    if dataset_name in self.data['gt']:
                        seq_gt = self.data['gt'][dataset_name][object_name][i]
                    seq_pose_available = sequence_data['pose']
                    seq_pose = None

                    expand_data = False
                    sequence_length = None

                    if seq_gt is not None:
                        sequence_length = seq_gt.shape[0]
                    # Ground truth data is not available for these datasets
                    elif dataset_name in ['ycbv_real']:
                        sequence_length = len(glob.glob(os.path.join(sequence_data['rgb_path'], '*')))

                    # Check if the length of sequence and that of the pose array is the same
                    if sequence_length != seq_pose_available.shape[0]:
                        # expand available poses with invalid poses such that the renderer can skip them
                        # if not check if a list of indexes is provided
                        if not 'indexes' in sequence_data:
                            print('Algorithm ' + algorithm['name'] + ' (label ' + algorithm['label'] + ')' +\
                                  ' provides ' + str(seq_pose_all.shape) + ' matrix as pose while the actual sequence has shape ' + \
                                  str(sequence_length) + '. However, a list of indexes for the poses has not been provided.' + \
                                  ' Cannot continue.')

                        seq_pose = []
                        indexes = sequence_data['indexes']
                        for j in range(sequence_length):
                            if j in indexes:
                                seq_pose.append(seq_pose_available[list(indexes).index(j), :])
                            else:
                                seq_pose.append([0] * 7)

                        seq_pose = numpy.array(seq_pose)
                    else:
                        seq_pose = seq_pose_available

                    # copy some information from sequence data
                    output_sequence_data['cam_intrinsics'] = sequence_data['cam_intrinsics']
                    output_sequence_data['mesh_path'] = sequence_data['mesh_path']
                    output_sequence_data['rgb_path'] = sequence_data['rgb_path']
                    if 'segmentation_path' in sequence_data:
                        output_sequence_data['segmentation_path'] = sequence_data['segmentation_path']

                    # compose output path and assing post-processed poses
                    output_sequence_data['output_path_rgb'] = os.path.join(self.output_path, experiment_name, algorithm['label'], object_name, 'sequence_' + str(i))
                    output_sequence_data['output_path_thumbnail'] = os.path.join(self.output_path, experiment_name, algorithm['label'], object_name, 'sequence_' + str(i), 'thumb')
                    output_sequence_data['output_path_segmentation'] = os.path.join(self.output_path, experiment_name, algorithm['label'], object_name, 'sequence_' + str(i), 'segmentation')
                    output_sequence_data['pose'] = seq_pose

                    exp_results[algorithm['label']][object_name].append(output_sequence_data)


    def evaluate_experiment(self, experiment_name):
        """Evaluate a single experiment."""

        print('Processing experiment: ' + experiment_name)
        print('')

        self.data[experiment_name] = {}
        exp_data = self.data[experiment_name]

        self.results[experiment_name] = {}
        exp_results = self.results[experiment_name]

        # Reserve a variable for the virtual object 'ALL'
        # (representing a unique sequence given by the union of all the sequences)
        gt_pose_ALL = {}
        pose_ALL = {}

        gt_vel_ALL = {}
        vel_ALL = {}

        time_ALL = {}

        # Load experiment
        experiment = self.experiments(experiment_name)

        # Load data for all the algorithms
        for algorithm in experiment:
            loader = DataLoader(algorithm)
            exp_data[algorithm['label']] = loader.load()

        # Evaluate metrics for the algorithm in the experiment
        # For each algorithm
        for algorithm in experiment:
            print('Processing metrics' + str(self.metric_names) + ' for algorithm with label ' + algorithm['label'] + '.')

            dataset_name = algorithm['config']['dataset']

            excluded_objects = algorithm['config']['excluded_objects']

            # Find out whether ground truth data is available for velocities
            has_velocity_gt = dataset_name + '_velocity' in self.data['gt']

            # Check if ground truth data is available and stop evaluation otherwise
            if dataset_name not in self.data['gt'] :
                continue

            exp_results[algorithm['label']] = {}

            # For each object
            for object_name in self.objects[dataset_name]:

                print('    processing object ' + object_name)
                exp_results[algorithm['label']][object_name] = {}

                if object_name == 'ALL':
                    gt_pose = gt_pose_ALL
                    pose = pose_ALL

                    gt_vel = gt_vel_ALL
                    vel = vel_ALL

                    time = time_ALL

                elif object_name in excluded_objects:
                    continue

                else:
                    gt_pose = None
                    pose = None

                    gt_vel = None
                    vel = None

                    time = None

                    # For each sequence belonging to the object
                    for i, sequence_data in enumerate(exp_data[algorithm['label']][object_name]):

                        seq_gt_pose_all = self.data['gt'][dataset_name][object_name][i]
                        seq_gt_pose = None

                        if has_velocity_gt:
                            seq_gt_vel_all = self.data['gt'][dataset_name + '_velocity'][object_name][i]
                            seq_gt_vel = None

                        seq_pose_all = sequence_data['pose']
                        seq_pose = None

                        # Find out whether it is possible to evaluate velocity metrics for this sequence
                        has_velocity = has_velocity_gt and 'velocity' in sequence_data

                        if has_velocity:
                            seq_vel_all = sequence_data['velocity']
                            seq_vel = None

                        if 'time' in sequence_data:
                            seq_time_all = sequence_data['time']
                            seq_time = None

                        # Check if the user required a specific subset of indexes for the evaluation
                        # and use them for all algorithms different from the one whose indexes are considered
                        # e.g. PoseRBPF running at 7fps will produce less frames than the ground truth
                        # and we might want to evaluate all the algorithms on that subset of frames
                        if self.subset_from is not None and algorithm['label'] != self.subset_from:
                            seq_pose_indexes = exp_data[self.subset_from][object_name][i]['indexes']

                            # Take into account HO-3D experiments with missing DOPE predictions at the beginning of the scene
                            if dataset_name == 'ho3d':
                                if object_name in self.ho3d_padding_list:
                                    padding_info = self.ho3d_padding_list[object_name]
                                    if i in padding_info:
                                        padding_info_i = padding_info[i]
                                        seq_pose_indexes = seq_pose_indexes[seq_pose_indexes >= padding_info_i['padding']]

                            seq_gt_pose = seq_gt_pose_all[seq_pose_indexes, :]

                            seq_pose = seq_pose_all[seq_pose_indexes, :]

                            if has_velocity:
                                seq_gt_vel = seq_gt_vel_all[seq_pose_indexes, :]
                                seq_vel = seq_vel_all[seq_pose_indexes, :]

                            if 'time' in sequence_data:
                                seq_time = seq_time_all[seq_pose_indexes, :]
                        else:
                            # Check if the length of ground truth and pose is the same
                            if seq_gt_pose_all.shape != seq_pose_all.shape:
                                # if not check if a list of indexes is provided
                                if not 'indexes' in sequence_data:
                                    print('Algorithm ' + algorithm['name'] + ' (label ' + algorithm['label'] + ')' +\
                                          ' provides ' + str(seq_pose_all.shape) + ' matrix as pose while the ground truth has shape ' + \
                                          str(seq_gt_pose_all.shape) + '. However, a list of indexes for the poses has not been provided.' + \
                                          ' Cannot continue.')

                                seq_pose_indexes = sequence_data['indexes']
                                seq_pose = seq_pose_all

                                if has_velocity:
                                    seq_vel = seq_vel_all

                                if 'time' in sequence_data:
                                    seq_time = seq_time_all

                                if self.expand_if_missing:
                                    seq_gt_pose = seq_gt_pose_all

                                    # Take into account HO-3D experiments with missing DOPE predictions at the beginning of the scene
                                    if dataset_name == 'ho3d':
                                        if object_name in self.ho3d_padding_list:
                                            padding_info = self.ho3d_padding_list[object_name]
                                            if i in padding_info:
                                                padding_info_i = padding_info[i]
                                                selection_vector = seq_pose_indexes >= padding_info_i['padding']
                                                seq_pose_indexes = seq_pose_indexes[selection_vector] - padding_info_i['padding']

                                                skipped_indexes = numpy.array([index for index in range(seq_pose_all.shape[0])])
                                                skipped_indexes = skipped_indexes[selection_vector]
                                                seq_pose = seq_pose_all[skipped_indexes, :]

                                                seq_gt_pose = seq_gt_pose_all[padding_info_i['padding']:, :]

                                    # Make a copy of the data
                                    tmp_seq_pose = copy.deepcopy(seq_pose)
                                    seq_pose = []
                                    seq_pose.append(tmp_seq_pose[0, :])

                                    if has_velocity:
                                        tmp_seq_vel = copy.deepcopy(seq_vel)
                                        seq_vel = []
                                        seq_vel.append(tmp_seq_vel[0, :])

                                    if 'time' in sequence_data:
                                        tmp_seq_time = copy.deepcopy(seq_time)
                                        seq_time = []
                                        seq_time.append(tmp_seq_time(0))

                                    # At this point we are sure that the first frame is valid
                                    for j in range(1, seq_gt_pose.shape[0]):
                                        if j not in seq_pose_indexes:
                                            seq_pose.append(seq_pose[-1])

                                            if has_velocity:
                                                seq_vel.append(seq_vel[-1])

                                            if 'time' in sequence_data:
                                                seq_time.append(seq_time[-1])
                                        else:
                                            seq_pose.append(tmp_seq_pose[list(seq_pose_indexes).index(j), :])

                                            if has_velocity:
                                                seq_vel.append(tmp_seq_vel[list(seq_pose_indexes).index(j), :])

                                            if 'time' in sequence_data:
                                                seq_time.append(tmp_seq_time[list(seq_pose_indexes).index(j)])

                                    seq_pose = numpy.array(seq_pose)
                                    if has_velocity:
                                        seq_vel = numpy.array(seq_vel)

                                    if 'time' in sequence_data:
                                        seq_time = numpy.array(seq_time)

                                else:
                                    # Take into account HO-3D experiments with missing DOPE predictions at the beginning of the scene
                                    if dataset_name == 'ho3d':
                                        if object_name in self.ho3d_padding_list:
                                            padding_info = self.ho3d_padding_list[object_name]
                                            if i in padding_info:
                                                padding_info_i = padding_info[i]
                                                selection_vector = seq_pose_indexes >= padding_info_i['padding']
                                                seq_pose_indexes = seq_pose_indexes[selection_vector]

                                                skipped_indexes = numpy.array([index for index in range(seq_pose_all.shape[0])])
                                                skipped_indexes = skipped_indexes[selection_vector]
                                                seq_pose = seq_pose_all[skipped_indexes, :]

                                                if has_velocity:
                                                    seq_vel = seq_vel_all[skipped_indexes, :]

                                                if 'time' in sequence_data:
                                                    seq_time = seq_time_all[skipped_indexes, :]

                                    seq_gt_pose = seq_gt_pose_all[seq_pose_indexes, :]

                                    if has_velocity:
                                        seq_gt_vel = seq_gt_vel_all[seq_pose_indexes, :]

                            else:
                                seq_gt_pose = seq_gt_pose_all

                                seq_pose = seq_pose_all

                                if has_velocity:
                                    seq_gt_vel = seq_gt_vel_all
                                    seq_vel = seq_vel_all

                                if 'time' in sequence_data:
                                    seq_time = seq_time_all

                                # Take into account HO-3D experiments with missing DOPE predictions at the beginning of the scene
                                if dataset_name == 'ho3d':
                                    if object_name in self.ho3d_padding_list:
                                        padding_info = self.ho3d_padding_list[object_name]
                                        if i in padding_info:
                                            padding_info_i = padding_info[i]
                                            seq_gt_pose = seq_gt_pose[padding_info_i['padding'] :, :]

                                            seq_pose = seq_pose[padding_info_i['padding'] :, :]

                                            if has_velocity:
                                                seq_gt_vel = seq_gt_vel[padding_info_i['padding'] :, :]
                                                seq_vel = seq_vel[padding_info_i['padding'] :, :]

                                            if 'time' in sequence_data:
                                                seq_time = seq_time[padding_info_i['padding'] :, :]

                        # Add computation time for optical flow if necessary
                        if 'of_set' in algorithm['config']:
                            of_set = algorithm['config']['of_set']

                            if 'nvof_1' in of_set:
                                # 3 ms are required for optical flow NVOF_1_0 @ 1280p
                                # Warning this will be wrong if you use NVOF_1_0 at different resolution
                                seq_time += 3
                            elif 'nvof_2' in of_set:
                                # 6 ms are required for optical flow NVOF_2_0 @ 640p
                                # Warning this will be wrong if you use NVOF_2_0 at different resolution
                                seq_time += 6

                        # Concatenate data belonging to the same object
                        if i == 0:
                            gt_pose = seq_gt_pose

                            pose = seq_pose

                            if has_velocity:
                                gt_vel = seq_gt_vel
                                vel = seq_vel

                            if 'time' in sequence_data:
                                time = seq_time
                        else:
                            gt_pose = numpy.concatenate((gt_pose, seq_gt_pose), axis = 0)

                            pose = numpy.concatenate((pose, seq_pose), axis = 0)

                            if has_velocity:
                                gt_vel = numpy.concatenate((gt_vel, seq_gt_vel), axis = 0)
                                vel = numpy.concatenate((vel, seq_vel), axis = 0)

                            if 'time' in sequence_data:
                                time = numpy.concatenate((time, seq_time), axis = 0)

                    gt_pose_ALL[object_name] = gt_pose

                    pose_ALL[object_name] = pose

                    if algorithm['name'] == 'ours' and has_velocity:
                    # Take into account pole displacement of the 6D velocity
                    # in order to compare to the ground truth signal
                        for i in range(gt_vel.shape[0]):
                            vo = vel[i, 0:3]
                            w = vel[i, 3:6]
                            r = gt_pose[i, 0:3]
                            vel[i, 0:3] = vo + numpy.cross(w, r)

                    if has_velocity:
                        gt_vel_ALL[object_name] = gt_vel
                        vel_ALL[object_name] = vel

                    if 'time' in sequence_data:
                        time_ALL[object_name] = time

                for metric_name in self.metrics:
                    if 'velocity' in metric_name:
                        exp_results[algorithm['label']][object_name][metric_name] = self.metrics[metric_name].evaluate(object_name, gt_vel, vel, time)
                    else:
                        exp_results[algorithm['label']][object_name][metric_name] = self.metrics[metric_name].evaluate(object_name, gt_pose, pose, time)

        # Remove the experiment if there are no results for that experiment
        if len(exp_results) == 0:
            del(self.results[experiment_name])

        print('')


    def render(self):
        """Render results."""

        renders = {}

        if self.output_head == 'markdown':
            renderer = ResultsMarkdownRenderer()
        elif self.output_head == 'latex':
            renderer = ResultsLaTeXRenderer()
        elif self.output_head == 'latex-summary':
            renderer = ResultsLaTeXSummaryRenderer()
        elif self.output_head == 'video':
            renderer = ResultsVideoRenderer()
        elif self.output_head == 'video-segmentation':
            renderer = ResultsSegmentationVideoRenderer()
        elif self.output_head == 'thumbnail':
            renderer = ResultsThumbnailRenderer()
        elif self.output_head == 'plot':
            # not implemented
            pass

        for result_name in self.results:
            renders[result_name] = renderer.render(result_name, self.results[result_name], self.objects, self.experiments, self.subset_from)

        return renderer.extension, renders


    def save(self, renders, extension):
        """Save results."""

        if len(renders) == 0:
            return

        os.makedirs(self.output_path, exist_ok = 'True')
        if self.output_head in ['markdown', 'latex', 'latex-summary']:

            print('Rendered results saved to:')

            for render_name in renders:
                file_path = os.path.join(self.output_path, render_name)
                file_path += '_' + self.metric_name
                if self.subset_from is not None:
                    file_path += '_subset_' + self.subset_from
                file_path +=  '.' + extension
                file_path = "_".join(file_path.split(' '))

                print('"' + render_name + '" in ' + file_path)

                with open(file_path, 'w') as f:
                    f.write(renders[render_name])

        elif self.output_head == 'video':
            # nothing to do there as the video renderer directly saves on disk
            pass
        elif self.output_head == 'plot':
            # not implemented
            pass


def main():
    experiments = Experiments().experiments
    experiment_names = [name for name in experiments]

    parser = argparse.ArgumentParser()
    parser.add_argument\
    (
        '--metric-name',
        dest = 'metric_name',
        type = str,
        required =\
        (
            'video' not in sys.argv and\
            'video-segmentation' not in sys.argv and\
            'thumbnail' not in sys.argv
        ),
        help = "available metrics: ['ad', 'add-distances', 'error', 'rmse']"
    )

    parser.add_argument('--experiment-name', dest = 'experiment_name', type = str, required = False, help = 'available experiments: ' + str(experiment_names))
    parser.add_argument('--use-subset', dest = 'use_subset', type = str, required = False, help = "name of the algorithm whose ground truth indexes should be used for the evaluation. available names are ['ours', 'se3tracknet, 'poserbpf']")
    parser.add_argument('--output-head', dest = 'output_head', type = str, required = False, help = "available heads: ['latex', 'latex-summary', 'markdown', 'plot', 'thumbnail', 'video', 'video-segmentation']", default = 'markdown')
    parser.add_argument('--output-path', dest = 'output_path', type = str, required = False, help = "where to save results", default = './evaluation_output')
    parser.add_argument('--disable-ho3d-padding', dest = 'disable_ho3d_padding', type = bool, default = False, help = "whether to handle that DOPE predictions for HO-3D in sequence 006_mustard_bottle_2 are missing starting from the first frame")
    parser.add_argument('--expand-if-missing', dest = 'expand_if_missing', type = bool, default = False, help = "whether to sample and hold algorithm output if provided frames are less than ground truth frames")

    options = parser.parse_args()

    evaluator = Evaluator(options.metric_name, options.experiment_name, options.output_head, options.output_path, options.use_subset, options.disable_ho3d_padding, options.expand_if_missing)


if __name__ == '__main__':
    main()
