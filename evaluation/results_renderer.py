#===============================================================================
#
# Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT)
#
# This software may be modified and distributed under the terms of the
# GPL-2+ license. See the accompanying LICENSE file for details.
#
#===============================================================================

import sys
sys.path.insert(0, './build/lib/')

import ffmpeg
import markdown_table
import matplotlib
import matplotlib.pyplot as plt
import numpy
import object_renderer
import os
from decimal import Decimal
from pathlib import Path
from matplotlib import rc


rc('font',**{'family':'sans-serif','sans-serif':['Helvetica']})
rc('text', usetex = True)


class ResultsLaTeXRenderer():

    def __init__(self):
        """Contructor."""

        self.extension = 'tex'


    def render(self, results_name, results, objects, experiments_data, subset_from):
        """Renderer."""
        pass


class ResultsMarkdownRenderer():

    def __init__(self):
        """Contructor."""

        self.alg_labels =\
        {
            'ours' : 'ours',
            'ours_gt_mask' : 'gt mask',
            'ours_ho3d_mask' : 'ho3d mask',
            'ours_gt_pose' : 'gt pose',
            'ours_gt' : 'gt',
            'ours_no_outrej' : 'w/o outrej',
            'ours_no_posesync' : 'w/o pose sync',
            'ours_no_flowaid' : 'w/o flow',
            'ours_no_pose' : 'w/o pose',
            'ours_no_velocity' : 'w/o vel',
            'ours_test' : 'test',
            'ours_nvof_1' : 'nvof 1',
            'ours_nvof_2' : 'nvof 2',
            'se3tracknet' : 'se3',
            'poserbpf' : 'rbpf',
            'dope_ideal' : 'dope ideal',
            'dope_real' : 'dope real'
        }

        self.metric_labels =\
        {
            'rmse_cartesian_3d' : '3d',
            'rmse_cartesian_x' : 'x',
            'rmse_cartesian_y' : 'y',
            'rmse_cartesian_z' : 'z',
            'rmse_angular' : 'ang',
            'add' : 'ADD',
            'adi' : 'ADI',
            'time' : 'time',
            'excess_33_ms' : '% > 33 ms',
            'time_of' : 'time w/ OF',
            'excess_33_ms_of' : '% > 33 ms w/ OF'
        }

        self.digits =\
        {
            'rmse_cartesian_3d' : Decimal('.1'),
            'rmse_cartesian_x' : Decimal('.1'),
            'rmse_cartesian_y' : Decimal('.1'),
            'rmse_cartesian_z' : Decimal('.1'),
            'rmse_angular' : Decimal('.001'),
            'add' : Decimal('.01'),
            'adi' : Decimal('.01'),
            'time' : Decimal('.1'),
            'excess_33_ms' : Decimal('.001'),
            'time_of' : Decimal('.1'),
            'excess_33_ms_of' : Decimal('.001')
        }

        less_than = lambda x, y : x < y
        greater_than = lambda x, y : x > y

        self.best_evaluator=\
        {
            'rmse_cartesian_3d' : less_than,
            'rmse_cartesian_x' : less_than,
            'rmse_cartesian_y' : less_than,
            'rmse_cartesian_z' : less_than,
            'rmse_angular' : less_than,
            'add' : greater_than,
            'adi' : greater_than,
            'time' : less_than,
            'excess_33_ms' : less_than,
            'time_of' : less_than,
            'excess_33_ms_of' : less_than
        }

        self.extension = 'md'


    def render(self, results_name, results, objects, experiments_data, subset_from):
        """Renderer."""

        # Get dataset name
        dataset_name = experiments_data(results_name)[0]['config']['dataset']

        # Get excluded objects
        excluded_objects = experiments_data(results_name)[0]['config']['excluded_objects']

        # Get objects
        objects = objects[dataset_name]

        # Get algorithm names
        algorithm_names = [name for name in results]

        # Get metric names
        metric_names = [name for name in results[algorithm_names[0]][objects[0]]]

        # Construct table header
        header_vector = ['obj']

        for j, metric_name in enumerate(metric_names):
            for i, alg_name in enumerate(algorithm_names):
                header_vector.append(self.metric_labels[metric_name] + ' (' + self.alg_labels[alg_name] + ')')

        # Find best result for each row
        best = {}
        for object_name in objects:

            if object_name in excluded_objects:
                continue

            best[object_name] = {}

            for i, metric_name in enumerate(metric_names):
                best[object_name][metric_name] = algorithm_names[0]
                best_result = results[algorithm_names[0]][object_name][metric_name]

                for j, alg_name in enumerate(algorithm_names):
                    result = results[alg_name][object_name][metric_name]

                    if self.best_evaluator[metric_name](result, best_result):
                        best_result = result
                        best[object_name][metric_name] = alg_name

        # Construct table content
        data_matrix = []

        for object_name in objects:

            if object_name in excluded_objects:
                continue

            row = []
            row.append(object_name)
            for j, metric_name in enumerate(metric_names):
                for i, alg_name in enumerate(algorithm_names):
                    result = results[alg_name][object_name][metric_name]
                    result_str = str(Decimal(result).quantize(self.digits[metric_name]))
                    if alg_name == best[object_name][metric_name]:
                        result_str = '**' + result_str + '**'
                    row.append(result_str)
            data_matrix.append(row)

        # Get experiment configuration
        experiment_configuration = experiments_data.print_experiment(results_name)

        output = '### ' + results_name + '\r\n\r\n'
        if subset_from is not None:
            output += '**Ground truth frames taken from ' + subset_from + ' results.**\r\n\r\n'
        output += experiment_configuration
        output += markdown_table.render(header_vector, data_matrix)

        return output


class ResultsMatplotlibRenderer():

    def __init__(self):
        """Constructor."""

        self.extension = 'png'


    def cm_to_inch(self, *tupl):
        """Convert a tuple of centimeters to inches."""

        inch = 2.54
        if isinstance(tupl[0], tuple):
            return tuple(i/inch for i in tupl[0])
        else:
            return tuple(i/inch for i in tupl)


    def render(self, results_name, results, objects, experiments_data, subset_from):
        """Renderer."""

        # Find metric name
        algorithm_names = [name for name in results]
        metric_names = [name for name in results[algorithm_names[0]][objects[0]]]

        if 'add-distances' in metric_names:
            return self.render_add_distances(results, objects)
        elif 'error_cartesian' in metric_names or 'error_angular' in metric_names:
            return self.render_error(results, objects)


    def render_add_distances(self, results, objects):
        """Render ADD distances."""

        fig, ax = plt.subplots(int(numpy.ceil(len(objects) / 3)), 3)

        for i, object_name in enumerate(objects):
            axis = ax[int(i / 3), int(i % 3)]
            axis.set_title('$\mathrm{' + "\,".join(object_name.split('_')) + '}$')
            axis.grid()

            # Plot data
            for j, alg_name in enumerate(results):
                distances = results[alg_name][object_name]['add-distances']
                axis.plot(distances)

            # Add x axis labels to final row only
            if int(i / 3) == int((len(objects) - 1) / 3):
                axis.set_xlabel('$\mathrm{Samples}$')
            # Add y axis labels to first column only
            if int(i % 3) == 0:
                axis.set_ylabel('$\mathrm{ADD\,-\,distances}$')

        # Add legend
        algorithm_labels = []
        for j, alg_name in enumerate(results):
            algorithm_labels.append('$\mathrm{' + alg_name + '}$')
        fig.legend(labels = algorithm_labels, ncol = 3, loc = "upper center", frameon = False)

        figure = plt.gcf()
        figure.set_size_inches(self.cm_to_inch(48, 24))

        return [figure]


    def render_error(self, results, objects):
        """Render Cartesian and angular error as function of time."""

        figures = []

        for i, object_name in enumerate(objects):

            fig, ax = plt.subplots(2, 2)

            # Plot data
            for j, alg_name in enumerate(results):
                error_x = results[alg_name][object_name]['error_cartesian_x']
                error_y = results[alg_name][object_name]['error_cartesian_y']
                error_z = results[alg_name][object_name]['error_cartesian_z']
                error_angular = results[alg_name][object_name]['error_angular']

                ax[0, 0].plot(error_x, linewidth = 0.8)
                ax[0, 1].plot(error_y, linewidth = 0.8)
                ax[1, 0].plot(error_z, linewidth = 0.8)
                ax[1, 1].plot(error_angular, linewidth = 0.8)

                ax[0, 0].grid()
                ax[0, 1].grid()
                ax[1, 0].grid()
                ax[1, 1].grid()

                # Add titles
                ax[0, 0].set_title('$e_{x}$')
                ax[0, 1].set_title('$e_{y}$')
                ax[1, 0].set_title('$e_{z}$')
                ax[1, 1].set_title('$e_{a}$')

                # Add x axis labels to final row only
                ax[1, 0].set_xlabel('$\mathrm{Samples}$')
                ax[1, 1].set_xlabel('$\mathrm{Samples}$')

                # Add y axis labels
                ax[0, 0].set_ylabel('$(cm)$')
                ax[0, 1].set_ylabel('$(cm)$')
                ax[1, 0].set_ylabel('$(cm)$')
                ax[1, 1].set_ylabel('$(deg)$')

            algorithm_labels = []
            for j, alg_name in enumerate(results):
                algorithm_labels.append('$\mathrm{' + alg_name + '}$')
            fig.legend(labels = algorithm_labels, ncol = 3, loc = "upper center", frameon = False)

            figure = plt.gcf()
            plt.subplots_adjust(hspace = 0.4)
            figure.set_size_inches(self.cm_to_inch(36, 12))

            figures.append(figure)



        return figures


class ResultsVideoRenderer():

    def __init__(self):
        """Contructor."""

        self.extension = 'mp4'


    def frames_to_video(self, cam_intrinsics, input_path):
        """Convert frames to output mp4 video."""

        input_options =\
        {
            'r': 30,
            'f': 'image2'
        }

        output_options =\
        {
            'crf': 25,
            'pix_fmt': 'yuv420p',
            'vcodec' : 'libx264',
            's' : str(cam_intrinsics['width']) + 'x' + str(cam_intrinsics['height'])
        }

        ffmpeg\
        .input(os.path.join(input_path, '%d.png'), **input_options)\
        .output(os.path.join(input_path, 'output.mp4'), **output_options)\
        .run(overwrite_output = True)


    def render(self, results_name, results, objects, experiments_data, subset_from):
        """Renderer."""

        # for each algorithm in the experiment
        for algorithm_name in results:
            algorithm_results = results[algorithm_name]

            # for each object
            for object_name in algorithm_results:
                object_name_results = algorithm_results[object_name]

                # for each sequence
                for sequence_results in object_name_results:

                    # create output folder
                    os.makedirs(sequence_results['output_path'], exist_ok = True)

                    # render frames
                    object_renderer.render\
                    (
                        sequence_results['mesh_path'],
                        sequence_results['rgb_path'],
                        sequence_results['output_path'],
                        sequence_results['cam_intrinsics'],
                        sequence_results['pose']
                    )

                    # convert to a video
                    self.frames_to_video(sequence_results['cam_intrinsics'], sequence_results['output_path'])

                    # remove frames to free space
                    [f.unlink() for f in Path(sequence_results['output_path']).glob("*.png") if f.is_file()]
