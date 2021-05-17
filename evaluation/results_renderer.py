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

import cv2
import ffmpeg
import glob
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


class ResultsTableRenderer():

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
            'poserbpf_50' : 'rbpf 15/50',
            'poserbpf_100' : 'rbpf 15/100',
            'poserbpf_200' : 'rbpf 7/200',
            'dope' : 'dope',
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
            'rmse_linear_velocity' : 'v',
            'rmse_angular_velocity' : 'w',
            'max_linear_velocity' : 'max v',
            'max_angular_velocity' : 'max w',
            'add' : 'ADD',
            'adi' : 'ADI',
            'time' : 'time',
            'excess_33_ms' : '# > 33 ms',
        }

        self.digits =\
        {
            'rmse_cartesian_3d' : Decimal('.1'),
            'rmse_cartesian_x' : Decimal('.1'),
            'rmse_cartesian_y' : Decimal('.1'),
            'rmse_cartesian_z' : Decimal('.1'),
            'rmse_angular' : Decimal('.001'),
            'rmse_linear_velocity' : Decimal('.001'),
            'rmse_angular_velocity' : Decimal('.001'),
            'max_linear_velocity' : Decimal('.001'),
            'max_angular_velocity' : Decimal('.001'),
            'add' : Decimal('.01'),
            'adi' : Decimal('.01'),
            'time' : Decimal('.1'),
            'excess_33_ms' : Decimal('0')
        }

        less_than = lambda x, y : x < y
        greater_than = lambda x, y : x > y
        nothing = lambda x, y : False

        self.best_evaluator=\
        {
            'rmse_cartesian_3d' : less_than,
            'rmse_cartesian_x' : less_than,
            'rmse_cartesian_y' : less_than,
            'rmse_cartesian_z' : less_than,
            'rmse_angular' : less_than,
            'rmse_linear_velocity' : less_than,
            'rmse_angular_velocity' : less_than,
            'max_linear_velocity' : nothing,
            'max_angular_velocity' : nothing,
            'add' : greater_than,
            'adi' : greater_than,
            'time' : less_than,
            'excess_33_ms' : less_than
        }


    def process(self, results_name, results, objects, experiments_data, subset_from):
        """Streamline processing of object names and excluded objects, algorithm names,
        metric names and best algorithms given the results."""

        # Get objects
        dataset_name = experiments_data(results_name)[0]['config']['dataset']
        objects = objects[dataset_name]

        # Get excluded objects
        excluded_objects = experiments_data(results_name)[0]['config']['excluded_objects']

        # Get algorithm names
        algorithm_names = [name for name in results]

        # Get metric names
        metric_names = [name for name in results[algorithm_names[0]][objects[0]]]

        # Find best result for each row
        best = {}
        for object_name in objects:

            if object_name in excluded_objects:
                continue

            best[object_name] = {}

            for i, metric_name in enumerate(metric_names):

                for alg_name in algorithm_names:
                    if 'gt' not in alg_name and 'ideal' not in alg_name:
                        best[object_name][metric_name] = alg_name
                        best_result = results[alg_name][object_name][metric_name]
                        break

                for j, alg_name in enumerate(algorithm_names):
                    result = results[alg_name][object_name][metric_name]

                    # Do not consider ground truth-aided scenarios to find the best
                    if 'gt' in alg_name or 'ideal' in alg_name:
                        continue

                    if self.best_evaluator[metric_name](result, best_result):
                        best_result = result
                        best[object_name][metric_name] = alg_name

        return objects, excluded_objects, algorithm_names, metric_names, best


class ResultsLaTeXRenderer(ResultsTableRenderer):

    def __init__(self):
        """Contructor."""

        super().__init__()

        self.extension = 'tex'

        self.alg_labels =\
        {
            'ours' : 'Ours',
            'ours_gt_mask' : 'ideal segm.',
            'ours_gt_pose' : 'ideal pose',
            'ours_gt' : 'ideal segm. and pose',
            'ours_flowaid' : 'refined segm.',
            'ours_no_flowaid' : 'w/o refined segm.',
            'ours_no_posesync' : 'w/o refined pose',
            'ours_no_outrej' : 'w/o outlier rej.',
            'ours_no_velocity' : 'w/o velocity.',
            'ours_no_pose' : 'w/o pose.',
            'dope_real' : 'DOPE',
            'dope_ideal' : 'ideal DOPE',
            'se3tracknet' : 'se3',
            'poserbpf' : 'PoseRBPF',
            'dope' : 'DOPE'
        }

        self.metric_labels =\
        {
            'rmse_cartesian_3d' : 'RMSE $e_{r} (cm)$',
            'rmse_cartesian_x' : 'RMSE $e_{x} (cm)$',
            'rmse_cartesian_y' : 'RMSE $e_{y} (cm)$',
            'rmse_cartesian_z' : 'RMSE $e_{z} (cm)$',
            'rmse_angular' : 'RMSE $e_{a}$ (deg)',
            'rmse_linear_velocity' : 'RMSE $e_{v} (m/s)$',
            'rmse_angular_velocity' : 'RMSE $e_{\omega} (deg/s)$',
            'add' : 'ADD (\%)'
        }


    def render(self, results_name, results, objects, experiments_data, subset_from):
        """Renderer."""

        objects, excluded_objects, algorithm_names, metric_names, best =\
            self.process(results_name, results, objects, experiments_data, subset_from)

        # Construct table header
        output =\
            '\\begin{table*}\r\n' +\
            '    \centering\r\n' +\
            '    \caption{.\label{tab:' + results_name + '}}\r\n' +\
            '    \\begin{tabular}{|'
        for i in range(len(metric_names) * len(algorithm_names) + 1):
            output += ' c |'
        output += '}\r\n'
        output +=\
            '    \hline\r\n'

        # First row
        output +=\
            '    metric'
        for metric_name in metric_names:
            output +=' & \multicolumn{' + str(len(algorithm_names)) + '}{c|}{' + self.metric_labels[metric_name] + '}'
        output += '\\\\\r\n'

        # Second row
        output +=\
            '    \hline\r\n'
        output +=\
            '    method'
        for metric_name in metric_names:
            for alg_name in algorithm_names:
                output += ' & ' + self.alg_labels[alg_name]
        output += '\\\\\r\n'

        # Results
        output +=\
            '    \hline\r\n'
        for object_name in objects:

            if object_name in excluded_objects:
                continue

            if object_name == 'ALL':
                output +=\
            '    \hline\r\n'

            name = object_name
            if name != 'ALL':
                name = '{\_}'.join(name.split('_'))

            output +=\
            '    ' + name
            for j, metric_name in enumerate(metric_names):
                for i, alg_name in enumerate(algorithm_names):
                    result = results[alg_name][object_name][metric_name]
                    result_str = str(Decimal(result).quantize(self.digits[metric_name]))
                    if alg_name == best[object_name][metric_name]:
                        result_str = '\\textbf{' + result_str + '}'

                    output += ' & ' + result_str

            output += '\\\\\r\n'

        # Construct table ending
        output +=\
            '    \hline\r\n' +\
            '    \end{tabular}\r\n' +\
            '\end{table*}\r\n'

        return output


class ResultsLaTeXSummaryRenderer(ResultsTableRenderer):

    def __init__(self):
        """Contructor."""

        super().__init__()

        self.extension = 'tex'

        self.alg_labels =\
        {
            'ours' : 'Ours',
            'ours_gt_mask' : 'ideal segm.',
            'ours_gt_pose' : 'ideal pose',
            'ours_gt' : 'ideal segm. and pose',
            'ours_flowaid' : 'refined segm.',
            'ours_no_flowaid' : 'w/o refined segm.',
            'ours_no_posesync' : 'w/o refined pose',
            'ours_no_outrej' : 'w/o outlier rej.',
            'ours_no_velocity' : 'w/o velocity.',
            'ours_no_pose' : 'w/o pose.',
            'dope_real' : 'DOPE',
            'dope_ideal' : 'ideal DOPE'
        }

        self.metric_labels =\
        {
            'rmse_cartesian_3d' : 'RMSE $e_{r} (cm)$',
            'rmse_angular' : 'RMSE $e_{a}$ (deg)',
            'add' : 'ADD (\%)'
        }


    def render(self, results_name, results, objects, experiments_data, subset_from):
        """Renderer."""

        objects, excluded_objects, algorithm_names, metric_names, best =\
            self.process(results_name, results, objects, experiments_data, subset_from)

        # Construct table header
        output =\
            '\\begin{table}\r\n' +\
            '    \centering\r\n' +\
            '    \caption{.\label{tab:' + results_name + '}}\r\n' +\
            '    \\begin{tabular}{|'
        for i in range(len(metric_names) + 1):
            output += ' c |'
        output += '}\r\n'
        output +=\
            '    \hline\r\n'

        # First row
        output +=\
            '    metric'
        for metric_name in metric_names:
            output +=' & ' + self.metric_labels[metric_name]
        output += '\\\\\r\n'

        # Results
        output +=\
            '    \hline\r\n'
        for object_name in objects:

            if object_name in excluded_objects:
                continue

            if object_name != 'ALL':
                continue

            for i, alg_name in enumerate(algorithm_names):
                output +=\
            '    ' + self.alg_labels[alg_name]

                for j, metric_name in enumerate(metric_names):

                    result = results[alg_name][object_name][metric_name]
                    result_str = str(Decimal(result).quantize(self.digits[metric_name]))
                    if alg_name == best[object_name][metric_name]:
                        result_str = '\\textbf{' + result_str + '}'

                    output += ' & ' + result_str

                output += '\\\\\r\n'

        # Construct table ending
        output +=\
            '    \hline\r\n' +\
            '    \end{tabular}\r\n' +\
            '\end{table}\r\n'

        return output


class ResultsMarkdownRenderer(ResultsTableRenderer):

    def __init__(self):
        """Contructor."""

        super().__init__()

        self.extension = 'md'


    def render(self, results_name, results, objects, experiments_data, subset_from):
        """Renderer."""

        objects, excluded_objects, algorithm_names, metric_names, best =\
            self.process(results_name, results, objects, experiments_data, subset_from)

        # Construct table header
        header_vector = ['obj']

        for j, metric_name in enumerate(metric_names):
            for i, alg_name in enumerate(algorithm_names):
                header_vector.append(self.metric_labels[metric_name] + ' (' + self.alg_labels[alg_name] + ')')

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
                    os.makedirs(sequence_results['output_path_rgb'], exist_ok = True)

                    # create list of indexes
                    indexes = list(range(sequence_results['pose'].shape[0]))

                    # render frames
                    object_renderer.render\
                    (
                        sequence_results['mesh_path'],
                        sequence_results['rgb_path'],
                        sequence_results['output_path_rgb'],
                        sequence_results['cam_intrinsics'],
                        indexes,
                        sequence_results['pose']
                    )

                    # convert to a video
                    self.frames_to_video(sequence_results['cam_intrinsics'], sequence_results['output_path_rgb'])

                    # remove frames to free space
                    [f.unlink() for f in Path(sequence_results['output_path_rgb']).glob("*.png") if f.is_file()]


class ResultsThumbnailRenderer():

    def __init__(self):
        """Contructor."""

        self.extension = 'png'


    def render(self, results_name, results, objects, experiments_data, subset_from):
        """Renderer."""

        # this render awaits extra argument in the experiments_data field
        experiment_data = experiments_data(results_name)

        # store output paths
        paths = []

        # for each algorithm in the experiment
        for i, algorithm_name in enumerate(results):

            if 'thumbnail' not in experiment_data[i]['config']:
                print('Output head "thumbnail" requires extra parameters set "thumbnail" in the experiment configuration')

            parameters = experiment_data[i]['config']['thumbnail']

            sequence_results = results[algorithm_name][parameters['object']][0]

            # create output folder
            os.makedirs(sequence_results['output_path_thumbnail'], exist_ok = True)

            # remove existing frames
            [f.unlink() for f in Path(sequence_results['output_path_thumbnail']).glob("*.png") if f.is_file()]

            # render frames
            object_renderer.render\
            (
                sequence_results['mesh_path'],
                sequence_results['rgb_path'],
                sequence_results['output_path_thumbnail'],
                sequence_results['cam_intrinsics'],
                parameters['frames'],
                sequence_results['pose']
            )

            paths.append(sequence_results['output_path_thumbnail'])

        # extract output path from any of the sequence output path
        output_path = '/'.join(sequence_results['output_path_thumbnail'].split('/')[:-4])

        # sequences share the same input rgb path
        paths.insert(0, sequence_results['rgb_path'])

        # sequences share the same crop size
        crop_size = parameters['crop']

        # sequences share the same list of indexes
        indexes = parameters['frames']

        # evaluate output size
        border_size = 10
        cols_width = crop_size[2] - crop_size[0]
        rows_width = crop_size[3] - crop_size[1]
        number_thumb = len(indexes)
        number_algorithms = 1 + len(paths)
        cols = (number_thumb - 1) * border_size + cols_width * number_thumb
        rows = (number_algorithms - 1) * border_size + rows_width * number_algorithms

        # compose image with thumbnails
        image = numpy.full((rows, cols, 3), (255, 255, 255), numpy.uint8)

        # attach frames
        for i, path in enumerate(paths):
            for j, index in enumerate(indexes):
                img = cv2.imread(os.path.join(path, str(index) + '.png'))
                img = img[crop_size[1] : crop_size[3], crop_size[0] : crop_size[2], :]
                image[(rows_width + border_size) * i : (rows_width + border_size) * i + rows_width,\
                      (cols_width + border_size) * j : (cols_width + border_size) * j + cols_width, :] = img

        # save
        cv2.imwrite(os.path.join(output_path, 'thumb.png'), image)
