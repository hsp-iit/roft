#===============================================================================
#
# Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT)
#
# This software may be modified and distributed under the terms of the
# GPL-2+ license. See the accompanying LICENSE file for details.
#
#===============================================================================

import markdown_table
from decimal import Decimal


class ResultsMarkdownRenderer():

    def __init__(self):
        """Contructor."""

        self.alg_labels =\
        {
            'ours' : 'ours',
            'se3tracknet' : 'se3',
            'poserbpf' : 'rbpf'
        }

        self.metric_labels =\
        {
            'rmse_cartesian_x' : 'x',
            'rmse_cartesian_y' : 'y',
            'rmse_cartesian_z' : 'z',
            'rmse_angular' : 'ang'
        }

        self.digits =\
        {
            'rmse_cartesian_x' : Decimal('.1'),
            'rmse_cartesian_y' : Decimal('.1'),
            'rmse_cartesian_z' : Decimal('.1'),
            'rmse_angular' : Decimal('.001')
        }

        less_than = lambda x, y : x < y
        greater_than = lambda x, y : x > y

        self.best_evaluator=\
        {
            'rmse_cartesian_x' : less_than,
            'rmse_cartesian_y' : less_than,
            'rmse_cartesian_z' : less_than,
            'rmse_angular' : less_than
        }

        self.extension = 'md'


    def render(self, results_name, results, objects, experiments_data, subset_from):
        """Renderer."""

        algorithm_names = [name for name in results]
        metric_names = [name for name in results[algorithm_names[0]][objects[0]]]

        # Construct table header
        header_vector = ['obj']

        for i, alg_name in enumerate(algorithm_names):
            for j, metric_name in enumerate(metric_names):
                header_vector.append(self.metric_labels[metric_name] + ' (' + self.alg_labels[alg_name] + ')')

        # Find best result for each row
        best = {}
        for object_name in objects:
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
            row = []
            row.append(object_name)
            for i, alg_name in enumerate(algorithm_names):
                for j, metric_name in enumerate(metric_names):
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


class ResultsLaTeXRenderer():

    def __init__(self):
        """Contructor."""
        pass


    def render(self, results_name, results):
        """Contructor."""
        pass
