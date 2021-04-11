#===============================================================================
#
# Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT)
#
# This software may be modified and distributed under the terms of the
# GPL-2+ license. See the accompanying LICENSE file for details.
#
#===============================================================================

import argparse
import os
from data_loader import DataLoader
from experiments import Experiments
from objects import Objects
from metrics import Metric
from results_renderer import ResultsMarkdownRenderer, ResultsLaTeXRenderer


class Evaluator():

    def __init__(self, metric_name, experiment_name, output_head, output_path):
        """Constructor."""

        self.data = {}
        self.results = {}

        # Load object names
        self.objects = Objects().objects
        print('List of objects:')
        print(self.objects)
        print('')

        # Load experiments
        self.experiments = Experiments()
        print('Available experiments:')
        print(self.experiments)
        print('')

        # Load ground truth data
        self.data['gt'] = DataLoader({'name' : 'gt'}).load()

        # Initialize metrics
        self.metrics = {}
        self.metric_names = []
        if metric_name == 'rmse':
            self.metric_names = ['rmse_cartesian_' + coord for coord in ['x', 'y', 'z']]
            # self.metric_names.append('rmse_angular')
        elif metric_name == 'ad':
            self.metric_names = ['add', 'adi']

        self.metrics = { name : Metric(name) for name in self.metric_names }

        # Process experiments
        if experiment_name is not None:
            self.evaluate_experiment(experiment_name)
        else:
            # Process all the experiments if the user does not specify a specific experiment
            for experiment in self.experiments.experiments:
                self.evaluate_experiment(experiment)

        # Render results
        if output_head is not None:
            extension, renders = self.render(output_head)

            if output_path is not None:
                print('Rendered results saved to:')
                for render_name in renders:
                    file_path = os.path.join(output_path, render_name + '.' + extension)
                    print('"' + render_name + '" in ' + file_path)
                    with open(file_path, 'w') as f:
                        f.write(renders[render_name])
            else:
                for render_name in renders:
                    print('')
                    print('Render for "' + render_name + '"')
                    print(renders[render_name])
                    print('')


    def evaluate_experiment(self, experiment_name):
        """Process a single experiment."""

        print('Processing experiment: ' + experiment_name)

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

        # Evaluate metrics for the algorithm in the experiment
        # For each algorithm
        for algorithm in experiment:
            print('Processing metrics' + str(self.metric_names) + ' for algorithm with label ' + algorithm['label'] + '.')
            exp_results[algorithm['label']] = {}

            # For each object
            for object_name in self.objects:
                exp_results[algorithm['label']][object_name] = {}
                object_data = exp_data[algorithm['label']][object_name]

                gt_pose_all = self.data['gt'][object_name]
                gt_pose = None
                pose = object_data['pose']

                # Check if the length of ground truth and pose is the same
                if gt_pose_all.shape != pose.shape:
                    # if not check if a list of indexes is provided
                    if not 'indexes' in object_data:
                        print('Algorithm ' + algorithm['name'] + ' (label ' + algorithm['label'] + ')' +\
                              ' provides ' + str(pose.shape) + ' matrix as pose while the ground truth has shape ' + \
                              str(gt_pose_all.shape) + '. However, a list of indexes for the poses has not been provided.' + \
                              ' Cannot continue.')
                    pose_indexes = object_data['indexes']
                    gt_pose = gt_pose_all[pose_indexes, :]
                else:
                    gt_pose = gt_pose_all

                for metric_name in self.metrics:
                    exp_results[algorithm['label']][object_name][metric_name] = self.metrics[metric_name].evaluate(gt_pose, pose)


    def render(self, head):
        """Render results using head 'head'."""

        renders = {}

        if head == 'markdown':
            renderer = ResultsMarkdownRenderer()
        elif head == 'latex':
            renderer = ResultsLaTeXRenderer()

        for result_name in self.results:
            renders[result_name] = renderer.render(result_name, self.results[result_name], self.objects, self.experiments)

        return renderer.extension, renders


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--metric-name', dest = 'metric_name', type = str, required = True)
    parser.add_argument('--experiment-name', dest = 'experiment_name', type = str, required = False)
    parser.add_argument('--use-subset', dest = 'use_subset', type = str, required = False)
    parser.add_argument('--output_head', dest = 'output_head', type = str, required = False)
    parser.add_argument('--output_path', dest = 'output_path', type = str, required = False)

    options = parser.parse_args()

    evaluator = Evaluator(options.metric_name, options.experiment_name, options.output_head, options.output_path)


if __name__ == '__main__':
    main()
