#===============================================================================
#
# Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT)
#
# This software may be modified and distributed under the terms of the
# GPL-2+ license. See the accompanying LICENSE file for details.
#
#===============================================================================

import argparse
from data_loader import DataLoader
from objects import Objects
from metrics import Metric


class Evaluator():

    def __init__(self, metric_name):
        """Constructor."""

        algorithms = ['gt', 'ours']
        data = {}

        # Initialize metrics
        self.metrics = {}
        metric_names = []
        if metric_name == 'rmse':
            metric_names = ['rmse_cartesian_' + coord for coord in ['x', 'y', 'z']]
            metric_names.append('rmse_angular')
        elif metric_name == 'ad':
            metric_names = ['add', 'adi']

        self.metrics = { name : Metric(name) for name in metric_names}

        # Load data for all the algorithms
        for algorithm in algorithms:
            loader = DataLoader(algorithm)
            data[algorithm] = loader.load()

        # Load object names
        self.objects = Objects().objects

        # Evaluate
        algorithm = 'ours'
        variant = 'full'
        content = 'pose_estimate_ycb'

        results = {}
        results[algorithm] = {}

        for object_name in self.objects:
            results[algorithm][object_name] = {}

            gt_pose = data['gt'][object_name]
            pose = data[algorithm][variant][object_name][content]

            for metric_name in metric_names:
                results[algorithm][object_name][metric_name] = self.metrics[metric_name].evaluate(gt_pose, pose)

        print(results)

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--metric-name', dest = 'metric_name', type = str, required = True)

    options = parser.parse_args()

    evaluator = Evaluator(options.metric_name)


if __name__ == '__main__':
    main()
