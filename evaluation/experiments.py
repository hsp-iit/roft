#===============================================================================
#
# Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT)
#
# This software may be modified and distributed under the terms of the
# GPL-2+ license. See the accompanying LICENSE file for details.
#
#===============================================================================


class Experiments():

    def __init__(self):
        """Constructor."""

        self._experiments = {}

        self.add_algorithm_to_experiment('exp_ycbvs_DOPE', 'ours', masks_train_set = 'mrcnn_ycbv_bop_pbr', nvof_set = 'nvof_2_slow', gt_masks = False, dataset = 'ycbv_synthetic')
        self.add_algorithm_to_experiment('exp_ycbvs_DOPE', 'poserbpf', particles = 200, fps = 7, reinit = True, reinit_from = 'dope', dataset = 'ycbv_synthetic')
        self.add_algorithm_to_experiment('exp_ycbvs_DOPE', 'se3tracknet', reinit = True, reinit_from = 'dope', reinit_fps = 5, dataset = 'ycbv_synthetic')

        self.add_algorithm_to_experiment('exp_ycbvs_real', 'ours', masks_train_set = 'mrcnn_ycbv_bop_pbr', nvof_set = 'nvof_2_slow', gt_masks = False, dataset = 'ycbv_synthetic')
        self.add_algorithm_to_experiment('exp_ycbvs_real', 'poserbpf', particles = 200, fps = 7, reinit = False, reinit_from = '', dataset = 'ycbv_synthetic')
        self.add_algorithm_to_experiment('exp_ycbvs_real', 'se3tracknet', reinit = True, reinit_from = 'dope', reinit_fps = 5, dataset = 'ycbv_synthetic')

        self.add_algorithm_to_experiment('exp_ycbvs_real_rbpf50', 'ours', masks_train_set = 'mrcnn_ycbv_bop_pbr', nvof_set = 'nvof_2_slow', gt_masks = False, dataset = 'ycbv_synthetic')
        self.add_algorithm_to_experiment('exp_ycbvs_real_rbpf50', 'poserbpf', particles = 50, fps = 15, reinit = False, reinit_from = '', dataset = 'ycbv_synthetic')
        self.add_algorithm_to_experiment('exp_ycbvs_real_rbpf50', 'se3tracknet', reinit = True, reinit_from = 'dope', reinit_fps = 5, dataset = 'ycbv_synthetic')

        self.add_algorithm_to_experiment('exp_ycbvs_ideal', 'ours', masks_train_set = 'mrcnn_ycbv_bop_pbr', nvof_set = 'nvof_2_slow', gt_masks = False, dataset = 'ycbv_synthetic')
        self.add_algorithm_to_experiment('exp_ycbvs_ideal', 'poserbpf', particles = 200, fps = 30, reinit = True, reinit_from = 'rbpf', dataset = 'ycbv_synthetic')
        self.add_algorithm_to_experiment('exp_ycbvs_ideal', 'se3tracknet', reinit = True, reinit_from = 'dope', reinit_fps = 5, dataset = 'ycbv_synthetic')

    @property
    def experiments(self):
        """Return all the experiments."""
        return self._experiments


    def __call__(self, experiment_name):
        """Return an experiment given the name."""

        return self._experiments[experiment_name]


    def add_algorithm_to_experiment(self, experiment_name, algorithm_name, **algorithm_conf):
        """Add a new algorithm to the a named experiment."""

        if experiment_name not in self._experiments:
            self._experiments[experiment_name] = []

        # if a label is not provided, it is set equal to the algorithm name
        algorithm_label = algorithm_name
        if 'label' in algorithm_conf:
            algorithm_label = algorithm_conf['label']

        self._experiments[experiment_name].append\
        (
            {
                'label' : algorithm_label,
                'name' : algorithm_name,
                'config' : algorithm_conf
            }
        )


    def print_experiment(self, experiment_name):
        """Print experiment configuration."""

        string = 'Experiment name: ' + experiment_name + '\n\r'

        experiment = self._experiments[experiment_name]

        for algorithm in experiment:
            string += '    algorithm: ' + algorithm['name'] + ', '
            string += 'label: ' + algorithm['label'] + ', '
            string += 'config: '

            for key in algorithm['config']:
                string += key + '=' + str(algorithm['config'][key]) + ', '

            string += '\n\r'

        return string


    def __str__(self):
        """Override string to be printed for this object."""

        string = ''

        for i, experiment_name in enumerate(self._experiments):
            string += self.print_experiment(experiment_name)
            if i != len(self.experiments) - 1:
                string += '\n\r'

        return string


def main():
    experiments = Experiments()
    print(experiments)


if __name__ == '__main__':
    main()
