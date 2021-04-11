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

        self.add_algorithm_to_experiment('exp_DOPE', 'ours', variant = 'full')
        self.add_algorithm_to_experiment('exp_DOPE', 'poserbpf', particles = 200, fps = 7, reinit = True, reinit_from = 'dope')
        self.add_algorithm_to_experiment('exp_DOPE', 'se3tracknet', reinit = True, reinit_from = 'dope', reinit_fps = 5)

        self.add_algorithm_to_experiment('exp_real', 'ours', variant = 'full')
        self.add_algorithm_to_experiment('exp_real', 'poserbpf', particles = 200, fps = 7, reinit = False, reinit_from = '')
        self.add_algorithm_to_experiment('exp_real', 'se3tracknet', reinit = False, reinit_from = '', reinit_fps = -1)

        self.add_algorithm_to_experiment('exp_real_rbpf50', 'ours', variant = 'full')
        self.add_algorithm_to_experiment('exp_real_rbpf50', 'poserbpf', particles = 50, fps = 15, reinit = False, reinit_from = '')
        self.add_algorithm_to_experiment('exp_real_rbpf50', 'se3tracknet', reinit = False, reinit_from = '', reinit_fps = -1)

        self.add_algorithm_to_experiment('exp_ideal', 'ours', variant = 'full')
        self.add_algorithm_to_experiment('exp_ideal', 'poserbpf', particles = 200, fps = 30, reinit = False, reinit_from = '')
        self.add_algorithm_to_experiment('exp_ideal', 'se3tracknet', reinit = False, reinit_from = '', reinit_fps = -1)

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
            label = algorithm_conf['label']

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

        for experiment_name in self._experiments:
            string += self.print_experiment(experiment_name)

        return string


def main():
    experiments = Experiments()
    print(experiments)


if __name__ == '__main__':
    main()
