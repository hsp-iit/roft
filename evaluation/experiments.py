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

        self.experiments = {}

        self.add_algorithm_to_experiment('exp_w_dope_predictions', 'ours', variant = 'full')
        self.add_algorithm_to_experiment('exp_w_dope_predictions', 'se3tracknet', reinit = True, reinit_from = 'dope', reinit_fps = 5)
        self.add_algorithm_to_experiment('exp_w_dope_predictions', 'poserbpf', particles = 200, fps = 7, reinit = False, dope = True)


    def add_algorithm_to_experiment(self, experiment_name, algorithm_name, **algorithm_conf):
        """Add a new algorithm to the a named experiment."""

        if experiment_name not in self.experiments:
            self.experiments[experiment_name] = []

        # if a label is not provided, it is set equal to the algorithm name
        algorithm_label = ''
        if 'label' in algorithm_conf:
            label = algorithm_conf['label']

        self.experiments[experiment_name].append\
        (
            {
                'label' : algorithm_label,
                'name' : algorithm_name,
                'config' : algorithm_conf
            }
        )


    def experiment(self, experiment_name):
        """Return an experiment given the name."""

        return self.experiments[experiment_name]


    def __str__(self):
        """Override string to be printed for this object."""

        string = ''

        for experiment_name in self.experiments:
            string += 'Experiment name: ' + experiment_name + '\n\r'

            experiment = self.experiments[experiment_name]

            for algorithm in experiment:
                string += '    algorithm: ' + algorithm['name'] + ' '
                string += '(config: '

                for key in algorithm['config']:
                    string += key + '=' + str(algorithm['config'][key]) + ', '

                string += ')\n\r'

        return string


def main():
    experiments = Experiments()
    print(experiments)


if __name__ == '__main__':
    main()
