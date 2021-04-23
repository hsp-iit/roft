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

        # synthetic YCB-Video
        self.add_algorithm_to_experiment('exp_ycbvs_real', 'ours', masks_set = 'mrcnn_ycbv_bop_pbr', of_set = 'nvof_2_slow', pose_set = 'dope', dataset = 'ycbv_synthetic')
        self.add_algorithm_to_experiment('exp_ycbvs_real', 'poserbpf', particles = 200, fps = 7, init_from = 'Dope', reinit = False, reinit_from = '', dataset = 'ycbv_synthetic')
        self.add_algorithm_to_experiment('exp_ycbvs_real', 'se3tracknet', reinit = True, reinit_from = 'dope', reinit_fps = 5, dataset = 'ycbv_synthetic')

        self.add_algorithm_to_experiment('exp_ycbvs_real_50', 'ours', masks_set = 'mrcnn_ycbv_bop_pbr', of_set = 'nvof_2_slow', pose_set = 'dope', dataset = 'ycbv_synthetic')
        self.add_algorithm_to_experiment('exp_ycbvs_real_50', 'poserbpf', particles = 50, fps = 15, init_from = 'Dope', reinit = False, reinit_from = '', dataset = 'ycbv_synthetic')
        self.add_algorithm_to_experiment('exp_ycbvs_real_50', 'se3tracknet', reinit = True, reinit_from = 'dope', reinit_fps = 5, dataset = 'ycbv_synthetic')

        # HO-3D
        self.add_algorithm_to_experiment('exp_ho3d_real', 'ours', masks_set = 'mrcnn_ycbv_bop_pbr', of_set = 'nvof_2_slow', pose_set = 'dope', dataset = 'ho3d')
        self.add_algorithm_to_experiment('exp_ho3d_real', 'poserbpf', particles = 200, fps = 7, init_from = 'Dope', reinit = False, reinit_from = '', dataset = 'ho3d')
        self.add_algorithm_to_experiment('exp_ho3d_real', 'se3tracknet', reinit = True, reinit_from = 'dope', reinit_fps = 5, dataset = 'ho3d')

        self.add_algorithm_to_experiment('exp_ho3d_real_50', 'ours', masks_set = 'mrcnn_ycbv_bop_pbr', of_set = 'nvof_2_slow', pose_set = 'dope', dataset = 'ho3d')
        self.add_algorithm_to_experiment('exp_ho3d_real_50', 'poserbpf', particles = 50, fps = 15, init_from = 'Dope', reinit = False, reinit_from = '', dataset = 'ho3d')
        self.add_algorithm_to_experiment('exp_ho3d_real_50', 'se3tracknet', reinit = True, reinit_from = 'dope', reinit_fps = 5, dataset = 'ho3d')

        # HO-3D ablations
        self.add_algorithm_to_experiment('exp_ho3d_ablation_0', 'ours', masks_set = 'mrcnn_ycbv_bop_pbr', of_set = 'nvof_2_slow', pose_set = 'dope', dataset = 'ho3d')
        self.add_algorithm_to_experiment('exp_ho3d_ablation_0', 'ours', label = 'ours_gt_mask', masks_set = 'gt', of_set = 'nvof_2_slow', pose_set = 'dope', dataset = 'ho3d')

        self.add_algorithm_to_experiment('exp_ho3d_ablation_0_soa', 'ours', masks_set = 'mrcnn_ycbv_bop_pbr', of_set = 'nvof_2_slow', pose_set = 'dope', dataset = 'ho3d')
        self.add_algorithm_to_experiment('exp_ho3d_ablation_0_soa', 'ours', label = 'ours_gt_mask', masks_set = 'gt', of_set = 'nvof_2_slow', pose_set = 'dope', dataset = 'ho3d')
        self.add_algorithm_to_experiment('exp_ho3d_ablation_0_soa', 'ours', label = 'ours_ho3d_mask', masks_set = 'mrcnn_ho3d', of_set = 'nvof_2_slow', pose_set = 'dope', dataset = 'ho3d')
        self.add_algorithm_to_experiment('exp_ho3d_ablation_0_soa', 'poserbpf', particles = 50, fps = 15, init_from = 'Dope', reinit = False, reinit_from = '', dataset = 'ho3d')
        self.add_algorithm_to_experiment('exp_ho3d_ablation_0_soa', 'se3tracknet', reinit = True, reinit_from = 'dope', reinit_fps = 5, dataset = 'ho3d')

        # self.add_algorithm_to_experiment('exp_ho3d_ablation_1', 'ours', masks_set = 'mrcnn_ycbv_bop_pbr', of_set = 'nvof_2_slow', pose_set = 'dope', dataset = 'ho3d')
        # self.add_algorithm_to_experiment('exp_ho3d_ablation_1', 'ours', label = 'ours_gt_pose', masks_set = 'mrcnn_ycbv_bop_pbr', of_set = 'nvof_2_slow', pose_set = 'gt', dataset = 'ho3d')

        # self.add_algorithm_to_experiment('exp_ho3d_ablation_2', 'ours', masks_set = 'mrcnn_ycbv_bop_pbr', of_set = 'nvof_2_slow', pose_set = 'dope', dataset = 'ho3d')
        # self.add_algorithm_to_experiment('exp_ho3d_ablation_2', 'ours', label = 'ours_gt', masks_set = 'gt', of_set = 'nvof_2_slow', pose_set = 'gt', dataset = 'ho3d')


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

        if 'excluded_objects' not in algorithm_conf:
            algorithm_conf['excluded_objects'] = []

        self._experiments[experiment_name].append\
        (
            {
                'label' : algorithm_label,
                'name' : algorithm_name,
                'config' : algorithm_conf
            }
        )


    def exclude_object_from_evaluation(self, experiment_name, object_name):
        """Exclude object from the evaluation."""

        experiment = self._experiments[experiment_name]

        for algorithm in experiment:
            if 'excluded_objects' not in algorithm['config']:
                algorithm['config']['excluded_objects'] = []

            algorithm['config']['excluded_objects'].append(object_name)


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
