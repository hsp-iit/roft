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

        # FastYCB
        self.add_algorithm_to_experiment('exp_fastycb', 'dope', dataset = 'ycbv_synthetic', simulate_inference = True)
        self.add_algorithm_to_experiment('exp_fastycb', 'ours', masks_set = 'mrcnn_ycbv_bop_pbr', of_set = 'nvof_1_slow', pose_set = 'dope', dataset = 'ycbv_synthetic')
        self.add_algorithm_to_experiment('exp_fastycb', 'poserbpf', particles = 50, fps = 15, init_from = 'dope', reinit = False, reinit_from = '', dataset = 'ycbv_synthetic')
        self.add_algorithm_to_experiment('exp_fastycb', 'se3tracknet', init_from = 'dope', reinit = True, reinit_from = 'dope', reinit_fps = 5, dataset = 'ycbv_synthetic')
        self.add_parameters_to_experiment('exp_fastycb', 'thumbnail', {'object' : '003_cracker_box', 'frames' : (565, 572, 576, 580), 'crop' : (390, 60, 1120, 574)})

        self.add_algorithm_to_experiment('exp_fastycb_ablation', 'dope', label = 'dope_ideal', dataset = 'ycbv_synthetic', simulate_inference = False)
        self.add_algorithm_to_experiment('exp_fastycb_ablation', 'dope', label = 'dope_real', dataset = 'ycbv_synthetic', simulate_inference = True)
        self.add_algorithm_to_experiment('exp_fastycb_ablation', 'ours', label = 'ours_gt', masks_set = 'gt', of_set = 'nvof_1_slow', pose_set = 'gt', dataset = 'ycbv_synthetic')
        self.add_algorithm_to_experiment('exp_fastycb_ablation', 'ours', label = 'ours_gt_pose', masks_set = 'mrcnn_ycbv_bop_pbr', of_set = 'nvof_1_slow', pose_set = 'gt', dataset = 'ycbv_synthetic')
        self.add_algorithm_to_experiment('exp_fastycb_ablation', 'ours', label = 'ours_gt_mask', masks_set = 'gt', of_set = 'nvof_1_slow', pose_set = 'dope', dataset = 'ycbv_synthetic')
        self.add_algorithm_to_experiment('exp_fastycb_ablation', 'ours', masks_set = 'mrcnn_ycbv_bop_pbr', of_set = 'nvof_1_slow', pose_set = 'dope', dataset = 'ycbv_synthetic')
        self.add_algorithm_to_experiment('exp_fastycb_ablation', 'ours', label = 'ours_no_posesync', masks_set = 'mrcnn_ycbv_bop_pbr', of_set = 'nvof_1_slow', pose_set = 'dope', dataset = 'ycbv_synthetic', no_posesync = True)
        self.add_algorithm_to_experiment('exp_fastycb_ablation', 'ours', label = 'ours_no_outrej', masks_set = 'mrcnn_ycbv_bop_pbr', of_set = 'nvof_1_slow', pose_set = 'dope', dataset = 'ycbv_synthetic', no_outrej = True)
        self.add_algorithm_to_experiment('exp_fastycb_ablation', 'ours', label = 'ours_no_flowaid', masks_set = 'mrcnn_ycbv_bop_pbr', of_set = 'nvof_1_slow', pose_set = 'dope', dataset = 'ycbv_synthetic', no_flowaid = True)
        self.add_algorithm_to_experiment('exp_fastycb_ablation', 'ours', label = 'ours_no_velocity', masks_set = 'mrcnn_ycbv_bop_pbr', of_set = 'nvof_1_slow', pose_set = 'dope', dataset = 'ycbv_synthetic', no_velocity = True)
        self.add_algorithm_to_experiment('exp_fastycb_ablation', 'ours', label = 'ours_no_pose', masks_set = 'mrcnn_ycbv_bop_pbr', of_set = 'nvof_1_slow', pose_set = 'dope', dataset = 'ycbv_synthetic', no_pose = True)

        self.add_algorithm_to_experiment('exp_fastycb_roft', 'ours', masks_set = 'mrcnn_ycbv_bop_pbr', of_set = 'nvof_1_slow', pose_set = 'dope', dataset = 'ycbv_synthetic')

        self.add_algorithm_to_experiment('exp_fastycb_velocities', 'ours', masks_set = 'gt', of_set = 'nvof_1_slow', pose_set = 'dope', dataset = 'ycbv_synthetic', label = 'ours_gt_mask')
        self.add_algorithm_to_experiment('exp_fastycb_velocities', 'ours', masks_set = 'mrcnn_ycbv_bop_pbr', of_set = 'nvof_1_slow', pose_set = 'dope', dataset = 'ycbv_synthetic', label = 'ours_flowaid')
        self.add_algorithm_to_experiment('exp_fastycb_velocities', 'ours', masks_set = 'mrcnn_ycbv_bop_pbr', of_set = 'nvof_1_slow', pose_set = 'dope', dataset = 'ycbv_synthetic', no_flowaid = True, label = 'ours_no_flowaid')

        self.add_algorithm_to_experiment('exp_fastycb_se3_plain', 'se3tracknet', init_from = 'dope', reinit = False, reinit_from = '', dataset = 'ycbv_synthetic')

        # FastYCB qualitative sequences
        self.add_algorithm_to_experiment('exp_fastycb_qual_roft', 'ours', masks_set = 'mrcnn_ycbv_bop_pbr', of_set = 'nvof_1_slow', pose_set = 'dope', dataset = 'ycbv_real')

        self.add_algorithm_to_experiment('exp_fastycb_qual', 'dope', dataset = 'ycbv_real', simulate_inference = True, label = 'dope')
        self.add_algorithm_to_experiment('exp_fastycb_qual', 'ours', masks_set = 'mrcnn_ycbv_bop_pbr', of_set = 'nvof_1_slow', pose_set = 'dope', dataset = 'ycbv_real')
        self.add_algorithm_to_experiment('exp_fastycb_qual', 'poserbpf', particles = 50, fps = 15, init_from = 'dope', reinit = False, reinit_from = '', dataset = 'ycbv_real')
        self.add_algorithm_to_experiment('exp_fastycb_qual', 'se3tracknet', init_from = 'dope', reinit = True, reinit_from = 'dope', reinit_fps = 5, dataset = 'ycbv_real')
        self.add_parameters_to_experiment('exp_fastycb_qual', 'thumbnail', {'object' : '003_cracker_box', 'frames' : (1280, 1290, 1300, 1310), 'crop' : (0, 0, 730, 514)})

        # HO-3D
        self.add_algorithm_to_experiment('exp_ho3d_roft', 'ours', masks_set = 'mrcnn_ycbv_bop_pbr', of_set = 'nvof_2_slow', pose_set = 'dope', dataset = 'ho3d')

        self.add_algorithm_to_experiment('exp_ho3d_se3_plain', 'se3tracknet', init_from = 'dope', reinit = False, reinit_from = '', dataset = 'ho3d')

        self.add_algorithm_to_experiment('exp_ho3d', 'dope', dataset = 'ho3d', simulate_inference = True)
        self.add_algorithm_to_experiment('exp_ho3d', 'ours', masks_set = 'mrcnn_ycbv_bop_pbr', of_set = 'nvof_2_slow', pose_set = 'dope', dataset = 'ho3d')
        self.add_algorithm_to_experiment('exp_ho3d', 'poserbpf', particles = 50, fps = 15, init_from = 'dope', reinit = False, reinit_from = '', dataset = 'ho3d')
        self.add_algorithm_to_experiment('exp_ho3d', 'se3tracknet', init_from = 'dope', reinit = True, reinit_from = 'dope', reinit_fps = 5, dataset = 'ho3d')

        # HO-3D ablations
        self.add_algorithm_to_experiment('exp_ho3d_ablation', 'ours', masks_set = 'mrcnn_ycbv_bop_pbr', of_set = 'nvof_2_slow', pose_set = 'dope', dataset = 'ho3d')
        self.add_algorithm_to_experiment('exp_ho3d_ablation', 'ours', label = 'ours_gt_mask', masks_set = 'gt', of_set = 'nvof_2_slow', pose_set = 'dope', dataset = 'ho3d')
        self.add_algorithm_to_experiment('exp_ho3d_ablation', 'ours', label = 'ours_gt_pose', masks_set = 'mrcnn_ycbv_bop_pbr', of_set = 'nvof_2_slow', pose_set = 'gt', dataset = 'ho3d')
        self.add_algorithm_to_experiment('exp_ho3d_ablation', 'ours', label = 'ours_gt', masks_set = 'gt', of_set = 'nvof_2_slow', pose_set = 'gt', dataset = 'ho3d')


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


    def add_parameters_to_experiment(self, experiment_name, parameters_group_name, parameters_values):
        """Attach given parameters to all the items in a given experiment."""

        for item in self._experiments[experiment_name]:
            item['config'][parameters_group_name] = parameters_values


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
