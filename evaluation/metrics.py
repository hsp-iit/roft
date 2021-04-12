#===============================================================================
#
# Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT)
#
# This software may be modified and distributed under the terms of the
# GPL-2+ license. See the accompanying LICENSE file for details.
#
#===============================================================================

import copy
import numpy
import os
from objects import Objects
from pyquaternion import Quaternion
from third_party.bop_pose_error import add as bop_add
from third_party.bop_pose_error import adi as bop_adi
from third_party.bop_pose_error import VOCap


class Metric():

    def __init__(self, name):
        """Constructor."""

        self.name = name
        self.mapping =\
        {
            'rmse_cartesian_x' : self.rmse_cartesian_x,
            'rmse_cartesian_y' : self.rmse_cartesian_y,
            'rmse_cartesian_z' : self.rmse_cartesian_z,
            'rmse_angular' : self.rmse_angular,
            'error_cartesian_x' : self.error_cartesian_x,
            'error_cartesian_y' : self.error_cartesian_y,
            'error_cartesian_z' : self.error_cartesian_z,
            'error_angular' : self.error_angular,
            'add' : self.add,
            'adi' : self.adi,
            'add-distances' : self.add_distances
        }

        if name not in self.mapping:
            self.log('__init__', 'Metric ' + name + ' does not exist.', starter = True)
            exit(1)

        # Load point clouds for ADD-AUC and ADI-AUC evaluation
        self.auc_points = { object_name : self.load_point_cloud(os.path.join('./YCB_Video_Models/models/', object_name, 'points.xyz')) for object_name in Objects().objects }


    def log(self, method, msg, starter = False):
        """Log function."""
        if starter:
            print('Metric::' + method + '()')
        print('    info: ' + msg)


    def load_point_cloud(self, file_name):

        data = []

        with open(file_name, newline = '') as file_data:
            for row in file_data:
                data.append([float(num_string.rstrip()) for num_string in row.rstrip().split(sep = ' ') if num_string != ''])
        return numpy.array(data)


    def evaluate(self, object_name, indexes, reference, signal):
        """Evaluate the metric on the signal according to its name."""

        return self.mapping[self.name](object_name, indexes, reference, signal)


    def error_cartesian(self, reference, signal, coordinate_name):
        """Evaluate the Cartesian error for the specified coordinate."""

        index_mapping = {'x' : 0, 'y' : 1, 'z' : 2}
        index = index_mapping[coordinate_name]

        # Expressed in centimeters
        return (reference[:, index] - signal[:, index]) * 100.0


    def error_angular(self, object_name, indexes, reference, signal):
        """Evaluate the angular error for the orientation.

        References for the adopted metric available in: https://link.springer.com/article/10.1007/s10851-009-0161-2"""

        def aa_to_r(aa):
            """Axis angle to rotation matrix."""
            q = Quaternion(axis=[aa[0], aa[1], aa[2]], angle=aa[3])

            return q.rotation_matrix

        def r_to_log(r):
            """Rotation matrix to its norm."""
            q = Quaternion(matrix = r)
            axis = q.axis * q.angle

            return numpy.linalg.norm(axis)

        error = numpy.empty(reference.shape[0])

        for i in range(reference.shape[0]):
            R_reference = aa_to_r(reference[i, 3 :])
            R_signal = aa_to_r(signal[i, 3 :])
            R_error = numpy.dot(R_reference, R_signal.transpose())
            error[i] = r_to_log(R_error) * 180.0 / numpy.pi

        return error


    def error_cartesian_x(self, object_name, indexes, reference, signal):
        """Evaluate the Cartesian error for the x coordinate."""

        return self.error_cartesian(reference, signal, 'x')


    def error_cartesian_y(self, object_name, indexes, reference, signal):
        """Evaluate the Cartesian error for the y coordinate."""

        return self.error_cartesian(reference, signal, 'y')


    def error_cartesian_z(self, object_name, indexes, reference, signal):
        """Evaluate the Cartesian error for the y coordinate."""

        return self.error_cartesian(reference, signal, 'z')


    def rmse_cartesian(self, reference, signal, coordinate_name):
        """Evaluate the RMSE Cartesian error for the specified coordinate."""

        error = self.error_cartesian(reference, signal, coordinate_name)

        return numpy.linalg.norm(error) / numpy.sqrt(error.shape[0])


    def rmse_cartesian_x(self, object_name, indexes, reference, signal):
        """Evaluate the RMSE Cartesian error for the x coordinate."""

        return self.rmse_cartesian(reference, signal, 'x')


    def rmse_cartesian_y(self, object_name, indexes, reference, signal):
        """Evaluate the RMSE Cartesian error for the y coordinate."""

        return self.rmse_cartesian(reference, signal, 'y')


    def rmse_cartesian_z(self, object_name, indexes, reference, signal):
        """Evaluate the RMSE Cartesian error for the y coordinate."""

        return self.rmse_cartesian(reference, signal, 'z')


    def rmse_angular(self, object_name, indexes, reference, signal):
        """Evaluate the RMSE angular error for the orientation."""

        error = self.error_angular(object_name, indexes, reference, signal)

        return numpy.linalg.norm(error) / numpy.sqrt(error.shape[0])


    def adi(self, object_name, indexes, reference, signal):
        """Evaluate ADI-AUC."""

        distances, auc = self.auc(object_name, reference, signal, 'adi')

        return auc


    def add(self, object_name, indexes, reference, signal):
        """Evaluate ADD-AUC."""

        distances, auc = self.auc(object_name, reference, signal, 'add')

        return auc


    def add_distances(self, object_name, indexes, reference, signal):
        """Evaluate sorted ADD distances."""

        distances, auc = self.auc(object_name, reference, signal, 'add')

        return numpy.sort(distances)


    def auc(self, object_name, reference, signal, ad_name):
        """Evaluate AUC for the Average Distance metric having name ad_name."""

        distances = []
        for i in range(reference.shape[0]):
            gt_t = reference[i, 0 : 3]
            gt_R = Quaternion(axis = reference[i, 3 : 6], angle = reference[i, 6]).rotation_matrix

            estimate_t = signal[i, 0 : 3]
            estimate_R = Quaternion(axis = signal[i, 3 : 6], angle = signal[i, 6]).rotation_matrix

            if ad_name == 'adi':
                distances.append(bop_adi(estimate_R, estimate_t, gt_R, gt_t, self.auc_points[object_name]))
            elif ad_name == 'add':
                distances.append(bop_add(estimate_R, estimate_t, gt_R, gt_t, self.auc_points[object_name]))

        distances = numpy.array(distances)
        distances_copy = copy.deepcopy(distances)
        threshold = 0.1
        inf_indexes = numpy.where(distances > threshold)[0]
        distances[inf_indexes] = numpy.inf
        sorted_distances = numpy.sort(distances)
        n = len(sorted_distances)
        accuracy = numpy.cumsum(numpy.ones((n, ), numpy.float32)) / n

        return distances_copy, VOCap(sorted_distances, accuracy) * 100.0
