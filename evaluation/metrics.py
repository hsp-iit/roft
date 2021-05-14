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
            'rmse_cartesian_3d' : self.rmse_cartesian_3d,
            'rmse_cartesian_x' : self.rmse_cartesian_x,
            'rmse_cartesian_y' : self.rmse_cartesian_y,
            'rmse_cartesian_z' : self.rmse_cartesian_z,
            'rmse_angular' : self.rmse_angular,
            'rmse_linear_velocity' : self.rmse_linear_velocity,
            'rmse_angular_velocity' : self.rmse_angular_velocity,
            'max_linear_velocity' : self.max_linear_velocity,
            'max_angular_velocity' : self.max_angular_velocity,
            'add' : self.add,
            'adi' : self.adi,
            'time' : self.time,
            'excess_33_ms' : self.time_excess_33_ms
        }

        if name not in self.mapping:
            self.log('__init__', 'Metric ' + name + ' does not exist.', starter = True)
            exit(1)

        # Load point clouds for ADD-AUC and ADI-AUC evaluation
        self.auc_points = { object_name : self.load_point_cloud(os.path.join('./YCB_Video_Models/models/', object_name, 'points.xyz')) for object_name in Objects().objects['models']}


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


    def make_union_objects(self, signal):
        """Given a dictionary of signal for several objects, return the union of those signals."""

        tmp_signal = copy.deepcopy(signal)
        names = [name for name in tmp_signal]
        signal = tmp_signal[names[0]]

        for i in range(1, len(names)):
            signal = numpy.concatenate((signal, tmp_signal[names[i]]), axis = 0)

        return signal


    def evaluate(self, object_name, reference, signal, time):
        """Evaluate the metric on the signal according to its name."""

        return self.mapping[self.name](object_name, reference, signal, time)


    def error_cartesian(self, object_name, reference, signal, coordinate_name):
        """Evaluate the Cartesian error for the specified coordinate."""

        index_mapping = {'x' : 0, 'y' : 1, 'z' : 2}
        index = index_mapping[coordinate_name]

        if object_name == 'ALL':
            reference = self.make_union_objects(reference)
            signal = self.make_union_objects(signal)

        # Expressed in centimeters
        return (reference[:, index] - signal[:, index]) * 100.0


    def error_cartesian_3d(self, object_name, reference, signal):
        """Evaluate the 3D Cartesian error."""

        if object_name == 'ALL':
            reference = self.make_union_objects(reference)
            signal = self.make_union_objects(signal)

        error = (reference[:, 0:3] - signal[:, 0:3]) * 100.0

        return numpy.linalg.norm(error, axis = 1)


    def error_angular(self, object_name, reference, signal):
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

        if object_name == 'ALL':
            reference = self.make_union_objects(reference)
            signal = self.make_union_objects(signal)

        error = numpy.empty(reference.shape[0])

        for i in range(reference.shape[0]):
            R_reference = aa_to_r(reference[i, 3 :])
            R_signal = aa_to_r(signal[i, 3 :])
            R_error = numpy.dot(R_reference, R_signal.transpose())
            error[i] = r_to_log(R_error) * 180.0 / numpy.pi

        return error


    def error_cartesian_x(self, object_name, reference, signal):
        """Evaluate the Cartesian error for the x coordinate."""

        return self.error_cartesian(object_name, reference, signal, 'x')


    def error_cartesian_y(self, object_name, reference, signal):
        """Evaluate the Cartesian error for the y coordinate."""

        return self.error_cartesian(object_name, reference, signal, 'y')


    def error_cartesian_z(self, object_name, reference, signal):
        """Evaluate the Cartesian error for the y coordinate."""

        return self.error_cartesian(object_name, reference, signal, 'z')


    def error_linear_velocity(self, object_name, reference, signal):
        """Evaluate the linear velocity error."""

        if object_name == 'ALL':
            reference = self.make_union_objects(reference)
            signal = self.make_union_objects(signal)

        error = (reference[:, 0:3] - signal[:, 0:3])

        return numpy.linalg.norm(error, axis = 1)


    def error_angular_velocity(self, object_name, reference, signal):
        """Evaluate the angular velocity error."""

        if object_name == 'ALL':
            reference = self.make_union_objects(reference)
            signal = self.make_union_objects(signal)

        error = (reference[:, 3:6] - signal[:, 3:6]) * 180.0 / numpy.pi

        return numpy.linalg.norm(error, axis = 1)


    def norm_linear_velocity(self, object_name, reference, signal):
        """Evaluate the linear velocity norm."""

        if object_name == 'ALL':
            reference = self.make_union_objects(reference)
            signal = self.make_union_objects(signal)

        return numpy.linalg.norm(reference[:, 0:3], axis = 1)


    def norm_angular_velocity(self, object_name, reference, signal):
        """Evaluate the angular velocity norm."""

        if object_name == 'ALL':
            reference = self.make_union_objects(reference)
            signal = self.make_union_objects(signal)

        return numpy.linalg.norm(reference[:, 3:6], axis = 1)


    def rmse_cartesian(self, object_name, reference, signal, coordinate_name):
        """Evaluate the RMSE Cartesian error for the specified coordinate."""

        error = self.error_cartesian(object_name, reference, signal, coordinate_name)

        return numpy.linalg.norm(error) / numpy.sqrt(error.shape[0])


    def rmse_cartesian_3d(self, object_name, reference, signal, time):
        """Evaluate the RMSE 3D Cartesian error."""

        error = self.error_cartesian_3d(object_name, reference, signal)

        return numpy.linalg.norm(error) / numpy.sqrt(error.shape[0])


    def rmse_cartesian_x(self, object_name, reference, signal, time):
        """Evaluate the RMSE Cartesian error for the x coordinate."""

        return self.rmse_cartesian(object_name, reference, signal, 'x')


    def rmse_cartesian_y(self, object_name, reference, signal, time):
        """Evaluate the RMSE Cartesian error for the y coordinate."""

        return self.rmse_cartesian(object_name, reference, signal, 'y')


    def rmse_cartesian_z(self, object_name, reference, signal, time):
        """Evaluate the RMSE Cartesian error for the y coordinate."""

        return self.rmse_cartesian(object_name, reference, signal, 'z')


    def rmse_angular(self, object_name, reference, signal, time):
        """Evaluate the RMSE angular error for the orientation."""

        error = self.error_angular(object_name, reference, signal)

        return numpy.linalg.norm(error) / numpy.sqrt(error.shape[0])


    def rmse_linear_velocity(self, object_name, reference, signal, time):
        """Evaluate the RMSE linear velocity error."""

        error = self.error_linear_velocity(object_name, reference, signal)

        return numpy.linalg.norm(error) / numpy.sqrt(error.shape[0])


    def rmse_angular_velocity(self, object_name, reference, signal, time):
        """Evaluate the RMSE angular velocity error."""

        error = self.error_angular_velocity(object_name, reference, signal)

        return numpy.linalg.norm(error) / numpy.sqrt(error.shape[0])


    def max_linear_velocity(self, object_name, reference, signal, time):
        """Evaluate the maximum linear velocity."""

        return numpy.max(self.norm_linear_velocity(object_name, reference, signal))


    def max_angular_velocity(self, object_name, reference, signal, time):
        """Evaluate the maximum angular velocity."""

        return numpy.max(self.norm_angular_velocity(object_name, reference, signal)) * 180.0 / numpy.pi


    def adi(self, object_name, reference, signal, time):
        """Evaluate ADI-AUC."""

        distances, auc = self.auc(object_name, reference, signal, 'adi')

        return auc


    def add(self, object_name, reference, signal, time):
        """Evaluate ADD-AUC."""

        distances, auc = self.auc(object_name, reference, signal, 'add')

        return auc


    def add_distances(self, object_name, reference, signal, time):
        """Evaluate sorted ADD distances."""

        distances, auc = self.auc(object_name, reference, signal, 'add')

        return numpy.sort(distances)


    def auc(self, object_name, reference, signal, ad_name):
        """Evaluate AUC for the Average Distance metric having name ad_name."""

        if object_name == 'ALL':
            names = [name for name in signal]
        else:
            names = [object_name]
            signal = { object_name : signal}
            reference = { object_name : reference }

        distances = None
        for i, name in enumerate(names):
            sig = signal[name]
            ref = reference[name]

            dists = []
            for j in range(ref.shape[0]):
                gt_t = ref[j, 0 : 3]
                gt_R = Quaternion(axis = ref[j, 3 : 6], angle = ref[j, 6]).rotation_matrix

                estimate_t = sig[j, 0 : 3]
                estimate_R = Quaternion(axis = sig[j, 3 : 6], angle = sig[j, 6]).rotation_matrix

                if ad_name == 'adi':
                    dists.append(bop_adi(estimate_R, estimate_t, gt_R, gt_t, self.auc_points[name]))
                elif ad_name == 'add':
                    dists.append(bop_add(estimate_R, estimate_t, gt_R, gt_t, self.auc_points[name]))

            if i == 0:
                distances = numpy.array(dists)
            else:
                distances = numpy.concatenate((distances, dists), axis = 0)

        distances_copy = copy.deepcopy(distances)
        threshold = 0.1
        inf_indexes = numpy.where(distances > threshold)[0]
        distances[inf_indexes] = numpy.inf
        sorted_distances = numpy.sort(distances)
        n = len(sorted_distances)
        accuracy = numpy.cumsum(numpy.ones((n, ), numpy.float32)) / n

        return distances_copy, VOCap(sorted_distances, accuracy) * 100.0


    def time(self, object_name, reference, signal, time):
        """Evaluate mean time."""

        execution_time = time
        if object_name == 'ALL':
            execution_time = self.make_union_objects(time)

        return numpy.mean(execution_time[:, 0])


    def time_excess_33_ms(self, object_name, reference, signal, time):
        """Eveluate % of frames taking more than 33 ms to be processed."""

        execution_time = time
        if object_name == 'ALL':
            execution_time = self.make_union_objects(time)

        number_exceeding = numpy.zeros(shape = len(execution_time[:, 0]))
        number_exceeding[execution_time[:, 0] > 33.0] = 1

        excess = sum(number_exceeding)

        return excess
