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
from pyquaternion import Quaternion


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
            # 'ADD' : self.add,
            # 'ADI' : self.adi
        }

        if name not in self.mapping:
            self.log('__init__', 'Metric ' + name + ' does not exist.', starter = True)
            exit(1)


    def log(self, method, msg, starter = False):
        """Log function."""
        if starter:
            print('Metric::' + method + '()')
        print('    info: ' + msg)


    def evaluate(self, reference, signal, key_frames = None, **kwargs):
        """Evaluate the metric on the signal according to its name.

        If key_frames is not None, the evaluation is restricted to the frames in key_frames."""

        if key_frames is not None:
            tmp_ref = copy.deepcopy(reference)
            tmp_sig = copy.deepcopy(signal)
            reference = []
            signal = []

            for i in key_frames:
                reference.append(tmp_ref[i, :])
                signal.append(tmp_sig[i, :])

            reference = numpy.array(reference)
            signal = numpy.array(signal)

        return self.mapping[self.name](reference, signal, *kwargs)


    def rmse_cartesian(self, reference, signal, **kwargs):
        """Evaluate the RMSE cartesian error for the specified coordinate."""

        index_mapping = {'x' : 0, 'y' : 1, 'z' : 2}
        index = index_mapping[kwargs['coord']]

        # Expressed in centimeters
        return numpy.linalg.norm(reference[:, index] - signal[:, index]) / numpy.sqrt(signal.shape[0]) * 100.0


    def rmse_cartesian_x(self, reference, signal, **kwargs):
        """Evaluate the RMSE cartesian error for the x coordinate."""

        return self.rmse_cartesian(reference, signal, coord = 'x')


    def rmse_cartesian_y(self, reference, signal, **kwargs):
        """Evaluate the RMSE cartesian error for the y coordinate."""

        return self.rmse_cartesian(reference, signal, coord = 'y')


    def rmse_cartesian_z(self, reference, signal, **kwargs):
        """Evaluate the RMSE cartesian error for the y coordinate."""

        return self.rmse_cartesian(reference, signal, coord = 'z')


    def rmse_angular(self, reference, signal, **kwargs):
        """Evaluate the RMSE angular error for the orientation.

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

        return numpy.linalg.norm(error) / numpy.sqrt(error.shape[0])
