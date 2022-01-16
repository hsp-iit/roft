#===============================================================================
#
# Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT)
#
# This software may be modified and distributed under the terms of the
# GPL-2+ license. See the accompanying LICENSE file for details.
#
#===============================================================================

import argparse
import cv2
import json
import numpy
import os
import yaml
import sys
from PIL import Image
from PIL import ImageDraw

# Import DOPE classes
sys.path.insert(0, sys.argv[6])
from src.dope.inference.cuboid import Cuboid3d
from src.dope.inference.cuboid_pnp_solver import CuboidPNPSolver
from src.dope.inference.detector import ObjectDetector, ModelData


class DOPEInference():

    def __init__(self, options):
        """Constructor."""

        self.dataset_name = options.dataset_name
        self.dataset_path = options.dataset_path
        self.file_heading_zeros = options.file_heading_zeros
        self.object_name = options.object_name
        self.output_path = options.output_path
        self.rgb_format = options.rgb_format

        # Set the desired GPU id
        os.environ['CUDA_DEVICE_ORDER'] = 'PCI_BUS_ID'
        os.environ['CUDA_VISIBLE_DEVICES'] = str(options.gpu_id)

        # Object name to video ids map for ho3d
        # ids with heading 100 are from the training set
        self.ho3d_video_ids =\
        {
            '003_cracker_box' : [0, 1, 2],
            '004_sugar_box' : [0, 1, 2, 3, 4],
            '006_mustard_bottle' : [0, 1, 2, 3],
            '010_potted_meat_can' : [100, 101, 102, 103, 104],
        }

        # Object-related information
        self.objects_info = \
        {
            '003_cracker_box' : { 'name' : 'cracker' , 'weight' : 'cracker_60'},
            '004_sugar_box' : { 'name' : 'sugar', 'weight' : 'sugar_60' },
            '005_tomato_soup_can' : { 'name' : 'soup', 'weight' : 'soup_60' },
            '006_mustard_bottle' : { 'name' : 'mustard', 'weight' : 'mustard_60' },
            '009_gelatin_box' : { 'name' : 'gelatin', 'weight' : 'gelatin_60' },
            '010_potted_meat_can' : { 'name' : 'meat', 'weight' : 'meat_20' }
        }

        # Load general DOPE configuration
        config_path = os.path.join(options.dope_path, './config', 'config_pose.yaml')
        self.params = None
        self.config = lambda: None
        with open(config_path, 'r') as in_file:
            self.params = yaml.load(in_file)

        self.config.mask_edges = 1
        self.config.mask_faces = 1
        self.config.vertex = 1
        self.config.threshold = 0.5
        self.config.softmax = 1000
        self.config.thresh_angle = self.params['thresh_angle']
        self.config.thresh_map = self.params['thresh_map']
        self.config.sigma = self.params['sigma']
        self.config.thresh_points = self.params['thresh_points']

        # Load camera configuration
        #
        # The structure of the camera matrix is:
        #     camera_matrix[0, 0] = fx
        #     camera_matrix[1, 1] = fy
        #     camera_matrix[0, 2] = cx
        #     camera_matrix[1, 2] = cy
        #     camera_matrix[2, 2] = 1.0
        #
        self.camera_matrix = numpy.zeros((3,3))
        if self.dataset_name == 'ho3d':
            pass # as this is done at video sequence level
        elif self.dataset_name == 'ycbv':
            pass # to be done
        elif self.dataset_name == 'fastycb':
            self.camera_matrix[0,0] = 614.7142806307731
            self.camera_matrix[1,1] = 614.7142806307731
            self.camera_matrix[0,2] = 320.0
            self.camera_matrix[1,2] = 240.0
        elif self.dataset_name == 'fastycb_qual':
            self.camera_matrix[0,0] = 460.8377380371095
            self.camera_matrix[1,1] = 460.8377380371095
            self.camera_matrix[0,2] = 329.501647949219
            self.camera_matrix[1,2] = 246.10359700520797
        self.camera_matrix[2,2] = 1.0
        self.camera_distortion = numpy.zeros((4,1))

        # Load model
        self.object_info = self.objects_info[self.object_name]
        self.model = ModelData(self.object_info['name'], os.path.join(options.dope_path, './weights/', self.object_info['weight'] + '.pth'))
        self.model.load_net_model()

        # Load PnP solver
        if self.dataset_name == 'ho3d':
            pass # as this is done at video sequence level
        elif self.dataset_name == 'fastycb_qual' or self.dataset_name == 'fastycb':
            self.pnp_solver = CuboidPNPSolver\
            (
                self.object_info['name'],
                self.camera_matrix,
                Cuboid3d(self.params['dimensions'][self.object_info['name']]),
                dist_coeffs = self.camera_distortion
            )

        # Load colors for prediction drawing
        self.draw_colors = tuple(self.params['draw_colors'][self.object_info['name']])


    def draw_line(self, draw, point1, point2, line_color, line_width):
        """Draws line on image."""

        if not point1 is None and point2 is not None:
            draw.line([point1, point2], fill = line_color, width = line_width)


    def draw_dot(self, draw, point, point_color, point_radius):
        """Draws dot (filled circle) on image."""

        if point is not None:
            xy =\
            [
                point[0] - point_radius,
                point[1] - point_radius,
                point[0] + point_radius,
                point[1] + point_radius
            ]
            draw.ellipse(xy, fill = point_color, outline = point_color)


    def draw_cube(self, draw, points, color = (255, 0, 0)):
        """Draws cube with a thick solid line across the front top edge and an X on the top face."""

        line_width_for_drawing = 2

        # Draw front
        self.draw_line(draw, points[0], points[1], color, line_width_for_drawing)
        self.draw_line(draw, points[1], points[2], color, line_width_for_drawing)
        self.draw_line(draw, points[3], points[2], color, line_width_for_drawing)
        self.draw_line(draw, points[3], points[0], color, line_width_for_drawing)

        # Draw back
        self.draw_line(draw, points[4], points[5], color, line_width_for_drawing)
        self.draw_line(draw, points[6], points[5], color, line_width_for_drawing)
        self.draw_line(draw, points[6], points[7], color, line_width_for_drawing)
        self.draw_line(draw, points[4], points[7], color, line_width_for_drawing)

        # Draw sides
        self.draw_line(draw, points[0], points[4], color, line_width_for_drawing)
        self.draw_line(draw, points[7], points[3], color, line_width_for_drawing)
        self.draw_line(draw, points[5], points[1], color, line_width_for_drawing)
        self.draw_line(draw, points[2], points[6], color, line_width_for_drawing)

        # Draw dots
        self.draw_dot(draw, points[0], point_color=color, point_radius=4)
        self.draw_dot(draw, points[1], point_color=color, point_radius=4)

        # Draw x on the top
        self.draw_line(draw, points[0], points[5], color, line_width_for_drawing)
        self.draw_line(draw, points[1], points[4], color, line_width_for_drawing)


    def process(self):
        """Process the data."""

        path = None
        output_path = None

        if self.dataset_name == 'ho3d':
            # More than one sequence for the same object
            for video_id in self.ho3d_video_ids[self.object_name]:

                video_name = self.object_name + '_' + str(video_id)

                # Compose input path
                path = os.path.join(self.dataset_path, video_name)

                # Compose output path and create it
                output_path = os.path.join(self.output_path, video_name)

                # Setup camera matrix
                camera_matrix_path = os.path.join(self.dataset_path, video_name, 'cam_K.json')
                with open(camera_matrix_path, 'r') as in_file:
                    params = json.load(in_file)
                    self.camera_matrix[0,0] = params['fx']
                    self.camera_matrix[1,1] = params['fy']
                    self.camera_matrix[0,2] = params['cx']
                    self.camera_matrix[1,2] = params['cy']

                self.pnp_solver = CuboidPNPSolver\
                (
                    self.object_info['name'],
                    self.camera_matrix,
                    Cuboid3d(self.params['dimensions'][self.object_info['name']]),
                    dist_coeffs = self.camera_distortion
                )

                self.process_sequence(path, output_path)

        elif self.dataset_name == 'fastycb':
            # Compose input path
            path = os.path.join(self.dataset_path, self.object_name)

            # Compose output path and create it
            output_path = os.path.join(self.output_path, self.object_name)

            self.process_sequence(path, output_path)

        elif self.dataset_name == 'fastycb_qual':
            # Compose input path
            path = os.path.join(self.dataset_path, self.object_name + '_real')

            # Compose output path and create it
            output_path = os.path.join(self.output_path, self.object_name + '_real')

            self.process_sequence(path, output_path)


    def process_sequence(self, path, output_path):
        """Process one sequence and save to output path."""

        path = os.path.join(path, 'rgb')
        output_path = os.path.join(output_path, 'dope')
        debugging_path = os.path.join(output_path, 'debugging')

        os.makedirs(output_path, exist_ok = True)
        os.makedirs(debugging_path, exist_ok = True)

        # Open output file for DOPE predictions
        pose_output_path = os.path.join(output_path, 'poses.txt')
        pose_file = open(pose_output_path, 'w')

        frame_index = -1
        while True:
            frame_index += 1

            frame_path = os.path.join(path, str(frame_index).zfill(self.file_heading_zeros) + '.' + self.rgb_format)
            print('Processing ' + frame_path)

            # Load image from file
            img = cv2.imread(frame_path, cv2.IMREAD_COLOR)
            if img is None:
                # No more sequential images found, processing complete.
                break

            img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

            if self.dataset_name == 'fastycb' or self.dataset_name == 'fastycb_qual':
                # These frames are @ 1280x720, hence they are zeropadded, extended and resized
                # to the original resolution required for DOPE
                img_extended = numpy.zeros((960, 1280, 3), dtype = numpy.uint8)
                img_extended[120:720 + 120, :, :] = img
                img = cv2.resize(img_extended, (640, 480))

            img_copy = img.copy()
            img_array = Image.fromarray(img_copy)
            draw = ImageDraw.Draw(img_array)

            results = ObjectDetector.detect_object_in_image(self.model.net, self.pnp_solver, img, self.config)

            # Process results, only the first valid detection is considered
            valid_result = False
            for i, result in enumerate(results):

                if valid_result == True:
                    break

                # Make sure that field 'location' is valid
                if result['location'] is not None:

                    valid_result = True

                    position = result['location']
                    quaternion = result['quaternion']

                    # Position is tranformed in meters
                    pose_file.write\
                    (
                        format(position[0] * 0.01, '.6f') + ' ' +
                        format(position[1] * 0.01, '.6f') + ' ' +
                        format(position[2] * 0.01, '.6f') + ' ' +
                        format(quaternion.axis[0], '.6f') + ' ' +
                        format(quaternion.axis[1], '.6f') + ' ' +
                        format(quaternion.axis[2], '.6f') + ' ' +
                        format(quaternion.angle, '.6f') + '\n'
                    )

                    # Draw detected pose for debugging purposes
                    points = []
                    for pair in result['projected_points']:
                        points.append(tuple(pair))
                    self.draw_cube(draw, points, self.draw_colors)


            if valid_result == False:
                # Mark this pose as an invalid frame
                print('Frame produced an invalid pose.')
                pose_file.write('0.0 0.0 0.0 0.0 0.0 0.0 0.0\n')

            output_frame_path = os.path.join(debugging_path, str(frame_index).zfill(self.file_heading_zeros) + '.jpg')
            img_output = numpy.array(img_array)
            img_output = cv2.cvtColor(img_output, cv2.COLOR_RGB2BGR)
            cv2.imwrite(output_frame_path, img_output)

        pose_file.close()


def main():
    parser = argparse.ArgumentParser(description = 'DOPE inference.')
    parser.add_argument('--dataset-name', dest = 'dataset_name', type = str, required = True)
    parser.add_argument('--dataset-path', dest = 'dataset_path', type = str, required = True)
    parser.add_argument('--dope-path', dest = 'dope_path', type = str, required = True)
    parser.add_argument('--gpu-id', dest = 'gpu_id', type = int, required = True)
    parser.add_argument('--file-heading-zeros', dest = 'file_heading_zeros', type = int, default = 1, required = False)
    parser.add_argument('--object-name', dest = 'object_name', type = str, required = True)
    parser.add_argument('--output-path', dest = 'output_path', type = str, required = True)
    parser.add_argument('--rgb-format', dest = 'rgb_format', type = str, required = True)

    options = parser.parse_args()

    DOPEInference(options).process()


if __name__ == '__main__':
    main()
