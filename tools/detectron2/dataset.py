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
import glob
import json
import numpy
import os
import pycocotools
import pickle
from glob import glob
from detectron2.structures import BoxMode
from tqdm import tqdm


class DatasetDescription:
    def __init__(self, name, path, do_generate):
        """Constructor."""

        self.name = name
        self.path = path
        self.do_generate = do_generate

        self.dataset_dicts = []

        self.classes = ['003', '004', '005', '006', '009', '010']
        self.all_classes = ['002', '003', '004', '005', '006', '007', '008', '009', '010', '011', '019', '021', '024', '025', '035', '036', '037', '040', '051', '052', '061']
        self.allowed_classes_idx = [self.all_classes.index(cl) for cl in self.classes]

        # BOP PBR configuration
        self.bop_pbr_actual_classes_idx = { self.allowed_classes_idx[i]: i for i in range(len(self.classes)) }

        # synthetic YCB-Video configuration
        self.synthetic_classes = ['003_cracker_box', '004_sugar_box', '005_tomato_soup_can', '006_mustard_bottle', '009_gelatin_box', '010_potted_meat_can']
        self.synthetic_actual_classes_idx = { name : self.classes.index(name.split('_')[0]) for name in self.synthetic_classes }


        # HO-3D configuration
        self.ho3d_classes = ['003_cracker_box', '004_sugar_box', '005_tomato_soup_can', '006_mustard_bottle', '009_gelatin_box', '010_potted_meat_can']
        self.ho3d_video_ids =\
        {
            '003_cracker_box' : [0, 1, 2],
            '004_sugar_box' : [0, 1, 2, 3, 4],
            '005_tomato_soup_can' : [],
            '006_mustard_bottle' : [0, 1, 2, 3],
            '009_gelatin_box' : [],
            '010_potted_meat_can' : [100, 101, 102, 103, 104],
        }
        self.ho3d_actual_classes_idx = { name : self.classes.index(name.split('_')[0]) for name in self.ho3d_classes }

        if self.do_generate:
            self.generate()
            self.save()
        else:
            self.load(self.path)


    def class_list(self):
        """Return the a list of the classes."""

        return self.classes


    def dataset(self):
        """Return the dataset itself."""

        return self.dataset_dicts


    def generate(self):
        """Generate the dataset description."""

        if self.name == 'ycbv_bop_pbr' or self.name =='pbr_combo':
            self.generate_ycbv_bop_pbr()
        elif self.name == 'fastycb':
            self.generate_fastycb()
        elif self.name == 'ho3d':
            self.generate_ho3d()


    def generate_ycbv_bop_pbr(self):
        """Generate the YCB-Video BOP PBR dataset description."""

        subs = sorted(glob(os.path.join(self.path, '*/')))

        idx = 0
        for sub in subs:
            print('Processing subfolder ' + sub)

            # print('Loading scene_gt.json')
            f = open(os.path.join(sub, 'scene_gt.json'))
            scene_gt = json.load(f)
            f.close()

            # print('Loading scene_gt_info.json')
            f = open(sub + 'scene_gt_info.json')
            scene_gt_info = json.load(f)
            f.close()

            for i in tqdm(range(len(scene_gt))):
                gt_container = scene_gt[str(i)]
                gt_info_container = scene_gt_info[str(i)]

                rgb_path = os.path.abspath(sub + '/rgb/' + str(i).zfill(6) + '.jpg')

                record = {}

                record['file_name'] = rgb_path
                record['image_id'] = idx
                record['height'] = 480
                record['width'] = 640

                annotations = []
                for j in range(len(gt_container)):
                    if gt_info_container[j]['px_count_visib'] == 0:
                        continue

                    bbox = gt_info_container[j]['bbox_visib']

                    object_id = gt_container[j]['obj_id'] - 1
                    if object_id not in self.allowed_classes_idx:
                        continue

                    mask_path = os.path.abspath(sub + '/mask_visib/' + str(i).zfill(6) + '_' + str(j).zfill(6) +  '.png')
                    mask = cv2.imread(mask_path, cv2.IMREAD_GRAYSCALE) / 255
                    mask = numpy.uint8(mask)

                    obj =\
                    {
                        'bbox' : [bbox[0], bbox[1], bbox[2], bbox[3]],
                        'bbox_mode': BoxMode.XYWH_ABS,
                        'segmentation' : pycocotools.mask.encode(numpy.asarray(mask, order='F')),
                        'category_id' : self.bop_pbr_actual_classes_idx[object_id]
                    }

                    annotations.append(obj)

                record['annotations'] = annotations
                self.dataset_dicts.append(record)

                idx += 1


    def generate_ho3d(self):
        """Generate the HO-3D dataset description."""

        idx = 0

        for object_name in self.ho3d_classes:
            short_name = object_name.split('_')[0]

            for video_id in self.ho3d_video_ids[object_name]:

                video_name = object_name + '_' + str(video_id)

                # Compose input path
                path = os.path.join(self.path, video_name)
                rgb_path = os.path.join(path, 'rgb')
                masks_path = os.path.join(path, 'masks', 'gt')

                for i in tqdm(range(len(glob(os.path.join(rgb_path, '*.png'))))):

                    frame_path = os.path.join(rgb_path, str(i) + '.png')
                    mask_path = os.path.join(masks_path, object_name + '_' + str(i) + '.png')

                    mask = cv2.imread(mask_path, cv2.IMREAD_GRAYSCALE)

                    _, thresh = cv2.threshold(mask, 127, 255, cv2.THRESH_BINARY)
                    contours, _ = cv2.findContours(thresh, 1, 2)
                    contours = contours[0]
                    bbox = cv2.boundingRect(contours)

                    mask = mask / 255
                    mask = numpy.uint8(mask)

                    record = {}

                    record['file_name'] = frame_path
                    record['image_id'] = idx
                    record['height'] = 480
                    record['width'] = 640

                    obj =\
                    {
                        'bbox' : [bbox[0], bbox[1], bbox[2], bbox[3]],
                        'bbox_mode': BoxMode.XYWH_ABS,
                        'segmentation' : pycocotools.mask.encode(numpy.asarray(mask, order='F')),
                        'category_id' : self.ho3d_actual_classes_idx[object_name]
                    }

                    record['annotations'] = [obj]

                    self.dataset_dicts.append(record)

                    idx += 1

    def generate_fastycb(self):
        """Generate the FastYCB dataset description."""

        idx = 0

        for object_name in self.synthetic_classes:
            short_name = object_name.split('_')[0]

            # Compose input path
            path = os.path.join(self.path, object_name)
            rgb_path = os.path.join(path, 'rgb')
            masks_path = os.path.join(path, 'masks', 'gt')

            for i in tqdm(range(len(glob(os.path.join(rgb_path, '*.png'))))):

                frame_path = os.path.join(rgb_path, str(i) + '.png')
                mask_path = os.path.join(masks_path, object_name + '_' + str(i) + '.png')

                mask = cv2.imread(mask_path, cv2.IMREAD_GRAYSCALE)

                _, thresh = cv2.threshold(mask, 127, 255, cv2.THRESH_BINARY)
                contours, _ = cv2.findContours(thresh, 1, 2)
                contours = contours[0]
                bbox = cv2.boundingRect(contours)

                mask = mask / 255
                mask = numpy.uint8(mask)

                record = {}

                record['file_name'] = frame_path
                record['image_id'] = idx
                record['height'] = 720
                record['width'] = 1280

                obj =\
                {
                    'bbox' : [bbox[0], bbox[1], bbox[2], bbox[3]],
                    'bbox_mode': BoxMode.XYWH_ABS,
                    'segmentation' : pycocotools.mask.encode(numpy.asarray(mask, order='F')),
                    'category_id' : self.synthetic_actual_classes_idx[object_name]
                }

                record['annotations'] = [obj]

                self.dataset_dicts.append(record)

                idx += 1


    def save(self):
        """Save the dataset description."""

        with open('./tools/detectron2/' + self.name + '_dataset_description.pickle', 'wb') as handle:
            pickle.dump(self.dataset_dicts, handle)


    def load(self, path):
        """Load the dataset description."""

        with open(path, 'rb') as handle:
            self.dataset_dicts = pickle.load(handle)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--dataset-name', dest = 'dataset_name', type = str, required = True)
    parser.add_argument('--dataset-path', dest = 'dataset_path', type = str, required = True)
    parser.add_argument('--generate', type = bool, required = True)

    options = parser.parse_args()

    dataset = DatasetDescription(options.dataset_name, options.dataset_path, options.generate)


if __name__ == '__main__':
    main()
