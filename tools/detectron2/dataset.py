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

        if self.do_generate:
            self.generate()
            self.save()
        else:
            self.load(self.path)

        self.classes = ['002', '003', '004', '005', '006', '007', '008', '009', '010', '011', '019', '021', '024', '025', '035', '036', '037', '040', '051', '052', '061']


    def class_list(self):
        """Return the a list of the classes."""

        return self.classes


    def dataset(self):
        """Return the dataset itself."""

        return self.dataset_dicts


    def generate(self):
        """Generate the dataset description."""

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

                    mask_path = os.path.abspath(sub + '/mask_visib/' + str(i).zfill(6) + '_' + str(j).zfill(6) +  '.png')
                    mask = cv2.imread(mask_path, cv2.IMREAD_GRAYSCALE) / 255
                    mask = numpy.uint8(mask)

                    obj =\
                    {
                        'bbox' : [bbox[0], bbox[1], bbox[2], bbox[3]],
                        'bbox_mode': BoxMode.XYWH_ABS,
                        'segmentation' : pycocotools.mask.encode(numpy.asarray(mask, order='F')),
                        'category_id' : object_id
                    }

                    annotations.append(obj)

                record['annotations'] = annotations
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
