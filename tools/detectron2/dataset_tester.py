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
import detectron2
import os
import random
from dataset import DatasetDescription
from detectron2 import model_zoo
from detectron2.config import get_cfg
from detectron2.data import MetadataCatalog, DatasetCatalog
from detectron2.engine import DefaultPredictor
from detectron2.utils.visualizer import Visualizer
from detectron2.utils.visualizer import ColorMode


def main():

    parser = argparse.ArgumentParser()
    parser.add_argument('--dataset-name', dest = 'dataset_name', type = str, required = True)
    options = parser.parse_args()

    dataset = DatasetDescription(options.dataset_name, './tools/detectron2/' + options.dataset_name + '_dataset_description.pickle', False)
    DatasetCatalog.register(options.dataset_name, lambda : dataset.dataset())
    MetadataCatalog.get(options.dataset_name).set(thing_classes = dataset.class_list())
    metadata = MetadataCatalog.get(options.dataset_name)

    counter = 0
    for d in random.sample(dataset.dataset(), 10):
        im = cv2.imread(d["file_name"])
        visualizer = Visualizer(im[:, :, ::-1], metadata = metadata, scale = 1.0)
        out = visualizer.draw_dataset_dict(d)
        cv2.imwrite('./tools/detectron2/dataset_tester_' +  options.dataset_name + '_' + str(counter) + '.png', out.get_image()[:, :, ::-1])
        counter += 1


if __name__ == '__main__':
    main()
