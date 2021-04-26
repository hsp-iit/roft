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
from dataset import DatasetDescription
from dataset_mapper import Mapper
from detectron2 import model_zoo
from detectron2.config import get_cfg
from detectron2.data import DatasetCatalog, build_detection_train_loader
from detectron2.data import detection_utils as utils
from detectron2.engine import DefaultTrainer


def main():

    parser = argparse.ArgumentParser()
    parser.add_argument('--dataset-name', dest = 'dataset_name', type = str, required = True)
    options = parser.parse_args()

    dataset = DatasetDescription(options.dataset_name, './tools/detectron2/' + options.dataset_name + '_dataset_description.pickle', False)
    DatasetCatalog.register(options.dataset_name, lambda : dataset.dataset())

    cfg = get_cfg()
    cfg.merge_from_file(model_zoo.get_config_file('COCO-InstanceSegmentation/mask_rcnn_R_50_FPN_3x.yaml'))
    cfg.DATASETS.TRAIN = (options.dataset_name,)
    cfg.INPUT.MASK_FORMAT = 'bitmask'


    counter = 0
    train_data_loader = build_detection_train_loader(cfg, mapper = Mapper(cfg, True))
    for batch in train_data_loader:
        for per_image in batch:
            # Pytorch tensor is in (C, H, W) format
            img = per_image["image"].permute(1, 2, 0).cpu().detach().numpy()
            img = utils.convert_image_to_rgb(img, cfg.INPUT.FORMAT)

            # visualizer = Visualizer(img, metadata=metadata, scale=scale)
            # target_fields = per_image["instances"].get_fields()
            # labels = [metadata.thing_classes[i] for i in target_fields["gt_classes"]]
            # vis = visualizer.overlay_instances\
            # (
            #     labels=labels,
            #     boxes=target_fields.get("gt_boxes", None),
            #     masks=target_fields.get("gt_masks", None),
            #     keypoints=target_fields.get("gt_keypoints", None),
            # )
            img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            cv2.imwrite('./tools/detectron2/dataset_tester_' +  options.dataset_name + '_' + str(counter) + '.png', img)

            counter += 1
            if counter > 20:
                exit(0)


if __name__ == '__main__':
    main()
