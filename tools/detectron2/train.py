#===============================================================================
#
# Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT)
#
# This software may be modified and distributed under the terms of the
# GPL-2+ license. See the accompanying LICENSE file for details.
#
#===============================================================================

import argparse
import detectron2
import os
from dataset import DatasetDescription
from dataset_mapper import Mapper
from detectron2 import model_zoo
from detectron2.config import get_cfg
from detectron2.data import DatasetCatalog, build_detection_train_loader
from detectron2.engine import DefaultTrainer


def main():

    parser = argparse.ArgumentParser()
    parser.add_argument('--dataset-name', dest = 'dataset_name', type = str, required = True)
    parser.add_argument('--gpu-id', dest = 'gpu_id', type = str, required = True)
    options = parser.parse_args()

    os.environ['CUDA_DEVICE_ORDER'] = 'PCI_BUS_ID'
    os.environ['CUDA_VISIBLE_DEVICES'] = options.gpu_id

    dataset = DatasetDescription(options.dataset_name, './tools/detectron2/' + options.dataset_name + '_dataset_description.pickle', False)
    DatasetCatalog.register(options.dataset_name, lambda : dataset.dataset())

    class Trainer(DefaultTrainer):
        @classmethod
        def build_train_loader(cls, cfg):
            return build_detection_train_loader(cfg, mapper = Mapper(cfg, True))

    cfg = get_cfg()
    cfg.merge_from_file(model_zoo.get_config_file('COCO-InstanceSegmentation/mask_rcnn_R_50_FPN_3x.yaml'))
    cfg.DATASETS.TRAIN = (options.dataset_name,)
    cfg.DATASETS.TEST = ()
    cfg.DATALOADER.NUM_WORKERS = 1
    cfg.INPUT.MASK_FORMAT = 'bitmask'
    cfg.MODEL.WEIGHTS = model_zoo.get_checkpoint_url('COCO-InstanceSegmentation/mask_rcnn_R_50_FPN_3x.yaml')
    cfg.OUTPUT_DIR = './tools/detectron2/coco_mask_rcnn_R_50_FPN_3x_120k_' + options.dataset_name
    cfg.SOLVER.IMS_PER_BATCH = 2
    cfg.SOLVER.BASE_LR = 0.00025
    cfg.SOLVER.MAX_ITER= 120000
    cfg.MODEL.ROI_HEADS.BATCH_SIZE_PER_IMAGE = 512
    cfg.MODEL.ROI_HEADS.NUM_CLASSES = len(dataset.class_list())

    if options.dataset_name == 'ycbv_bop_pbr':
        cfg.MODEL.BACKBONE.FREEZE_AT = 0

    os.makedirs(cfg.OUTPUT_DIR, exist_ok = True)

    tr = Trainer(cfg)
    tr.resume_or_load(resume=False)
    tr.train()


if __name__ == '__main__':
    main()
