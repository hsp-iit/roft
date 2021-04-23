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
from detectron2 import model_zoo
from detectron2.config import get_cfg
from detectron2.evaluation import COCOEvaluator, inference_on_dataset
from detectron2.data import DatasetCatalog, MetadataCatalog, build_detection_test_loader
from detectron2.engine import DefaultTrainer


def main():

    parser = argparse.ArgumentParser()
    parser.add_argument('--dataset-name', dest = 'dataset_name', type = str, required = True)
    parser.add_argument('--gpu-id', dest = 'gpu_id', type = str, required = True)
    options = parser.parse_args()

    os.environ['CUDA_DEVICE_ORDER'] = 'PCI_BUS_ID'
    os.environ['CUDA_VISIBLE_DEVICES'] = options.gpu_id

    training_dataset_name = 'ycbv_bop_pbr'
    training_dataset_paths =\
    {
        'ycbv_bop_pbr' : './tools/detectron2/ycbv_bop_pbr_dataset_description.pickle'
    }
    weight_paths =\
    {
        'ycbv_bop_pbr' : './tools/detectron2/coco_mask_rcnn_R_50_FPN_3x_60k_ycbv_bop_pbr/model_0059999.pth'
    }

    training_dataset = DatasetDescription(training_dataset_name, training_dataset_paths[training_dataset_name], False)
    DatasetCatalog.register(training_dataset_name, lambda : training_dataset.dataset())

    testing_dataset = DatasetDescription(options.dataset_name, './tools/detectron2/' + options.dataset_name +  '_dataset_description.pickle', False)
    DatasetCatalog.register(options.dataset_name, lambda : testing_dataset.dataset())
    MetadataCatalog.get(options.dataset_name).set(thing_classes = testing_dataset.class_list())

    cfg = get_cfg()
    cfg.merge_from_file(model_zoo.get_config_file('COCO-InstanceSegmentation/mask_rcnn_R_50_FPN_3x.yaml'))
    cfg.DATASETS.TRAIN = (training_dataset_name,)
    cfg.INPUT.MASK_FORMAT = 'bitmask'
    cfg.MODEL.WEIGHTS = weight_paths[training_dataset_name]
    cfg.OUTPUT_DIR = '/'.join(weight_paths[training_dataset_name].split('/')[:-1])
    cfg.MODEL.ROI_HEADS.NUM_CLASSES = len(testing_dataset.class_list())

    trainer = DefaultTrainer(cfg)
    trainer.resume_or_load()
    evaluator = COCOEvaluator(options.dataset_name, ("bbox", "segm"), False, output_dir = './')
    validator_loader = build_detection_test_loader(cfg, options.dataset_name)
    inference_on_dataset(trainer.model, validator_loader, evaluator)


if __name__ == '__main__':
    main()
