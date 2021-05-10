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
from detectron2.evaluation import COCOEvaluator, inference_on_dataset
from detectron2.data import DatasetCatalog, MetadataCatalog, build_detection_test_loader
from detectron2.engine import DefaultTrainer
from tools.detectron2 import results_saver



def evaluate_single_ckpt(options):
    
    try:
        eval_ckpt = str(int(options.eval_ckpt) - 1).zfill(7)
    except:
        eval_ckpt = options.eval_ckpt
        
    print(f'--Using checkpoint: {eval_ckpt}!')
    weight_paths =\
    {
        'ycbv_bop_pbr' : os.path.join(options.weights_path, f'model_{eval_ckpt}.pth')
    }
    
    training_dataset_name = 'ycbv_bop_pbr'
    training_dataset_paths =\
    {
        'ycbv_bop_pbr' : './tools/detectron2/ycbv_bop_pbr_dataset_description.pickle'
    }

    training_dataset = DatasetDescription(training_dataset_name, training_dataset_paths[training_dataset_name], False)
    try:
        DatasetCatalog.register(training_dataset_name, lambda : training_dataset.dataset())
    except:
        print(f'--- WARNING: Using existing dataset {training_dataset_name}. If this was not intended, please modify.')

    testing_dataset = DatasetDescription(options.dataset_name, './tools/detectron2/' + options.dataset_name +  '_dataset_description.pickle', False)
    try:
        DatasetCatalog.register(options.dataset_name, lambda : testing_dataset.dataset())
    except:
        print(f'--- WARNING: Using existing dataset {training_dataset_name}. If this was not intended, please modify.')    
    MetadataCatalog.get(options.dataset_name).set(thing_classes = testing_dataset.class_list())

    cfg = get_cfg()
    cfg.merge_from_file(model_zoo.get_config_file('COCO-InstanceSegmentation/mask_rcnn_R_50_FPN_3x.yaml'))
    cfg.DATASETS.TRAIN = (training_dataset_name,)
    cfg.INPUT.MASK_FORMAT = 'bitmask'
    cfg.MODEL.WEIGHTS = weight_paths[training_dataset_name]
    cfg.OUTPUT_DIR = '/'.join(weight_paths[training_dataset_name].split('/')[:-1])
    cfg.MODEL.ROI_HEADS.NUM_CLASSES = len(testing_dataset.class_list())

    trainer = DefaultTrainer(cfg)
    trainer.resume_or_load(resume=False)
    evaluator = COCOEvaluator(options.dataset_name, ("bbox", "segm"), False, output_dir = './')
    validator_loader = build_detection_test_loader(cfg, options.dataset_name, mapper = Mapper(cfg, True, options.augmentation))
    results = inference_on_dataset(trainer.model, validator_loader, evaluator)
    
    return results
    

def boolean_string(s):
    if s not in {'False', 'True', True, False}:
        raise ValueError('Not a valid boolean string')
    return s == 'True' or s == True


def main():

    parser = argparse.ArgumentParser()
    parser.add_argument('--dataset-name', dest = 'dataset_name', type = str, required = True)
    parser.add_argument('--gpu-id', dest = 'gpu_id', type = str, required = True)
    parser.add_argument('--eval-ckpt', dest = 'eval_ckpt', default='final')
    parser.add_argument('--weights-path', dest = 'weights_path', type = str, default='./tools/detectron2/coco_mask_rcnn_R_50_FPN_3x_60k_ycbv_bop_pbr')
    parser.add_argument('--augmentation', dest = 'augmentation', default=False)
    options = parser.parse_args()
    options.augmentation = boolean_string(options.augmentation)

    os.environ['CUDA_DEVICE_ORDER'] = 'PCI_BUS_ID'
    os.environ['CUDA_VISIBLE_DEVICES'] = options.gpu_id

    results = list()
    if options.eval_ckpt != 'all':
        results_dict = evaluate_single_ckpt(options)
        results = [results_dict['segm']]
    else:
        ckpts = sorted([int(ckpt.split('_')[-1].split('.')[0])+1 
                        for ckpt in os.listdir(options.weights_path)
                        if ckpt.startswith('model_0') and ckpt.endswith('.pth')])
        for ckpt in ckpts:
            options.eval_ckpt = ckpt
            results_dict = evaluate_single_ckpt(options)
            results.append(results_dict['segm'])
    
    out_dir = os.path.join(os.getcwd(), 'detectron2_results', options.dataset_name +
                           os.path.basename(os.path.normpath(options.weights_path)))
    results_saver.save_res(results, out_dir=out_dir)


if __name__ == '__main__':
    main()
