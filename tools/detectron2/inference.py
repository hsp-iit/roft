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
import numpy
import os
from dataset import DatasetDescription
from detectron2 import model_zoo
from detectron2.config import get_cfg
from detectron2.data import MetadataCatalog, DatasetCatalog
from detectron2.engine import DefaultPredictor
from detectron2.utils.visualizer import Visualizer
from detectron2.utils.visualizer import ColorMode
from glob import glob
from tqdm import tqdm


class Detectron2Inference():

    def __init__(self, options):
        """Constructor."""

        self.dataset_name = options.dataset_name
        self.dataset_path = options.dataset_path
        self.object_name = options.object_name
        self.output_path = options.output_path
        self.rgb_format = options.rgb_format
        self.training_dataset_name = options.training_dataset_name

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

        # Configure and load detectron2 predictor
        self.dataset = DatasetDescription(options.training_dataset_name, './tools/detectron2/' + options.training_dataset_name + '_dataset_description.pickle', False)
        DatasetCatalog.register(options.training_dataset_name, lambda : self.dataset.dataset())
        MetadataCatalog.get(options.training_dataset_name).set(thing_classes = self.dataset.class_list())
        self.metadata = MetadataCatalog.get(options.training_dataset_name)

        cfg = get_cfg()
        cfg.merge_from_file(model_zoo.get_config_file('COCO-InstanceSegmentation/mask_rcnn_R_50_FPN_3x.yaml'))
        cfg.DATASETS.TRAIN = (options.training_dataset_name,)
        cfg.DATASETS.TEST = ()
        cfg.DATALOADER.NUM_WORKERS = 1
        cfg.INPUT.MASK_FORMAT = 'bitmask'
        cfg.MODEL.WEIGHTS = './tools/detectron2/coco_mask_rcnn_R_50_FPN_3x_120k_' + options.training_dataset_name + '/model_0059999.pth'
        cfg.SOLVER.IMS_PER_BATCH = 2
        cfg.SOLVER.BASE_LR = 0.00025
        cfg.SOLVER.MAX_ITER = 120000
        cfg.MODEL.ROI_HEADS.SCORE_THRESH_TEST = 0.6
        cfg.MODEL.ROI_HEADS.BATCH_SIZE_PER_IMAGE = 512
        cfg.MODEL.ROI_HEADS.NUM_CLASSES = len(self.dataset.class_list())

        self.predictor = DefaultPredictor(cfg)


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

                self.process_sequence(path, output_path)

        elif self.dataset_name == 'ycbv_synthetic':
            # Compose input path
            path = os.path.join(self.dataset_path, self.object_name)

            # Compose output path and create it
            output_path = os.path.join(self.output_path, self.object_name)

            self.process_sequence(path, output_path)


    def process_sequence(self, path, out_path):
        """Process one sequence and save to output path."""

        path = os.path.join(path, 'rgb')
        output_path = os.path.join(out_path, 'masks', 'mrcnn_' + self.training_dataset_name)
        debugging_path = os.path.join(out_path, 'masks', 'mrcnn_' + self.training_dataset_name, 'debugging')

        name = self.object_name
        name_short = self.object_name.split('_')[0]

        os.makedirs(output_path, exist_ok = True)
        os.makedirs(debugging_path, exist_ok = True)

        for image in tqdm(glob(os.path.join(path, '*.png'))):

            # Load image
            index = image.split('/')[-1].split('.')[0]
            im = cv2.imread(image)

            if self.dataset_name == 'ycbv_synthetic':
                # These frames are @ 1280x720, hence they are zeropadded, extended and resized
                # to the original resolution required for detectron2
                im_extended = numpy.zeros((960, 1280, 3))
                im_extended[120:720 + 120, :, :] = im
                im = cv2.resize(im_extended, (640, 480))

            # Predict masks
            outputs = self.predictor(im)
            instances = outputs['instances'].to('cpu')
            classes_id = instances.pred_classes.numpy()
            list_detections = []
            masks = instances.pred_masks.numpy()

            # Extract mask
            mask = numpy.zeros((480, 640), dtype = numpy.uint8)
            for i, id in enumerate(classes_id):
                if self.dataset.class_list()[id] == name_short:
                    mask[masks[i, :, :]] = 255

            # Debugging image
            v = Visualizer(im[:, :, ::-1], metadata = self.metadata, scale=1.0, instance_mode=ColorMode.IMAGE_BW)
            view = v.draw_instance_predictions(outputs['instances'].to('cpu'))
            view = view.get_image()[:, :, ::-1]
            cv2.imwrite(os.path.join(debugging_path, name + '_' + str(index) + '.jpg'), view)

            # These frames are @ 1280x720, hence we need to restore them back
            if self.dataset_name == 'ycbv_synthetic':
                tmp = cv2.resize(mask, (1280, 960))
                tmp_reduced = numpy.zeros((720, 1280))
                tmp_reduced = tmp[120:720 + 120, :]
                mask = tmp_reduced

            # Write mask
            cv2.imwrite(os.path.join(output_path, name + '_' + str(index) + '.png'), mask)


def main():
    parser = argparse.ArgumentParser(description = 'DOPE inference.')
    parser.add_argument('--dataset-name', dest = 'dataset_name', type = str, required = True)
    parser.add_argument('--dataset-path', dest = 'dataset_path', type = str, required = True)
    parser.add_argument('--gpu-id', dest = 'gpu_id', type = int, required = True)
    parser.add_argument('--object-name', dest = 'object_name', type = str, required = True)
    parser.add_argument('--output-path', dest = 'output_path', type = str, required = True)
    parser.add_argument('--training-dataset-name', dest = 'training_dataset_name', type = str, required = True)
    parser.add_argument('--rgb-format', dest = 'rgb_format', type = str, required = True)

    options = parser.parse_args()

    Detectron2Inference(options).process()


if __name__ == '__main__':
    main()
