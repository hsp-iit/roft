HO3D_PATH=`cat ./config/ho3d_location`
DETECTRON2_VENV_PATH=`cat ./config/detectron2_venv_location`

. $DETECTRON2_VENV_PATH/bin/activate
python ./tools/detectron2/dataset.py --dataset-name ho3d --dataset-path $HO3D_PATH --generate "true"
