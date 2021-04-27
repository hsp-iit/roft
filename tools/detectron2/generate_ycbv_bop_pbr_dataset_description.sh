YCBV_BOP_PBR_PATH=`cat ./config/ycbv_bop_pbr_location`
DETECTRON2_VENV_PATH=`cat ./config/detectron2_venv_location`

. $DETECTRON2_VENV_PATH/bin/activate
python ./tools/detectron2/dataset.py --dataset-name ycbv_bop_pbr --dataset-path $YCBV_BOP_PBR_PATH --generate "true"
