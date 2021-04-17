HO3D_PATH=?
DOPE_PATH=?
DOPE_VENV_PATH=?
GPU_ID=$1

. $DOPE_VENV_PATH/bin/activate
for OBJECT_NAME in 003_cracker_box 004_sugar_box 006_mustard_bottle 010_potted_meat_can; do
    python ./tools/dope/inference.py --dataset-name ho3d --dataset-path $HO3D_PATH --dope-path $DOPE_PATH --gpu-id $GPU_ID --output-path $HO3D_PATH --object-name $OBJECT_NAME --rgb-format png
done
