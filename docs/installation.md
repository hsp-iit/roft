(WIP)

We assume `CUDA 10.1` and that you are using different `venvs` for `DOPE` and `Detectron 2`.

### DOPE

Just install `DOPE` [requirements](https://github.com/NVlabs/Deep_Object_Pose/blob/master/requirements.txt)

### Detectron 2

Install [requirements](https://github.com/robotology/optical-flow-6d-tracking-code/blob/main/requirements.txt) of this repository (which includes `torch 1.6`).

Then proceed with `detectron 2` installation as follows:

```
python -m pip install detectron2 -f https://dl.fbaipublicfiles.com/detectron2/wheels/cu101/torch1.6/index.html
```

