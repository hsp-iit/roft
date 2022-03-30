<h1 align="center">
  ROFT
</h1>

<p align="center"><img src="https://github.com/hsp-iit/roft/blob/main/assets/scheme.png" alt=""/></p>


<h4 align="center">
  ROFT: Real-Time Optical Flow-Aided 6D Object Pose and Velocity Tracking
</h4>

<div align="center">
  IEEE Robotics and Automation Letters, vol. 7, no. 1, pp. 159-166, Jan. 2022
</div>

<div align="center">
  <a href="https://ieeexplore.ieee.org/document/9568706"><b>Paper</b></a> |
  <a href="https://arxiv.org/abs/2111.03821"><b>arXiv</b></a> |
  <a href="https://ieeexplore.ieee.org/ielx7/7083369/9568780/9568706/supp1-3119379.mp4?arnumber=9568706"><b>Video</b></a>
</div>

[![DOI](https://zenodo.org/badge/308858064.svg)](https://zenodo.org/badge/latestdoi/308858064) ![CI badge](https://github.com/hsp-iit/roft/workflows/C++%20CI%20Workflow/badge.svg)

## Reproducing the experiments

We support running the experiments on the [Fast-YCB](https://github.com/hsp-iit/fast-ycb) dataset via the provided Docker image.

1. Pull the docker image:
    ```console
    docker pull ghcr.io/hsp-iit/roft:latest
    ```
1. Launch the container:
    ```console
    docker run -it --rm --user user --env="DISPLAY" --net=host --device /dev/dri/ ghcr.io/hsp-iit/roft:latest
    ```
    If an NVIDIA GPU is adopted, please use instead:
    ```console
    docker run -it --rm --user user --env="DISPLAY" --net=host -e NVIDIA_DRIVER_CAPABILITIES=all -v /tmp/.X11-unix:/tmp/.X11-unix --gpus all --runtime=nvidia ghcr.io/hsp-iit/roft:latest
    ```
1. Update and build the project:
    ```console
    cd /home/user/roft
    git pull
    cd build
    make install
    ```
1. Download and extract the accompanying data (Fast-YCB dataset and pre-evaluated results) and the YCB-Video model set:
    ```console
    cd /home/user/roft
    bash tools/download/download_results.sh
    bash tools/download/download_fastycb.sh
    bash tools/download/download_ycb_models.sh
    ```
1. Initialize the datasets:
    ```console
    cd /home/user/roft
    bash test/init.sh
    ```
1. Run the experiments (optional):
    ```console
    cd /home/user/roft
    bash test/run_paper_experiments
    ```
    > The accompanying data contains the pre-evaluated results. If desired, the results can be re-evaluated using the above command.
1. Run the evaluation:
    ```console
    cd /home/user/roft
    bash evaluation/run_paper_evaluation
    ```
1. Visualize the results:
    The results on the Fast-YCB dataset (Table I, II, IV and Figure 3) can be found in `/home/user/roft/evaluation_output`:
    - `tableI.pdf`
    - `tableII.pdf`
    - `tableIV.pdf`
    - `Fig3_*.png`

    The docker image provides `evince` and `eog` in order open pdf and png files, respectively.

> In order to run part of the provided software it could be required to temporarily execute `xhost +` in a console outside of Docker in order to allow the container accessing the X server facilities. The command can be run even **after** the container has been already launched.

> If you want to install the repository manually, please refer to the recipe contained in the [**`Dockerfile`**](./dockerfiles/Dockerfile). Please be aware that the results might differ if unsupported versions of the dependencies are used.

> Support for reproducing the experiments on the HO-3D dataset will be added in the near future.

> Instructions on how to use the ROFT library in external C++ projects and how to execute ROFT on custom datasets will be added in the near future.

## Citing ROFT

If you find the ROFT code useful, please consider citing the associated publication:

```bibtex
@ARTICLE{9568706,
author={Piga, Nicola A. and Onyshchuk, Yuriy and Pasquale, Giulia and Pattacini, Ugo and Natale, Lorenzo},
journal={IEEE Robotics and Automation Letters},
title={ROFT: Real-Time Optical Flow-Aided 6D Object Pose and Velocity Tracking},
year={2022},
volume={7},
number={1},
pages={159-166},
doi={10.1109/LRA.2021.3119379}
}
```

and/or the repository itself by pressing on the `Cite this respository` button in the **About** section.

The pre-evaluated results on the [Fast-YCB](https://github.com/hsp-iit/fast-ycb) dataset are stored within the IIT Dataverse and identified by the following [![DOI:10.48557/9FJW42](http://img.shields.io/badge/DOI-10.48557/9FJW42-0a7bbc.svg)](https://doi.org/10.48557/9FJW42).

## Maintainer

This repository is maintained by:

| | |
|:---:|:---:|
| [<img src="https://github.com/xenvre.png" width="40">](https://github.com/xenvre) | [@xenvre](https://github.com/xenvre) |
