# Integrating the project's computer vision model with the intel realsense RGBD cameras.

# Resolving errors when integrating the RGBD camera hardware with the project's AI model :

01. If it's not already present, then add necessary code to load the custom model after an official model in the "6d_image" & "6d_stream" python scripts for the project (to aovid errors involving modules like 'ultralytics.utils' going missing).

02. If the version of ultralytics module for python used to train the project's version of nvidia foundationpose model is different from the one used to load the model, then there's a version mismatch error.

03. The prepared bash automation script for nvidia foundationpose software is "nfp_software[dot]sh" but it may not work as intended without adjustments due to path and address issues.

04. The prepared bash automation script for intel realsense hardware is "irs_hardware[dot]sh" but it may not work as intended without adjustments due to path and address issues.

05. The purpose of "nfp_software[dot]sh" is to integrate the nvidia foundationpose software with the intel realsense hardware but it may not work as intended without adjustments due to path and address issues.

06. The purpose of "both_combined[dot]sh" is to fully merge the nvidia foundationpose setup with the intel realsense hardware setup on all levels by modifying a running docker container.

# Mini conda - Quick command line install - Linux :

mkdir -p ~/miniconda3
wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh -O ~/miniconda3/miniconda.sh
bash ~/miniconda3/miniconda.sh -b -u -p ~/miniconda3
rm -rf ~/miniconda3/miniconda.sh
~/miniconda3/bin/conda init bash

# Mini conda - Command not found - Solution :

export PATH="/root/miniconda3/bin:$PATH"

# Error - run 'conda init' before 'conda activate' - Solution (to use before running the nvidia foundationpose environment) :

source activate base

# Error - Could not find a version that satisfies the requirement ultralytics.utils - No matching distribution found for ultralytics.utils - Solution (to use after running the requirements text file) :

pip3 install --upgrade ultralytics

# 2.1. FoundationPose → 2.1.1. Set up and installation → 2.1.1.1 - First Option :

01. create conda environment ...

conda create -n foundationpose python=3.9 -y

02. activate conda environment ...

conda activate foundationpose

03. Install Eigen3 3.4.0 under conda environment ...

conda install conda-forge::eigen=3.4.0 -y
export CMAKE_PREFIX_PATH="$CMAKE_PREFIX_PATH:/eigen/path/under/conda"

04. install dependencies ...

python -m pip install -r requirements.txt

05. Install NVDiffRast ...

python -m pip install --quiet --no-cache-dir git+https://github.com/NVlabs/nvdiffrast.git

06. Install Kaolin (Optional, needed if running model-free setup) ...

python -m pip install --quiet --no-cache-dir kaolin==0.15.0 -f https://nvidia-kaolin.s3.us-east-2.amazonaws.com/torch-2.0.0_cu118.html

07. Install PyTorch3D ...

python -m pip install --quiet --no-index --no-cache-dir pytorch3d -f https://dl.fbaipublicfiles.com/pytorch3d/packaging/wheels/py39_cu118_pyt200/download.html

08. Build extensions ...

CMAKE_PREFIX_PATH=$CONDA_PREFIX/lib/python3.9/site-packages/pybind11/share/cmake/pybind11 bash build_all_conda.sh

# 2.1. FoundationPose → 2.1.2. Running / Inference → 2.1.2.1 - Inputs :

01. sample directory address = ./RootFolder/demo_data/bottle0 

02. my directory address (windows 10) = E:\-_-02nd-redacted-term-_-\assets\3D Pose Estimator\FoundationPose\demo_data\bottle0

03. directory address (wsl 2 ubuntu 22) = /mnt/e/-_-02nd-redacted-term-_-/assets/3D Pose Estimator/FoundationPose/demo_data/bottle0/

# 2.1. FoundationPose → 2.1.2. Running / Inference → 2.1.2.2 - Command to run :

01. Video stream version ...

python 6d_stream.py 

02. Image input version ...

python 6d_image.py

# Author details.

S. S. W. (it23139[at]lbtu.lv)
