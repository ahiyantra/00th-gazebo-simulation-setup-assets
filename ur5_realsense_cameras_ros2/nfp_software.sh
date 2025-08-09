#!/usr/bin/bash

: <<'END_COMMENT'
cd ../../../../assets/_3D-Pose-Estimator_/FoundationPose 
END_COMMENT

cd ../_3D-Pose-Estimator_/FoundationPose 

pwd

mkdir -p ~/miniconda3

wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh -O ~/miniconda3/miniconda.sh

bash ~/miniconda3/miniconda.sh -b -u -p ~/miniconda3

rm -rf ~/miniconda3/miniconda.sh

~/miniconda3/bin/conda init bash

export PATH="/root/miniconda3/bin:$PATH"

source activate base

conda create -n foundationpose python=3.9 -y

conda activate foundationpose

conda install conda-forge::eigen=3.4.0 -y

export CMAKE_PREFIX_PATH="$CMAKE_PREFIX_PATH:/root/miniconda3/envs/foundationpose/include/eigen3"

python -m pip install -r requirements.txt

pip3 install --upgrade ultralytics

python -m pip install --quiet --no-cache-dir git+https://github.com/NVlabs/nvdiffrast.git

python -m pip install --quiet --no-cache-dir kaolin==0.15.0 -f https://nvidia-kaolin.s3.us-east-2.amazonaws.com/torch-2.0.0_cu118.html

python -m pip install --quiet --no-index --no-cache-dir pytorch3d -f https://dl.fbaipublicfiles.com/pytorch3d/packaging/wheels/py39_cu118_pyt200/download.html

CMAKE_PREFIX_PATH=$CONDA_PREFIX/lib/python3.9/site-packages/pybind11/share/cmake/pybind11 bash build_all_conda.sh

echo "All model weights for this version of NVIDIA's FoundationPose project should now be runnable!"