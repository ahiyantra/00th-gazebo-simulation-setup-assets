#!/usr/bin/bash

: <<'END_COMMENT'
# try the instructions givne below with the nvidia foundationpose project's docker image built on your personal device;
# i was running these commands in the "_3D-Pose-Estimator_/FoundationPose/" directory;
END_COMMENT

pip install pyrealsense2
export NINJA_JOBS=1
export MAKEFLAGS="-j1"
apt-get update && apt-get install -y lsb-release coreutils usbutils mesa-utils && apt-get clean all
pip install -r requirements.txt

: <<'END_COMMENT'
# we could use absolute paths if necessary;
$ export PYTHONPATH=$PYTHONPATH:/mnt/e/-_EDI_-/assets/_3D-Pose-Estimator_/FoundationPose/mycpp/build
# we should try to avoid using absolute paths;
END_COMMENT

export PYTHONPATH=$PYTHONPATH:$(pwd)/mycpp/build
CMAKE_PREFIX_PATH=$CONDA_PREFIX/lib/python3.9/site-packages/pybind11/share/cmake/pybind11 bash build_all_conda.sh

: <<'END_COMMENT'
# make a copy of "mycpp.cpython-39-x86_64-linux-gnu.so" in the "mycppp/build" directory & rename it to "mycpp.so";
END_COMMENT

cd mycpp/build
cp mycpp.cpython-39-x86_64-linux-gnu.so mycpp.so

: <<'END_COMMENT'
# the part representing python's version in the command shared above could need to be changed from "39" to 310";
# store address of the project directory in a variable to access it without absolute paths;
END_COMMENT

cd ../..
export PROJ_DIR=$(pwd)

: <<'END_COMMENT'
# shift to the directory of a certain library & rebuild it separately to avoid a certain error;
# the necessary library named below is in the model container but not in the imoco container;
END_COMMENT

cd /nvdiffrast
python setup.py build

: <<'END_COMMENT'
# we can return to the project directory using absolute path;
$ cd /mnt/e/-_EDI_-/assets/_3D-Pose-Estimator_/FoundationPose
# we should return to the project directory using variable path;
END_COMMENT

cd $PROJ_DIR

: <<'END_COMMENT'
# we can run the remaining commands for cudnn package before resolving the issue of nvdiffrast package being absent;
# the solution for the "ninja: build stopped: subcommand failed" issue is installing a suitable version of the cudnn package compatible with the cuda driver;
END_COMMENT

wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/cuda-keyring_1.1-1_all.deb
dpkg -i cuda-keyring_1.1-1_all.deb
apt-get update
apt-get -y install cudnn-cuda-11

: <<'END_COMMENT'
# don't use "sudo" keyword inside nvidia foundationpose project's docker container;
# we can try running some relevant bash commands from the official dockerfile for the nvidia foundationpose repository but that may not be enough to resolve the issue of nvdiffrast being missing in the imoco container;
END_COMMENT

CMAKE_PREFIX_PATH=$CONDA_PREFIX/lib/python3.9/site-packages/pybind11/share/cmake/pybind11 bash build_all.sh

: <<'END_COMMENT'
# use CTRL+C to pause a demo through keyboard interupption before closing the demo viewing window;
# if a demo script crashes upon running for the first time, then trying running it again once;
# for video stream processor = $ python run_demo.py / $ python3 run_demo.py
# for single image processor = $ python run_demo2.py / $ python3 run_demo2.py
# for video steam processor's multiple object detector version = $ python run_demo-multi.py (this didn't work!)
# for single image processor's multiple object detector version = $ python run_demo2-multi.py (this didn't work!)
# the 'get_instance_ids_in_image' function is an attribute of the 'BopBaseReader' class & not of the 'YcbineoatReader' class (it seems that these two classes aren't interchangeable);
# intel’s realsense cameras don’t work in some cases with a USB 3 port, even though they work with a USB 2 port (this issue can be side-stepped by changing ports);
# perhaps the solution for missing ultralytics module issue needs to be implemented before & not after trying to use the nvidia foundationpose models with the intel realsense cameras;
# it seems that we need to export system variables separately for each instance of the project's docker container if they weren't included during the building process;
# the bash commands from "irs_hardware[dot]sh" without "sudo" keyword are given below;
END_COMMENT

apt-get install usbutils -y
mkdir -p /etc/apt/keyrings
curl -sSf https://librealsense.intel.com/Debian/librealsense.pgp | tee /etc/apt/keyrings/librealsense.pgp > /dev/null
echo "deb [signed-by=/etc/apt/keyrings/librealsense.pgp] https://librealsense.intel.com/Debian/apt-repo `lsb_release -cs` main" | \
tee /etc/apt/sources.list.d/librealsense.list
apt-get update
apt-get install librealsense2-dkms librealsense2-utils librealsense2-dev librealsense2-dbg -y
apt install ros-humble-realsense2-* -y
pip install --upgrade ultralytics
=pip install pyrealsense2

: <<'END_COMMENT'
# the packages for intel realsense device can be installed before those for invidia foundationpose model but it's fine to change the order;
# the nvidia foundationpose project's docker container lacks "lsb-release" & "coreutils" packages; it causes errors for "lsb_release" & "tee" commands;
# all of the necessary video devices need to be identified & added to the docker container separately;
$ ls /dev/video*
# bash command option example for docker container: "--device=/dev/video0"
# the realsense-viewer app's motion module isn't working because the nvidia foundationpose docker container isn't built for it; we don't need it, so we can work without resolving this issue;
# for video stream processor = $ python 6d_stream.py / $ python3 6d_stream.py
# for single image processor = $ python 6d_image.py / $ python3 6d_image.py
# for video steam processor's multiple object detector version = $ python 6d_stream-multi_01.py (under construction)
# for single image processor's live feed version = $ python 6d_image-live_01.py / $  take_image_for_6d.py
# for single image processor's multiple object detector version = $ python 6d_image-multi_01.py (under construction)
END_COMMENT

echo "The nvidia foundationpose software and intel realsense hardware should now be integrated!"