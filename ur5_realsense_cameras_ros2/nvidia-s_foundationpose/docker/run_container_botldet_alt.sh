# old :

# docker rm -f foundationpose_-02nd-redacted-term-
# DIR=$(pwd)/../
# echo "DIR is set to: $DIR"
# xhost +  && docker run --gpus all --env NVIDIA_DISABLE_REQUIRE=1 -it --network=host --name foundationpose_-02nd-redacted-term-  --cap-add=SYS_PTRACE --security-opt seccomp=unconfined -v $DIR:$DIR -v /home:/home -v /mnt:/mnt -v /tmp/.X11-unix:/tmp/.X11-unix -v /tmp:/tmp --device=/dev/bus/usb/001/004 --device=/dev/video0 --device=/dev/video1 --device=/dev/video2 --device=/dev/video3 --device=/dev/video4 --device=/dev/video5 --ipc=host -e DISPLAY=${DISPLAY} -e GIT_INDEX_FILE foundationpose_-02nd-redacted-term-:09092024 bash -c "cd $DIR && bash"

# new :

#!/bin/bash
docker rm -f foundationpose_-02nd-redacted-term-

# Use the correct absolute path
DIR=/mnt/c/projects/3d-pose-est_customized/_3D-Pose-Estimator_/FoundationPose/

# Print directory to ensure correctness
echo "DIR is set to: $DIR"

# Run the Docker container with mounts
xhost + && docker run --gpus all --env NVIDIA_DISABLE_REQUIRE=1 -it --network=host --name foundationpose_-02nd-redacted-term- \
  --cap-add=SYS_PTRACE --security-opt seccomp=unconfined \
  -v $DIR:$DIR \
  -v /mnt/c/projects/3d-pose-est_customized/_3D-Pose-Estimator_:/home/_3D-Pose-Estimator_ \
  -v /mnt:/mnt \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v /tmp:/tmp \
  --device=/dev/bus/usb/002/002 \
  --device=/dev/video0 --device=/dev/video1 --device=/dev/video2 \
  --device=/dev/video3 --device=/dev/video4 --device=/dev/video5 \
  --ipc=host -e DISPLAY=${DISPLAY} -e GIT_INDEX_FILE \
  foundationpose_-02nd-redacted-term-:16092024 bash -c "cd $DIR && bash"
