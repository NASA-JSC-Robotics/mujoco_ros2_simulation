#!/bin/bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
IMAGE_NAME="mujoco_ros2_simulation"
TAG="humble"

pushd "${SCRIPT_DIR}/.." > /dev/null || exit

# Run the container and mount the source into the workspace directory
docker run --rm \
           -it \
           --network host \
           -e DISPLAY \
           -e QT_X11_NO_MITSHM=1 \
           --mount type=bind,src=/tmp/.X11-unix,dst=/tmp/.X11-unix,ro \
           --mount type=bind,src=.,dst="/opt/mujoco/ws/src/mujoco_ros2_simulation" \
           --name ${IMAGE_NAME} \
           ${IMAGE_NAME}:${TAG} \
           bash

popd > /dev/null || exit
