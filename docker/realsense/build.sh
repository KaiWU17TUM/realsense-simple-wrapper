#! /bin/sh

LIBRS_VERSION="2.50.0"
IMAGE_NAME="cheneeheng/librealsense"

if [ $# -eq 1 ]; then

    if [ "$1" = "ubuntu20compactuvc" ]; then
        BASE_IMAGE="ubuntu:20.04"
        DOCKER_FILE="dockerfiles/Dockerfile.Ubuntu20CompactUVC"
        IMAGE_NAME="${IMAGE_NAME}:u20.04-v${LIBRS_VERSION}-compact-uvc"
    elif [ "$1" = "ubuntu20compact" ]; then
        BASE_IMAGE="ubuntu:20.04"
        DOCKER_FILE="dockerfiles/Dockerfile.Ubuntu20Compact"
        IMAGE_NAME="${IMAGE_NAME}:u20.04-v${LIBRS_VERSION}-compact"
    elif [ "$1" = "ubuntu20" ]; then
        BASE_IMAGE="ubuntu:20.04"
        DOCKER_FILE="dockerfiles/Dockerfile.Ubuntu20"
        IMAGE_NAME="${IMAGE_NAME}:u20.04-v${LIBRS_VERSION}"
    elif [ "$1" = "ubuntu20cuda1171" ]; then
        BASE_IMAGE="nvidia/cuda:11.7.1-cudnn8-devel-ubuntu20.04"
        DOCKER_FILE="dockerfiles/Dockerfile.Ubuntu20"
        IMAGE_NAME="${IMAGE_NAME}:u20.04-cu11.7.1-v${LIBRS_VERSION}"
    elif [ "$1" = "ubuntu20cuda1171vtkopcvpcl" ]; then
        BASE_IMAGE="nvidia/cuda:11.7.1-cudnn8-devel-ubuntu20.04"
        DOCKER_FILE="dockerfiles/Dockerfile.Ubuntu20VTKOpenCVPCL"
        IMAGE_NAME="${IMAGE_NAME}:u20.04-cu11.7.1-v${LIBRS_VERSION}-vtk-opcv-pcl"
    else
        echo "Unknown argument, should be {ubuntu20compactuvc/ubuntu20compact/ubuntu20/ubuntu20cuda1171/ubuntu20cuda1171vtkopcvpcl}"
        exit 1
    fi

    echo "Building image : ${IMAGE_NAME}"
    DOCKER_BUILDKIT=1 docker build \
        --file ${DOCKER_FILE} \
        --target librealsense \
        --build-arg BASE_IMAGE=${BASE_IMAGE} \
        --build-arg LIBRS_VERSION=${LIBRS_VERSION} \
        --tag ${IMAGE_NAME} \
        .
    echo "Built image : ${IMAGE_NAME}\n"

else

    echo "1 argument is expected : {ubuntu20compactuvc/ubuntu20compact/ubuntu20/ubuntu20cuda1171/ubuntu20cuda1171vtkopcvpcl}"
    exit 1

fi
