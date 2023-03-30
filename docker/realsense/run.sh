#! /bin/sh

LIBRS_VERSION="2.50.0"

if [ $# -eq 2 ]; then

    if [ "$1" = "ubuntu20compact" ]; then
        IMAGE_NAME="${IMAGE_NAME}:u20.04-v${LIBRS_VERSION}-compact"
    elif [ "$1" = "ubuntu20" ]; then
        IMAGE_NAME="${IMAGE_NAME}:u20.04-v${LIBRS_VERSION}"
    elif [ "$1" = "ubuntu20cuda1171" ]; then
        IMAGE_NAME="${IMAGE_NAME}:u20.04-cu11.7.1-v${LIBRS_VERSION}"
    elif [ "$1" = "ubuntu20cuda1171vtkopcvpcl" ]; then
        IMAGE_NAME="${IMAGE_NAME}:u20.04-cu11.7.1-v${LIBRS_VERSION}-vtk-opcv-pcl"
    else
        echo "Unknown argument, should be {ubuntu20compact/ubuntu20/ubuntu20cuda1171/ubuntu20cuda1171vtkopcvpcl}"
        exit 1
    fi

    # By using --device-cgroup-rule flag we grant the docker continer permissions -
    # to the camera and usb endpoints of the machine.
    # It also mounts the /dev directory of the host platform on the contianer

    docker run -it --rm \
        -v /dev:/dev \
        --device-cgroup-rule "c 81:* rmw" \
        --device-cgroup-rule "c 189:* rmw" \
        ${TARGET_TAG} $2

else

    echo "2 arguments are expected : {ubuntu20compact/ubuntu20/ubuntu20cuda1171/ubuntu20cuda1171vtkopcvpcl} {cmd}"
    exit 1

fi
