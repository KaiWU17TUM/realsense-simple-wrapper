#!/bin/bash

PURPLE='\033[0;35m'
BLUE='\033[0;36m'
NC='\033[0m'

MODE=$1

if [ $# -eq 1 ]; then

    if [ "${MODE}" = "start" ]; then

        echo "${BLUE}STEP I : starting necessary services ...${NC}"
        sudo modprobe usbip_host
        sudo usbipd &

        echo "${BLUE}STEP II : listing available devices ...${NC}"
        DEVICE_ARRAY=$(sudo usbip list -l | cut -d " " -f 4 | grep [0-9])
        #echo "Devices connected are"
        for DEVICE in $DEVICE_ARRAY; do
            sudo usbip bind -b $DEVICE
        done

    elif [ "${MODE}" = "stop" ]; then

        printf "${BLUE}Detaching available devices ${NC}\n"
        DEVICE_ARRAY=$(sudo usbip list -l | cut -d " " -f 4 | grep [0-9])
        for DEVICE in $DEVICE_ARRAY; do
            printf "$DEVICE \n"
            sudo usbip unbind -b $DEVICE
        done

        sudo killall -9 usbipd

    else

        echo "argument 1 is expected to be : {start/stop}"
        exit 1

    fi

else

    echo "1 argument is expected : {start/stop}"
    exit 1

fi
