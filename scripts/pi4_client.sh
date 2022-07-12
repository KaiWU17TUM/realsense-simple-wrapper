#!/bin/bash

RED='\033[1;31m'
GREEN='\033[1;32m'
BLUE='\033[0;36m'
NC='\033[0m'

MODE=$1
RASP_IP=$2
RASP_USER=$3
RASP_ADDR=${RASP_USER}'@'${RASP_IP}

if [ $# -eq 3 ]; then

    if [ "${MODE}" = "start" ]; then

        printf "${GREEN}"
        printf "================================================================================\n"
        printf "${NC}"

        printf "${GREEN}"
        printf "Attaching all hardwares\n"
        printf "${NC}"

        printf "${BLUE}"
        printf "[1/4] Copy and run commands in Raspberry pi ${RASP_IP} \n"
        printf "${NC}"
        printf "Running commands in localmachine \n"
        scp -i ~/.ssh/id_ed25519 pi4_server.sh ${RASP_ADDR}:/home/${RASP_USER}/
        ssh -i ~/.ssh/id_ed25519 ${RASP_ADDR} 'sh pi4_server.sh start > /dev/null 2>&1 &'

        sleep 2

        printf "${BLUE}"
        printf "[2/4] Mounting necessary driver for usb-ip ... \n"
        printf "${NC}"
        sudo modprobe vhci-hcd

        printf "${BLUE}"
        printf "[3/4] Connecting existing devices from ip address ... $RASP_IP \n"
        printf "${NC}"
        # DEVICES_AVAILABLE=$(sudo usbip list -r $RASP_IP | grep Intel | cut -d ":" -f 1)
        DEVICES_AVAILABLE=$(usbip list -r $RASP_IP | grep -B 1 ": /sys" | cut -d ":" -f 1)
        for DEVICE in $DEVICES_AVAILABLE; do
            printf "Attaching to '$RASP_IP' busid '$DEVICE' \n"
            sudo usbip attach -r $RASP_IP -b $DEVICE
        done

        printf "${BLUE}"
        printf "[4/4] Changing access permission of USB devices ... $RASP_IP \n"
        printf "${NC}"
        sleep 3
        DEVICES_CONNECTED=$(ls /dev/ | grep ttyUSB*)
        for DEVICE in $DEVICES_CONNECTED; do
            printf "Changing permission of '$RASP_IP' /dev/$DEVICE \n"
            sudo chmod +777 /dev/$DEVICE
        done

        printf "${GREEN}"
        printf "Attached all hardwares\n"
        printf "${NC}"

        printf "${BLUE}"
        printf "Start the program to acquire data from devices \n"
        printf "Start the program to acquire data from realsense \n"
        printf "${NC}"

        printf "${GREEN}"
        printf "================================================================================\n"
        printf "${NC}"

    elif [ "${MODE}" = "stop" ]; then

        printf "${GREEN}"
        printf "================================================================================\n"
        printf "${NC}"

        printf "${GREEN}"
        printf "Detaching all hardwares\n"
        printf "${NC}"

        printf "${BLUE}"
        printf "Detach all devices \n"
        printf "${NC}"
        IPS_AVAILABLE=($(sudo usbip port | grep usbip | cut -d ":" -f 2 | cut -d "/" -f 3))
        PORTS_AVAILABLE=($(sudo usbip port | grep usbip -B3 | grep Port | cut -d ":" -f 1 | cut -d " " -f 2))
        for i in "${!PORTS_AVAILABLE[@]}"; do
            if [ "${IPS_AVAILABLE[i]}" = "${RASP_IP}" ]; then
                printf "${IPS_AVAILABLE[i]} :: ${PORTS_AVAILABLE[i]}\n"
                printf "Detaching port '${PORTS_AVAILABLE[i]}' \n"
                sudo usbip detach -p ${PORTS_AVAILABLE[i]}
            fi
        done

        printf "${BLUE}"
        printf "Running commands in RaspberryPi4 ${RASP_IP} \n"
        printf "${NC}"
        scp -i ~/.ssh/id_ed25519 pi4_server.sh ${RASP_ADDR}:/home/${RASP_USER}/
        ssh -i ~/.ssh/id_ed25519 ${RASP_ADDR} 'sh pi4_server.sh stop'

        printf "${GREEN}"
        printf "Detached all hardwares\n"
        printf "${NC}"

        printf "${GREEN}"
        printf "================================================================================\n"
        printf "${NC}"

    else

        printf "${RED}"
        printf "argument 1 is expected to be : {start/stop}\n"
        printf "${NC}"
        exit 1

    fi

else

    printf "${RED}"
    printf "2 arguments are expected : {start/stop}, {ip_address}, {username}\n"
    printf "${NC}"
    exit 1

fi
