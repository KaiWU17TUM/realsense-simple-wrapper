#!/bin/bash

GREEN='\033[1;32m'
BLUE='\033[0;36m'
NC='\033[0m'

MODE=$1
IP_RASP=$2
RASP_ADDR='pi@'${IP_RASP}

if [ $# -eq 2 ]; then

    if [ "${MODE}" = "start" ]; then

        printf "\n${GREEN}================================================================================\n"
        printf "Start initialization to setup hardwares in mocked ICU room\n"
        printf "================================================================================${NC}\n"

        printf "\n${BLUE}********************************************************************************\n"
        printf "STEP 1 : Copy and run commands in Raspberry pi ${IP_RASP} ${NC}\n"
        scp -i ~/.ssh/id_ed25519 server ${RASP_ADDR}:/home/pi/
        ssh -i ~/.ssh/id_ed25519 ${RASP_ADDR} 'sh pi4_server.sh start  > /dev/null 2>&1 &'

        printf "${BLUE}Running commands in localmachine ${NC}\n"

        sleep 2

        printf "\n${BLUE}********************************************************************************\n"
        printf "STEP 2 : Mounting necessary driver for usb-ip ... ${NC}\n"
        sudo modprobe vhci-hcd

        printf "\n${BLUE}********************************************************************************\n"
        printf "STEP 3 : Connecting existing devices from ip address ... $IP_RASP ${NC}\n"
        DEVICES_AVAILABLE=$(sudo usbip list -r $IP_RASP | grep Intel | cut -d ":" -f 1)

        for DEVICE in $DEVICES_AVAILABLE; do
            printf "$DEVICE \n"
            sudo usbip attach -r $IP_RASP -b $DEVICE
        done

        printf "\n${BLUE}********************************************************************************\n"
        printf "STEP 4 : Changing access permission of USB devices ... $IP_RASP ${NC}\n"
        sleep 3
        DEVICES_CONNECTED=$(ls /dev/ | grep ttyUSB*)
        for DEVICE in $DEVICES_CONNECTED; do
            printf "$DEVICE \n"
            sudo chmod +777 /dev/$DEVICE
        done

        printf "\n${GREEN}================================================================================\n"
        printf "Finish initialization\n"
        printf "================================================================================${NC}\n\n"

        printf "${BLUE}Start the program to acquire data from devices ${NC}\n"
        printf "${BLUE}Start the program to acquire data from realsense ${NC}\n\n"

    elif [ "${MODE}" = "stop" ]; then

        printf "\n${GREEN}================================================================================\n"
        printf "Detaching all hardwares in mocked ICU room\n"
        printf "================================================================================${NC}\n"

        printf "${BLUE}Running commands in localmachine ${NC}\n"

        printf "${BLUE}Detach all devices ${NC}\n"
        sudo usbip detach -p 00

        printf "${BLUE}Running commands in Raspberry pi ${IP_RASP} ${NC}\n"
        scp -i ~/.ssh/id_ed25519 server ${RASP_ADDR}:/home/pi/
        ssh -i ~/.ssh/id_ed25519 ${RASP_ADDR} 'sh pi4_server.sh stop'

        printf "\n${GREEN}================================================================================\n"
        printf "Detached all hardwares in mocked ICU room\n"
        printf "================================================================================${NC}\n\n"

    else

        echo "argument 1 is expected to be : {start/stop}"
        exit 1

    fi

else

    echo "2 arguments are expected : {start/stop}, {ip_address}"
    exit 1

fi
