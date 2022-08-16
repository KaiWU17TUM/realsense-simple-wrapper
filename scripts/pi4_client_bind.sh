#!/bin/bash

RASP_IP=$1
RASP_ADDR='realsense@'${RASP_IP}

ssh -i ~/.ssh/id_ed25519 ${RASP_ADDR} 'sh pi4_server.sh start > /dev/null 2>&1 &'

DEVICES_AVAILABLE=$(usbip list -r $RASP_IP | grep -B 1 ": /sys" | cut -d ":" -f 1)
for DEVICE in $DEVICES_AVAILABLE; do
	printf "Attaching to '$RASP_IP' busid '$DEVICE' \n"
	sudo usbip attach -r $RASP_IP -b $DEVICE
done
