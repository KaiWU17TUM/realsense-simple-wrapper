#!/bin/sh
# Based on : https://github.com/datasith/Ai_Demos_RPi/wiki/Raspberry-Pi-4-and-Intel-RealSense-D435
# udev rule
udevadm control --reload-rules && udevadm trigger
