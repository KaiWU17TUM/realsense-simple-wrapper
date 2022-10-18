# Running RS in RaspberryPi-4B

To run RS in raspberrypi 3/4, the linux kernel needs to be patched. This can be done either:

1. Compile from source : The official github repo contains [scripts](https://github.com/IntelRealSense/librealsense/tree/master/scripts) to do so, but they are not raspberrypi specific. A raspberrypi-4 speific instructions for patching can be found in this [guide](https://github.com/NobuoTsukamoto/realsense_examples/blob/master/doc/installation_raspberry_pi_64.md).

2. Flash a pre-build image : In one of he official [guide](https://dev.intelrealsense.com/docs/open-source-ethernet-networking-for-intel-realsense-depth-cameras) there is an image for (kernel-patched) Raspberrypi-OS built with RS-SDK-V2.34 . (update 18.10.22: img will not work with raspberrypi firmware from 2022++)

**Increase swapsize according to the [guide](https://github.com/datasith/Ai_Demos_RPi/wiki/Raspberry-Pi-4-and-Intel-RealSense-D435).** (see below)

## Setting up RaspberryPi-4B from scratch after (2.)

1. Check OS  
cat /etc/os-release  

2. Update raspberrypi config  
sudo raspi-config  
sudo reboot  

3. Update swap  
sudo nano /etc/init.d/dphys-swapfile   
sudo nano /etc/dphys-swapfile   
sudo /etc/init.d/dphys-swapfile restart swapon -s  
sudo reboot  

4. Update OS  
sudo apt-get update --allow-releaseinfo-change  
sudo apt-get upgrade  
sudo reboot  

5. Update pi4 firmware  
sudo rpi-eeprom-update (will show if update is available)  
sudo rpi-eeprom-update -d -a (upgrade firmware)  
sudo reboot  

6. WIFI - change position of `wext` + credentials  
sudo nano /etc/wpa_supplicant/functions.sh  
sudo nano /lib/dhcpcd/dhcpcd-hooks/10-wpa_supplicant  
sudo nano /etc/wpa_supplicant/wpa_supplicant.conf  
sudo reboot  

## Installing RS
RS SDK can be installed/upgraded by following the steps shown in this [guide](https://github.com/datasith/Ai_Demos_RPi/wiki/Raspberry-Pi-4-and-Intel-RealSense-D435). This [script](pi4_scripts/install_realsense_pi4.sh) follows the steps and performs the installation (except OpenGL and VNC).
