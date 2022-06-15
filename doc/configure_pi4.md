# Configuring raspberrypi4 to run realsense

Realsense needs kernel patching to run as mentioned in the main [README](README.md).

The following are some main steps for configuring the raspberrypi.

## Flash the image
- https://dev.intelrealsense.com/docs/open-source-ethernet-networking-for-intel-realsense-depth-cameras

## Main raspberrypi configs
- `sudo raspi-config`
- Network Options > Hostname
- Network Options > Wi-Fi (if wifi does not use wpa-eap)
- Localisation Options > Change Locale (if want to change from en_us.UTF-8)
- Localisation Options > Change Timezone
- Localisation Options > Change Keyboard Layout
- Localisation Options > Change Wifi Country
- Advanced Options > Expand Filesystem
- Advanced Options > Expand Filesystem
- save > reboot > yes

## Updating swap file
- `sudo nano /etc/dphys-swapfile` => 2048mb
- `sudo /etc/init.d/dphys-swapfile restart swapon -s`
- `sudo reboot`

## Adding wifi credentials
By default, raspberrypi wifi only connects to normal hotspots (WPA). To connect to enterprise version (WPA-EAP), the configurations need to be manually configured. 
- `sudo nano /etc/wpa_supplicant/wpa_supplicant.conf`
  ```
  ctrl_interface=DIR=/var/run/wpa_supplicant GROUP=netdev
  update_config=1
  country=DE

  network={
    ssid="<NETWORK-ID>"
    key_mgmt=WPA-EAP
    eap=PEAP
    identity="<USERNAME>"
    password="<PASSWORD>"
  }
  ```

Additionally, the *WEXT* driver needs to be the used for the WPA-EAP to work (see [here](https://raspberrypi.stackexchange.com/questions/112062/raspbian-buster-lite-couldnt-communicate-with-wpa-supplicant)).
- `sudo nano /etc/wpa_supplicant/functions.sh`
- `sudo nano /lib/dhcpcd/dhcpcd-hooks/10-wpa_supplicant`
- `sudo reboot`

## Updating
- `sudo apt-get update --allow-releaseinfo-change`
- `sudo apt-get upgrade`
- `sudo apt-get dist-upgrade`
- `sudo apt-get auto-remove`

## Removing unused apps
- `sudo apt-get purge geany thonny chromium* wiringpi pigpio vlc`
- `sudo apt autoremove`
- `sudo rm -rfv /home/realsense/MagPi`
- `sudo rm /usr/share/raspi-ui-overrides/applications/vlc.desktop`