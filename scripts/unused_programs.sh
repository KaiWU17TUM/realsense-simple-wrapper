sudo apt-get purge geany thonny chromium* wiringpi pigpio vlc -y
sudo apt autoremove -y

rm -rfv /home/realsense/MagPi
sudo rm /usr/share/raspi-ui-overrides/applications/vlc.desktop