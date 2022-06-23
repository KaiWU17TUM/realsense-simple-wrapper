#!/bin/bash -xe

# Based on : https://github.com/IntelRealSense/librealsense/raw/master/scripts/libuvc_installation.sh
GREEN='\033[1;32m'
BLUE='\033[0;36m'
NC='\033[0m'

printf "\n${GREEN}================================================================================\n"
printf "Installing librealsense with network support\n"
printf "================================================================================${NC}\n"

if [ $# -eq 1 ]; then
	INSTALLATION_DIR=${1}
else
	INSTALLATION_DIR=~
fi
echo "Installing librealsense in : ${INSTALLATION_DIR}\n"

#Locally suppress stderr to avoid raising not relevant messages
exec 3>&2
exec 2>/dev/null
con_dev=$(ls /dev/video* | wc -l)
exec 2>&3

if [ $con_dev -ne 0 ]; then
	echo -e "\e[32m"
	read -p "Remove all RealSense cameras attached. Hit any key when ready"
	echo -e "\e[0m"
fi

# Checks
lsb_release -a
printf "Kernel version $(uname -r)\n"

printf "\n${GREEN}================================================================================\n"
printf "Update packages\n"
printf "================================================================================${NC}\n"

sudo apt-get update --allow-releaseinfo-change && sudo apt-get dist-upgrade -y

MAJOR_VERSION=$(uname -r | awk -F '.' '{print $1}')
MINOR_VERSION=$(uname -r | awk -F '.' '{print $2}')
if [ $MAJOR_VERSION -ge 5 ] && [ $MINOR_VERSION -ge 10 ] || [ $MAJOR_VERSION -ge 6 ]; then
	printf "Linux Kernel : $(uname -r) is >= 5.10"
else
	printf "Linux Kernel : $(uname -r) is < 5.10"
	printf "But it has been updated in the previous command."
	printf "Rebooting..."
	sudo reboot
fi

# increase swap space
if [ $(sudo swapon --show | wc -l) -eq 0 ]; then
	echo "No swapon - setting up 1Gb swap file"
	sudo fallocate -l 8G /swapfile
	sudo chmod 600 /swapfile
	sudo mkswap /swapfile
	sudo swapon /swapfile
	sudo swapon --show
fi

sudo apt-get install -y \
	python3-pip \
	automake \
	libtool \
	git \
	cmake \
	libusb-1.0-0-dev \
	libx11-dev \
	xorg-dev \
	libglu1-mesa-dev
# vim \

printf "\n${GREEN}================================================================================\n"
printf "Get librealsense source code\n"
printf "================================================================================${NC}\n"

cd ${INSTALLATION_DIR}
sudo rm -rf ./librealsense
git clone --depth=1 -b v2.50.0 https://github.com/IntelRealSense/librealsense.git
sudo cp ${INSTALLATION_DIR}/librealsense/config/99-realsense-libusb.rules /etc/udev/rules.d/

# Update udev rule
sudo bash ${PWD}/install_realsense_pi4_udev.sh

# Update library paths
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
. ~/.bashrc

printf "\n${GREEN}================================================================================\n"
printf "Compile librealsense\n"
printf "================================================================================${NC}\n"

cd ${INSTALLATION_DIR}/librealsense

# fixing python module import error for V2.50.0
sed -i '217s/.*/    pybind11_add_module(pyrealsense2_net SHARED pyrs_net.cpp)/' wrappers/python/CMakeLists.txt
sed -i '218s/.*/    target_link_libraries(pyrealsense2_net PRIVATE realsense2-net)/' wrappers/python/CMakeLists.txt
sed -i '219s/.*/    set_target_properties(pyrealsense2_net PROPERTIES FOLDER Wrappers\/python)/' wrappers/python/CMakeLists.txt
sed -i '220s/.*/    set_target_properties(pyrealsense2_net PROPERTIES/' wrappers/python/CMakeLists.txt
sed -i '225s/.*/    install(TARGETS pyrealsense2_net/' wrappers/python/CMakeLists.txt

mkdir build && cd build
# cmake .. -DBUILD_EXAMPLES=true -DCMAKE_BUILD_TYPE=Release -DFORCE_LIBUVC=true
cmake .. \
	-DCMAKE_BUILD_TYPE=Release \
	-DBUILD_GRAPHICAL_EXAMPLES=OFF \
	-DBUILD_NETWORK_DEVICE=ON \
	-DBUILD_PYTHON_BINDINGS=bool:true \
	-DPYTHON_EXECUTABLE=$(which python3) \
	-DBUILD_SHARED_LIBS:BOOL=ON \
	-DFORCE_RSUSB_BACKEND=ON
make -j$(($(nproc) - 1))
# make -j1
sudo make install

printf "\n${GREEN}================================================================================\n"
printf "OpenCV and Numpy\n"
printf "================================================================================${NC}\n"

cd ~
. ~/.bashrc

# OpenCV and Numpy
sudo apt-get install -y \
	libopencv-dev \
	libatlas-base-dev

pip3 install opencv-python
pip3 install -U numpy

printf "\n${GREEN}================================================================================\n"
printf "Installed librealsense in with network support\n"
printf "================================================================================${NC}\n"
