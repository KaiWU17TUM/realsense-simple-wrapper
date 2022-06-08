#!/bin/sh
# Based on : https://github.com/datasith/Ai_Demos_RPi/wiki/Raspberry-Pi-4-and-Intel-RealSense-D435

# Locally suppress stderr to avoid raising not relevant messages
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
echo "Kernel version $(uname -r)"

echo "\n--------------------------------------------------------------------------------"
echo "Update packages"
echo "--------------------------------------------------------------------------------\n"

sudo apt-get update && sudo apt-get dist-upgrade -y
sudo apt-get install -y \
  automake \
  libtool \
  cmake \
  libusb-1.0-0-dev \
  libx11-dev \
  xorg-dev \
  libglu1-mesa-dev
# vim \

echo "\n--------------------------------------------------------------------------------"
echo "Get librealsense source code"
echo "--------------------------------------------------------------------------------\n"

cd ~/
sudo rm -rf ./librealsense
git clone https://github.com/IntelRealSense/librealsense.git
sudo cp librealsense/config/99-realsense-libusb.rules /etc/udev/rules.d/

# Update udev rule
sudo bash ~/realsense-wrapper-python/scripts/install_realsense_pi4_udev.sh

# Update library paths
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
. ~/.bashrc

echo "\n--------------------------------------------------------------------------------"
echo "Compile protobuff"
echo "--------------------------------------------------------------------------------\n"
# TODO: may not be needed if examples are not built.
cd ~
git clone --depth=1 -b v3.10.0 https://github.com/google/protobuf.git
cd protobuf
./autogen.sh
./configure
make -j$(($(nproc) - 1))
sudo make install
cd python
export LD_LIBRARY_PATH=../src/.libs
python3 setup.py build --cpp_implementation
python3 setup.py test --cpp_implementation
sudo python3 setup.py install --cpp_implementation
export PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION=cpp
export PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION_VERSION=3
sudo ldconfig
protoc --version

echo "\n--------------------------------------------------------------------------------"
echo "Install libtbb package"
echo "--------------------------------------------------------------------------------\n"

cd ~
wget https://github.com/PINTO0309/TBBonARMv7/raw/master/libtbb-dev_2018U2_armhf.deb
sudo dpkg -i ~/libtbb-dev_2018U2_armhf.deb
sudo ldconfig
rm libtbb-dev_2018U2_armhf.deb

echo "\n--------------------------------------------------------------------------------"
echo "Compile librealsense"
echo "--------------------------------------------------------------------------------\n"

cd ~/librealsense
mkdir build && cd build
# cmake .. -DBUILD_EXAMPLES=true -DCMAKE_BUILD_TYPE=Release -DFORCE_LIBUVC=true
cmake .. \
  -DCMAKE_BUILD_TYPE=Release \
  -DBUILD_NETWORK_DEVICE=ON \
  -DBUILD_PYTHON_BINDINGS=bool:true \
  -DPYTHON_EXECUTABLE=$(which python3) \
  -DBUILD_SHARED_LIBS:BOOL=ON \
  -DFORCE_RSUSB_BACKEND=ON
make -j$(($(nproc) - 1))
# sudo make install

# export PYTHONPATH=$PYTHONPATH:/home/realsense/librealsense/build

echo "\n--------------------------------------------------------------------------------"
echo "OpenCV and Numpy"
echo "--------------------------------------------------------------------------------\n"

cd ~
. ~/.bashrc

# OpenCV and Numpy
sudo apt-get install -y \
  libopencv-dev \
  libatlas-base-dev

pip3 install opencv-python
pip3 install -U numpy
