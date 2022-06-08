#!/bin/sh
# Based on : https://github.com/datasith/Ai_Demos_RPi/wiki/Raspberry-Pi-4-and-Intel-RealSense-D435

cd ~
git clone --depth=1 -b v3.10.0 https://github.com/google/protobuf.git
cd protobuf
./autogen.sh
./configure
make -j3
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

cd ~
wget https://github.com/PINTO0309/TBBonARMv7/raw/master/libtbb-dev_2018U2_armhf.deb
sudo dpkg -i ~/libtbb-dev_2018U2_armhf.deb
sudo ldconfig
rm libtbb-dev_2018U2_armhf.deb

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
make -j3
# sudo make install

export PYTHONPATH=$PYTHONPATH:/home/realsense/librealsense/build

cd ~
source ~/.bashrc

sudo apt-get update
sudo apt-get install libopencv-dev -y
sudo apt-get install libatlas-base-dev -y

# sudo rm -rf /usr/lib/python3/dist-packages/numpy
# sudo rm -rf /usr/lib/python3/dist-packages/numpy-1.16.2.egg-info/

pip3 install opencv-python
pip3 install -U numpy
