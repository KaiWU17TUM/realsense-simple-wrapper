#! /bin/sh

echo nameserver 8.8.8.8 | sudo tee /etc/resolv.conf

apt-get update \
    && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    libssl-dev \
    libusb-1.0-0-dev \
    pkg-config \
    libgtk-3-dev \
    libglfw3-dev \
    libgl1-mesa-dev \
    libglu1-mesa-dev \
    curl \
    python3 \
    python3-dev \
    ca-certificates

cd /usr/local/src

git clone --depth=1 -b v2.50.0 https://github.com/IntelRealSense/librealsense.git

sed -i '217s/.*/    pybind11_add_module(pyrealsense2_net SHARED pyrs_net.cpp)/' /usr/local/src/librealsense/wrappers/python/CMakeLists.txt
sed -i '218s/.*/    target_link_libraries(pyrealsense2_net PRIVATE realsense2-net)/' /usr/local/src/librealsense/wrappers/python/CMakeLists.txt
sed -i '219s/.*/    set_target_properties(pyrealsense2_net PROPERTIES FOLDER Wrappers\/python)/' /usr/local/src/librealsense/wrappers/python/CMakeLists.txt
sed -i '220s/.*/    set_target_properties(pyrealsense2_net PROPERTIES/' /usr/local/src/librealsense/wrappers/python/CMakeLists.txt
sed -i '225s/.*/    install(TARGETS pyrealsense2_net/' /usr/local/src/librealsense/wrappers/python/CMakeLists.txt

cd /usr/local/src/librealsense \
    && mkdir build && cd build \
    && cmake .. \
    -DCMAKE_C_FLAGS_RELEASE="${CMAKE_C_FLAGS_RELEASE} -s" \
    -DCMAKE_CXX_FLAGS_RELEASE="${CMAKE_CXX_FLAGS_RELEASE} -s" \
    -DCMAKE_INSTALL_PREFIX=/usr/local \
    -DBUILD_EXAMPLES=ON \
    -DBUILD_GRAPHICAL_EXAMPLES=OFF \
    -DBUILD_PYTHON_BINDINGS:bool=true \
    -DPYTHON_EXECUTABLE=$(which python3) \
    -DCMAKE_BUILD_TYPE=Release \
    -DBUILD_SHARED_LIBS:BOOL=ON \
    -DBUILD_NETWORK_DEVICE=ON \
    -DFORCE_RSUSB_BACKEND=ON \
    && make -j$(($(nproc)-1)) all \
    && make install

apt-get update \
    && apt-get install -y --no-install-recommends \
    libusb-1.0-0 \
    udev \
    apt-transport-https \
    software-properties-common \
    libopencv-dev

cp /usr/local/src/librealsense/config/99-realsense-libusb.rules /etc/udev/rules.d/
cp /usr/local/src/librealsense/wrappers/python/pyrealsense2/__init__.py /usr/lib/python3/dist-packages/pyrealsense2

export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib
