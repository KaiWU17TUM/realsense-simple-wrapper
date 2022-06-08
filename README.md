# RealSense in Python

This is a repository to run realsense camera devices (RS) using python.

## Installing RS with python wrapper

The python wrapper/package to run RS can be obtained through [pip](https://pypi.org/project/pyrealsense/) or compiled from [source](https://github.com/IntelRealSense/librealsense). Once installed the python wrapper can be called using `import pyrealsense` .

## Running RS in python

The code in [realsense](realsense) folder uses the [pyrealsense2](https://pypi.org/project/pyrealsense/) python package to stream images/frames from realsense devices.

| File                                                                 | Details                                                                                             |
| :------------------------------------------------------------------- | :-------------------------------------------------------------------------------------------------- |
| [realsense_run_devices.py](realsense/realsense_run_devices.py)       | Runs X realsense devices that are connected to the PC.                                              |
| [realsense_wrapper.py](realsense/realsense_wrapper.py)               | Contains the _RealsenseWrapper_ class that contains all the functions to run the realsense devices. |
| [realsense_device_manager.py](realsense/realsense_device_manager.py) | Helper functions from the official realsense repo.                                                  |
|                                                                      |                                                                                                     |

## Running RS in RaspberryPi-4B

- The official guide includes an open source ethernet networking [guide](https://dev.intelrealsense.com/docs/open-source-ethernet-networking-for-intel-realsense-depth-cameras) to stream RS over a network. It contains an image for (kernel-patched) Raspberrypi-OS built with RS-SDK-V2.34 . 

- The SDK can be upgraded by following the steps shown [here](https://github.com/datasith/Ai_Demos_RPi/wiki/Raspberry-Pi-4-and-Intel-RealSense-D435). Once pre-installation steps are done, this [script](scripts/install_realsense.sh) can be used to run the installation steps (except OpenGL and VNC).

-  Kernel patch for Raspberrypi
This [guide](https://github.com/NobuoTsukamoto/realsense_examples/blob/master/doc/installation_raspberry_pi_64.md) shows how to build/patch/compile linux for raspberrypi for RS.

## Good to know

### 1. [Sensor timestamp](https://github.com/IntelRealSense/librealsense/issues/2188)
- SENSOR_TIMESTAMP: Device clock / sensor starts taking the image
- FRAME_TIMESTAMP: Device clock / frame starts to transfer to the driver
- BACKEND_TIMESTAMP: PC clock / frame starts to transfer to the driver
- TIME_OF_ARRIVAL: PC clock / frame is transfered to the driver

### 2. [HW Sync](https://dev.intelrealsense.com/docs/external-synchronization-of-intel-realsense-depth-cameras)
The realsense can be syncroized using an external sync cable so that the cameras captures at the exact same timestamp.

### 3. Streaming using *rs-server* and viewing with *realsense-viewer*
The command `rs-server` needs to be run on the host connected to a RS to stream frames from the RS. The RS library on the host server needs to be compiled with `-DBUILD_NETWORK_DEVICE=ON` [flag](https://github.com/IntelRealSense/librealsense/issues/7123) in order for the `rs-server` to work.

Once the `rs-server` on the host is running `realsense-viewer` can be used to view the stream on a client that is connected to the host in the same network. Currently there is a [bug](https://github.com/IntelRealSense/librealsense/issues/9971) with SDK-V2.5 where the depth values are not streamed properly and stream settings in the realsense-viewer GUI cannot be changed (keeps crashing). [SDK-V2.47](https://github.com/IntelRealSense/librealsense/releases/tag/v2.47.0https://github.com/IntelRealSense/librealsense/releases/tag/v2.47.0) can be used instead.

### 4. `-DFORCE_LIBUVC=true` vs `-DFORCE_RSUSB_BACKEND=true`
The former is an old flag of the latter one [link](https://github.com/IntelRealSense/librealsense/issues/7144).

### 5. Metadata extraction
The extraction of metadata is supported in windows but requires kernel patching for linux OS. See [here](https://github.com/IntelRealSense/librealsense/blob/master/doc/frame_metadata.md#os-support) and [here](https://github.com/IntelRealSense/librealsense/issues/7039).
