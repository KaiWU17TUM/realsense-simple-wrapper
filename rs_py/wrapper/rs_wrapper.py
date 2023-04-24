# Network part is based on :
# https://github.com/IntelRealSense/librealsense/blob/master/wrappers/python/examples/net_viewer.py

import argparse
import cv2
import json
import numpy as np
import os
import time
import sys

from datetime import datetime
from functools import partial
from typing import Optional, Tuple, Union, Any

from rs_py.wrapper.rs_utils import CalibrationConfig
from rs_py.wrapper.rs_utils import Device
from rs_py.wrapper.rs_utils import StreamConfig
from rs_py.wrapper.rs_utils import DefaultStoragePaths
from rs_py.wrapper.rs_utils import StoragePaths
from rs_py.wrapper.rs_utils import print_no_device_enabled
from rs_py.wrapper.rs_utils import read_metadata
from rs_py.wrapper.rs_utils import print_rs2_device_infos
from rs_py.wrapper.rs_utils import print_camera_temperature
from rs_py.wrapper.rs_utils import check_if_color_depth_frames_are_valid

from rs_py.wrapper.realsense_device_manager import enumerate_connected_devices
from rs_py.wrapper.realsense_device_manager import post_process_depth_frame
from rs_py.utility import str2bool
from rs_py.utility import printout

from rs_py import rs
from rs_py import rsnet


RESET_LIMIT = 5
RESET_LIMIT_BUFFER = 1


class RealsenseWrapper:
    """Wrapper to run multiple realsense cameras.

    Code is written based on "realsense_device_manager.py" . Currently the
    code supports only reading depth and color images. Reading of IR stream
    is not implemented.

    """

    def __init__(self,
                 arg: argparse.Namespace,
                 device_sn: Optional[str] = None,
                 ctx: Optional[rs.context] = None) -> None:
        """Initializes the RealsenseWrapper class object.

        Args:
            arg (argparse.Namespace): argument parsed from cli.
            device_sn (Optional[str], optional): If not None, only the device
                with this sn will be used. Defaults to None.
            ctx
        """

        self._colorizer = rs.colorizer()

        self.arg = arg
        self.verbose = arg.rs_verbose

        self.ctx = ctx if ctx is not None else rs.context()
        # if given, we assume only 1 device is being used.
        self.single_device_sn = device_sn
        # list of Device class that will be used.
        self.enabled_devices = {}
        # # timestamp
        self.timestamp_mode = None
        # # calibration
        self.calib_data = CalibrationConfig()
        # # rs align method
        # align depth to color frame
        self._align = rs.align(rs.stream.color)
        # # configurations
        self._rs_cfg = {}
        self.stream_config_color = StreamConfig(
            rs.stream.color, arg.rs_image_width, arg.rs_image_height,
            arg.rs_color_format, arg.rs_fps
        )
        self.stream_config_depth = StreamConfig(
            rs.stream.depth, arg.rs_image_width, arg.rs_image_height,
            arg.rs_depth_format, arg.rs_fps
        )
        # # get list of connected realsense
        self.query_available_devices()
        # # Save paths
        if arg.rs_save_data:
            self.storage_paths = StoragePaths(
                [device_sn for device_sn, _ in self.available_devices],
                arg.rs_save_path)
        else:
            self.storage_paths = DefaultStoragePaths()

        # # Data holder
        # {serial_number: {data_type: data}}.
        #  data_type = color, depth, timestamp, calib
        self.frames = {}

        # for display and save option
        self.key = -1

        # # internal variables
        self.empty_frame_received_timers = {}
        self.internal_timestamp = {sn: 0 for sn, _ in self.available_devices}
        self.temperature_log_interval = 300
        # [time elapsed, current time]
        # if time elapsed > RESET_LIMIT reset device.
        self.poll_counter_per_dev = {sn: [0, 0]
                                     for sn, _ in self.available_devices}
        # Saves the last timestamp from device.
        self.last_timestamps = {
            sn: {'color_timestamp': {'timestamp': 0, 'count': 0},
                 'depth_timestamp': {'timestamp': 0, 'count': 0}}
            for sn, _ in self.available_devices
        }

    def query_available_devices(self):
        if self.arg.rs_ip is not None:
            # No multi ethernet support.
            # https://github.com/IntelRealSense/librealsense/issues/6376
            printout("Network mode", 'i')
            self.network = True
            self.available_devices = []
            dev = rsnet.net_device(self.arg.rs_ip)
            self.available_devices.append(
                (dev.get_info(rs.camera_info.serial_number), None))
            dev.add_to(self.ctx)
            printout(f"Connected to {self.arg.rs_ip}", 'i')
        else:
            printout("Local mode", 'i')
            self.network = False
            if self.single_device_sn is None:
                self.available_devices = enumerate_connected_devices(self.ctx)
                if self.arg.rs_use_one_dev_only:
                    self.available_devices = self.available_devices[0:1]
            else:
                self.available_devices = [(self.single_device_sn, 'D400')]
        self.available_devices = sorted(self.available_devices)

    def check_if_device_is_enabled(self, device_sn: str):
        return device_sn in self.enabled_devices

    def configure_stream(
            self,
            device_sn: str,
            stream_config_depth: Optional[StreamConfig] = None,
            stream_config_color: Optional[StreamConfig] = None) -> None:
        """Defines per device stream configurations.

        device('001622070408')
        device('001622070717')

        Args:
            device_sn (str, optional): serial number. Defaults to None.
        """
        if device_sn is not None and device_sn not in self._rs_cfg:
            cfg = rs.config()
            if stream_config_depth is not None:
                cfg.enable_stream(**stream_config_depth.data)
            if stream_config_color is not None:
                cfg.enable_stream(**stream_config_color.data)
            if not self.network:
                cfg.enable_device(device_sn)
            check = cfg.can_resolve(self.enabled_devices[device_sn].pipeline)
            if self.verbose:
                printout(f"'cfg' usable with 'pipeline' : {check}", 'i')
            self._rs_cfg[device_sn] = cfg

    def configure_color_sensor(self, device_sn: str):
        self.check_if_device_is_enabled(device_sn)
        sensor = self.enabled_devices[device_sn].color_sensor
        if self.arg.rs_autoexposure:
            sensor.set_option(rs.option.enable_auto_exposure, 1.0)
            sensor.set_option(rs.option.auto_exposure_priority, 0.0)
        else:
            sensor.set_option(rs.option.enable_auto_exposure, 0.0)
            sensor.set_option(rs.option.exposure, 100.0)
            if self.verbose:
                printout(f"{device_sn} color autoexposure off", 'i')
        self.enabled_devices[device_sn].color_sensor = sensor

    def configure_depth_sensor(self, device_sn: str):
        self.check_if_device_is_enabled(device_sn)
        sensor = self.enabled_devices[device_sn].depth_sensor
        if self.arg.rs_autoexposure:
            sensor.set_option(rs.option.enable_auto_exposure, 1.0)
            limit = sensor.get_option(rs.option.auto_exposure_limit)
            if self.verbose:
                printout(f"{device_sn} depth auto_exposure_limit {limit}", 'i')
        else:
            sensor.set_option(rs.option.enable_auto_exposure, 0.0)
            sensor.set_option(rs.option.exposure, 1000.0)
            if self.verbose:
                printout(f"{device_sn} depth autoexposure off", 'i')
        self.enabled_devices[device_sn].depth_sensor = sensor

    def configure_color_depth_sensor(self, device_sn: str):
        self.check_if_device_is_enabled(device_sn)
        _pipeline_profile = self.enabled_devices[device_sn].pipeline_profile
        _sensors = _pipeline_profile.get_device().query_sensors()
        for _sensor in _sensors:
            if _sensor.is_color_sensor():
                sensor = _sensor.as_color_sensor()
                self.enabled_devices[device_sn].color_sensor = sensor
                self.configure_color_sensor(device_sn)
                if self.verbose:
                    printout(f"{device_sn} color_sensor is available...", 'i')
            elif _sensor.is_depth_sensor():
                sensor = _sensor.as_depth_sensor()
                self.enabled_devices[device_sn].depth_sensor = sensor
                self.configure_depth_sensor(device_sn)
                if self.verbose:
                    printout(f"{device_sn} depth_sensor is available...", 'i')

    def configure_ir_emitter(self):
        """Sets the power of the IR laser. If power value is too high the
        rs connection will crash.

        https://github.com/IntelRealSense/librealsense/issues/1258
        """
        if len(self.enabled_devices) > 0:
            for device_sn, _ in self.enabled_devices.items():
                self.configure_ir_emitter_device(device_sn)
        else:
            print_no_device_enabled("configure_ir_emitter()")

    def configure_ir_emitter_device(self, device_sn: str):
        self.check_if_device_is_enabled(device_sn)
        sensor = self.enabled_devices[device_sn].depth_sensor
        if sensor.supports(rs.option.emitter_enabled):
            sensor.set_option(rs.option.emitter_enabled,
                              1 if self.arg.rs_enable_ir_emitter else 0)
            ir_range = sensor.get_option_range(rs.option.laser_power)
            if self.arg.rs_ir_emitter_power + 10 > ir_range.max:
                sensor.set_option(rs.option.laser_power, ir_range.max)
            elif self.arg.rs_ir_emitter_power - 10 < ir_range.min:
                sensor.set_option(rs.option.laser_power, ir_range.min)
            else:
                sensor.set_option(rs.option.laser_power,
                                  self.arg.rs_ir_emitter_power)
            if self.verbose:
                printout(f"{device_sn} ir emitter enabled...", 'i')
        self.enabled_devices[device_sn].depth_sensor = sensor

    def query_timestamp_mode(self, device_sn: str):
        pipeline_profile = self.enabled_devices[device_sn].pipeline_profile
        pipeline = self.enabled_devices[device_sn].pipeline
        wait_flag = True
        while wait_flag:
            streams = pipeline_profile.get_streams()
            frameset = pipeline.poll_for_frames()
            if frameset.size() == len(streams):
                wait_flag = False
        fmv = rs.frame_metadata_value
        if frameset.supports_frame_metadata(fmv.sensor_timestamp):
            self.timestamp_mode = fmv.sensor_timestamp
            printout(f"sensor_timestamp is being used..", 'i')
        elif frameset.supports_frame_metadata(fmv.frame_timestamp):
            self.timestamp_mode = fmv.frame_timestamp
            printout(f"frame_timestamp is being used..", 'i')
        # elif frameset.supports_frame_metadata(fmv.backend_timestamp):
        #     self.timestamp_mode = fmv.backend_timestamp
        #     print(f'[INFO] : backend_timestamp is being used...')
        elif frameset.supports_frame_metadata(fmv.time_of_arrival):
            self.timestamp_mode = fmv.time_of_arrival
            printout(f"time_of_arrival is being used..", 'i')
            print(f'[INFO] : time_of_arrival is being used...')
        else:
            # self.storage_paths_per_dev.pop(device_sn)
            printout("Both sensor_timestamp/frame_timestamp "
                     "are not available. Using host time...", 'w')
            self.timestamp_mode = 'host'

    def query_frame_timestamp(self, frame: rs.frame) -> int:
        if self.timestamp_mode is None:
            return -1
        elif self.timestamp_mode == 'host':
            return time.time()
        else:
            return frame.get_frame_metadata(self.timestamp_mode)

    def save_timestamp(self, frame_dict: dict, device_sn: str) -> None:
        """Saves the timestamp.

        Args:
            frame_dict (dict): Dictionary containing framedata.
            storage_paths (StoragePaths): Obj containing the storage paths.
        """
        if not self.storage_paths.save:
            return

        ts_file = os.path.join(self.storage_paths.timestamp[device_sn],
                               'timestamp.txt')
        with open(ts_file, 'a+') as f:
            f.write(f"{self.internal_timestamp[device_sn]}::"
                    f"{frame_dict['color_timestamp']}::"
                    f"{frame_dict['depth_timestamp']}\n")

    def flush_frames(self, num_frames: int = 30) -> None:
        """Dummy capture 'num_frames' frames to give
        autoexposure, etc. a chance to settle.

        Args:
            num_frames (int): Number of dummy frames to skip. Defaults to 30.
        """
        printout(f"Capturing dummy frames...", 'i')
        frames = {}
        for _ in range(num_frames):
            while len(frames) < len(self.enabled_devices.items()):
                for device_sn, dev in self.enabled_devices.items():
                    streams = dev.pipeline_profile.get_streams()
                    frameset = dev.pipeline.poll_for_frames()
                    if frameset.size() == len(streams):
                        frames[device_sn] = {}
        printout(f"Finished capturing dummy frames...", 'i')
        self.poll_counter_per_dev = {sn: [0, 0]  # [time elapsed, current time]
                                     for sn, _ in self.available_devices}

    def query_camera_calib(self) -> None:
        for device_sn, _ in self.enabled_devices.items():
            self.query_camera_calib_device(device_sn)

    def query_camera_calib_device(self, device_sn: str) -> None:
        self.calib_data.device_sn.append(device_sn)
        dev = self.enabled_devices[device_sn]
        profile = dev.pipeline_profile
        # Intrinsics of color & depth frames -----------------------------------
        profile_color = profile.get_stream(rs.stream.color)
        intr_color = profile_color.as_video_stream_profile()
        intr_color = intr_color.get_intrinsics()
        # Fetch stream profile for depth stream
        # Downcast to video_stream_profile and fetch intrinsics
        profile_depth = profile.get_stream(rs.stream.depth)
        intr_depth = profile_depth.as_video_stream_profile()
        intr_depth = intr_depth.get_intrinsics()
        # Extrinsic matrix from color sensor to Depth sensor -------------------
        profile_vid = profile_color.as_video_stream_profile()
        extr = profile_vid.get_extrinsics_to(profile_depth)
        extr_mat = np.eye(4)
        extr_mat[:3, :3] = np.array(extr.rotation).reshape(3, 3)
        extr_mat[:3, 3] = extr.translation
        # Depth scale ----------------------------------------------------------
        # for sensor in profile.get_device().query_sensors():
        #     if sensor.is_depth_sensor():
        depth_sensor = profile.get_device().first_depth_sensor()
        depth_sensor = rs.depth_stereo_sensor(depth_sensor)
        depth_scale = depth_sensor.get_depth_scale()
        depth_baseline = depth_sensor.get_stereo_baseline()
        # Calibrations ---------------------------------------------------------
        self.calib_data.color.append({
            'width': intr_color.width,
            'height': intr_color.height,
            'intrinsic_mat': [intr_color.fx, 0, intr_color.ppx,
                              0, intr_color.fy, intr_color.ppy,
                              0, 0, 1],
            'model': str(intr_color.model),
            'coeffs': intr_color.coeffs,
            'format': str(self.stream_config_color.format),
            'fps': self.stream_config_color.framerate,
        })
        self.calib_data.depth.append({
            'width': intr_depth.width,
            'height': intr_depth.height,
            'intrinsic_mat': [intr_depth.fx, 0, intr_depth.ppx,
                              0, intr_depth.fy, intr_depth.ppy,
                              0, 0, 1],
            'model': str(intr_depth.model),
            'coeffs': intr_depth.coeffs,
            'depth_scale': depth_scale,
            'depth_baseline': depth_baseline,
            'format': str(self.stream_config_depth.format),
            'fps': self.stream_config_color.framerate,
        })
        self.calib_data.T_color_depth.append({
            'rotation': extr.rotation,
            'translation': extr.translation
        })

    def save_calib(self) -> None:
        """Saves camera calibration. """

        if len(self.enabled_devices) == 0:
            printout(f"No devices are enabled...", 'w')
            return

        if not self.storage_paths.save:
            return

        for device_sn, _ in self.enabled_devices.items():
            assert os.path.exists(self.storage_paths.calib[device_sn])
            if os.path.isfile(self.storage_paths.calib[device_sn]):
                save_path = self.storage_paths.calib[device_sn]
            else:
                filename = f'dev{device_sn}_calib.json'
                save_path = os.path.join(
                    self.storage_paths.calib[device_sn], filename)
            self.query_camera_calib_device(device_sn)
            self.calib_data.save(save_path, device_sn)

        printout(f"Saved camera calibration data...", 'i')

    def initialize_pipeline(self) -> rs.pipeline:
        if self.network:
            return rs.pipeline(self.ctx)
        else:
            return rs.pipeline()

    def initialize(self) -> None:
        """Initializes the device pipelines and starts them. """
        for device_sn, _ in self.available_devices:
            self.initialize_device(device_sn)

    def initialize_device(self, device_sn: str) -> None:
        """Initializes a single device pipeline and starts it.

        Args:
            device_sn (str): Device to start
        """
        printout(f"Initializing RealSense devices {device_sn}", 'i')

        # 0. enabled devices
        self.enabled_devices[device_sn] = Device()

        # 1. Pipeline
        self.enabled_devices[device_sn].pipeline = self.initialize_pipeline()

        # 2. configure stream
        self.configure_stream(device_sn=device_sn,
                              stream_config_depth=self.stream_config_depth,
                              stream_config_color=self.stream_config_color)

        # 3. pipeline start
        self.start_device(device_sn)

        # 4. sensors
        self.configure_color_depth_sensor(device_sn)

        # 5. IR for depth
        self.configure_ir_emitter_device(device_sn)

        # 7. Camera info
        print_rs2_device_infos(device_sn, self.verbose)

        # 8. Check which timestamp is available.
        if len(self.enabled_devices) > 0:
            self.query_timestamp_mode(device_sn)

        # 9. Calib
        self.query_camera_calib_device(device_sn)

        printout(f"Initialized RealSense devices {device_sn}", 'i')

    def initialize_depth_sensor_ae(self) -> None:
        """Initializes the device pipelines and starts them for depth ae. """
        for device_sn, _ in self.available_devices:
            self.initialize_depth_sensor_ae_device(device_sn)

    def initialize_depth_sensor_ae_device(self, device_sn: str) -> None:
        """Initializes a single device pipeline and starts it for depth ae.

        Args:
            device_sn (str): Device to start
        """
        printout(f"Initializing RealSense depth sensor AE {device_sn}", 'i')

        # 0. enabled devices
        self.enabled_devices[device_sn] = Device()

        # 1. Pipeline
        self.enabled_devices[device_sn].pipeline = self.initialize_pipeline()

        # 2. configure stream
        # HACK: sensor AE configuration seems to only work for fps = 6
        self.stream_config_color.framerate = 6
        self.stream_config_depth.framerate = 6
        self.configure_stream(device_sn=device_sn,
                              stream_config_depth=self.stream_config_depth,
                              stream_config_color=self.stream_config_color)

        # 3. pipeline start
        self.start_device(device_sn)

        # 4. sensors
        _pipeline_profile = self.enabled_devices[device_sn].pipeline_profile
        _sensors = _pipeline_profile.get_device().query_sensors()
        for _sensor in _sensors:
            if _sensor.is_depth_sensor():
                sensor = _sensor.as_depth_sensor()
                sensor.set_option(rs.option.enable_auto_exposure, 1.0)
                # sensor.set_option(rs.option.auto_exposure_limit_toggle, 1.0)
                try:
                    time.sleep(1)
                    sensor.set_option(
                        rs.option.auto_exposure_limit,
                        self.arg.rs_depth_sensor_autoexposure_limit
                    )
                except RuntimeError as e:
                    printout("rs error in auto_exposure_limit", 'w')
                except Exception as e:
                    printout(e, 'w')
                else:
                    raise ValueError("rs error in auto_exposure_limit")

                self.enabled_devices[device_sn].depth_sensor = sensor
                if self.verbose:
                    printout(f"{device_sn} depth_sensor is available...", 'i')

        # 5. pipeline stop
        self.stop_device(device_sn)

        printout(f"Initialized RealSense depth sensor AE {device_sn}", 'i')

    def start(self) -> None:
        """Starts the devices. """
        if len(self.enabled_devices) > 0:
            for device_sn, _ in self.enabled_devices.items():
                self.start_device(device_sn)
        else:
            print_no_device_enabled("start()")

    def start_device(self, device_sn: str) -> None:
        self.check_if_device_is_enabled(device_sn)
        _pipeline = self.enabled_devices[device_sn].pipeline
        _pipeline_profile = _pipeline.start(self._rs_cfg[device_sn])
        _num_streams = len(_pipeline_profile.get_streams())
        self.enabled_devices[device_sn].pipeline_profile = _pipeline_profile
        self.enabled_devices[device_sn].num_streams = _num_streams
        if self.verbose:
            printout(f"{device_sn} pipeline started...", 'i')

    def step_clear(self):
        self.frames = {}
        # internal variables
        # output_msgs.clear()
        # self.valid_frame_received_flags.clear()
        # self.empty_frame_received_timers.clear()

    def step(self,
             display: int = 0,
             display_and_save_with_key: bool = False,
             use_colorizer: bool = False,
             save_depth_colormap: bool = False) -> None:
        """Gets the frames streamed from the enabled rs devices.

        Args:
            display (int): Whether to display the retrieved frames.
                The value corresponds to the scale to visualize the frames.
                0 = no display, > 0 = display scale
            display_and_save_with_key (bool): Display data and start saving
                on 'c' key stroke.
            use_colorizer (bool): Whether to use rs colorizer.
                Defaults to False.
            save_depth_colormap (bool): Whether to save the depth colormaps.
        """
        if len(self.enabled_devices) == 0:
            printout(f"No devices are enabled...", 'w')
            return

        self.step_clear()

        while len(self.frames) < len(self.enabled_devices.items()):
            for device_sn, _ in self.enabled_devices.items():
                frames = self.step_device(device_sn,
                                          display_and_save_with_key,
                                          use_colorizer,
                                          save_depth_colormap)

        if display > 0 or display_and_save_with_key:
            self.display_rs_data(self.frames, max(1, display))

    def step_device(self,
                    device_sn: str,
                    display_and_save_with_key: bool = False,
                    use_colorizer: bool = False,
                    save_depth_colormap: bool = False) -> dict:
        """Gets the frames streamed from an enabled rs device.

        Args:
            device_sn (str): device serial number.
            display_and_save_with_key (bool): Display data and start saving
                on 'c' key stroke.
            use_colorizer (bool): Whether to use rs colorizer.
                Defaults to False.
            save_depth_colormap (bool): Whether to save the depth colormaps.
        """
        self.check_if_device_is_enabled(device_sn)

        # 1. keystroke for saving displayed data
        if display_and_save_with_key:
            if self.key == -1:
                self.storage_paths.save = False
            else:
                if self.key & 0xFF == ord('c'):
                    self.storage_paths.save = True
                    self.key = -1

        # 2. Poll for frames.
        frameset, reset_status = self.get_frameset_with_poll(device_sn)
        # resets when too many empty frames.
        if reset_status:
            if device_sn in self.frames:
                self.frames.pop(device_sn)
            self.reset_device_with_empty_frames(device_sn)
            return

        # The 'if' check is needed only for 'poll_for_frames'
        # 3.a. Polled frames are empty.
        if frameset.size() == 0:
            return

        # 3.b. Polled frames are at least in length to the number of streams
        # 4. Make sure both color and depth frames are there.
        if frameset.size() >= self.enabled_devices[device_sn].num_streams:

            self.internal_timestamp[device_sn] = time.time_ns()

            # 5. Check if both frames are valid, skip step if one is invalid.
            if not check_if_color_depth_frames_are_valid(frameset):
                self.enabled_devices[device_sn].color_reset_counter += 1
                self.enabled_devices[device_sn].depth_reset_counter += 1
                printout("One of the streams is missing...", 'w')

            frame_dict = {}
            frame_dict['calib'] = self.calib_data.get_data(device_sn)

            # 6. Tries to align the frames, and skip step if exception occurs.
            try:
                # In the rs_net version, this returns zeros for depth frame
                aligned_frameset = self._align.process(frameset)
                # aligned_frameset = frameset
            except Exception as e:
                printout(f"Align failed...", 'w')
                return

            # TODO add exceptions
            # 6.b. Framesets are aligned.
            # 7. Loops through the streams to get color and depth.
            color_frame_dict, framedata = self.process_color_stream(
                frameset=aligned_frameset,
                device_sn=device_sn,
            )
            frame_dict.update(color_frame_dict)
            depth_frame_dict, framedata = self.process_depth_stream(
                frameset=aligned_frameset,
                device_sn=device_sn,
                use_colorizer=use_colorizer,
                save_colormap=save_depth_colormap
            )
            frame_dict.update(depth_frame_dict)
            print_camera_temperature(
                device_sn, self.temperature_log_interval, self.verbose)

            self.frames[device_sn] = frame_dict

            self.save_timestamp(frame_dict, device_sn)

            # 4. if reset due to time freeze, skip the current step.
            status = self.reset_device_with_frozen_timestamp(
                frame_dict, device_sn)
            if status:
                self.frames[device_sn] = {}

    def get_frameset_with_wait(self, device_sn: str) -> rs.composite_frame:
        dev = self.enabled_devices[device_sn]
        try:
            frameset = dev.pipeline.wait_for_frames(3000)  # ms
        except Exception as e:
            printout(e, 'w')
            printout("resetting device after waiting for 3000ms", 'w')
            self.stop(device_sn=device_sn)
            time.sleep(5)
            self.initialize_device(device_sn=device_sn)
            try:
                frameset = dev.pipeline.wait_for_frames(3000)  # ms
            except Exception as e:
                printout(e, 'w')
                printout("resetting device did not work, skip step", 'w')
                frameset = None
        return frameset

    def get_frameset_with_poll(self, device_sn: str) -> rs.composite_frame:
        reset = False
        frameset = self.enabled_devices[device_sn].pipeline.poll_for_frames()
        if frameset.size() == 0:
            self.poll_counter_per_dev[device_sn][0] += (
                time.time() - self.poll_counter_per_dev[device_sn][1])
        else:
            self.poll_counter_per_dev[device_sn][0] = 0
        if self.poll_counter_per_dev[device_sn][0] > RESET_LIMIT:
            reset = True
        self.poll_counter_per_dev[device_sn][1] = time.time()
        return frameset, reset
        # TODO: Need to recheck this.
        # # to prevent the same frame from being resaved until the RESET_LIMIT
        # if self.poll_counter_per_dev[device_sn][0] > RESET_LIMIT_BUFFER:
        #     return None, reset
        # else:
        #     return frameset, reset

    def reset_device_with_empty_frames(self,
                                       device_sn: str,
                                       ) -> Tuple[rs.composite_frame, bool]:
        printout(f"resetting device after no frames for {RESET_LIMIT}s", 'w')
        self.stop(device_sn=device_sn)
        time.sleep(5)
        self.initialize_device(device_sn=device_sn)
        self.poll_counter_per_dev[device_sn][0] = 0
        self.poll_counter_per_dev[device_sn][1] = time.time()

    def process_color_stream(self,
                             frameset: rs.composite_frame,
                             device_sn: str,
                             ) -> Tuple[dict, np.ndarray]:
        """Get color stream data.

        Args:
            frameset (rs.composite_frame): frameset from pipeline.
            storage_paths (Optional[StoragePaths], optional): If not None,
                data from frameset will be stored. Defaults to None.

        Returns:
            Tuple[dict, np.ndarray]: updated frame_dict and data from framset.
        """
        frame_dict = {}
        frame = frameset.first_or_default(rs.stream.color)
        timestamp = self.query_frame_timestamp(frame)
        frame_dict['color_timestamp'] = timestamp
        framedata = np.asanyarray(frame.get_data())
        frame_dict['color_framedata'] = framedata
        metadata = read_metadata(frame)
        frame_dict['color_metadata'] = metadata
        # No storage
        if not self.storage_paths.save:
            return frame_dict, framedata
        # save frame
        filedir = self.storage_paths.color[device_sn]
        if filedir is not None:
            np.save(os.path.join(filedir, f"{self.internal_timestamp[device_sn]}"),  # noqa
                    frame_dict['color_framedata'])
        # save meta
        filedir = self.storage_paths.color_metadata[device_sn]
        if filedir is not None:
            with open(os.path.join(filedir, f"{self.internal_timestamp[device_sn]}.json"), 'w') as json_f:  # noqa
                json.dump(frame_dict['color_metadata'], json_f, indent=4)
        return frame_dict, framedata

    def process_depth_stream(self,
                             frameset: rs.composite_frame,
                             device_sn: str,
                             use_colorizer: bool = False,
                             save_colormap: bool = False
                             ) -> Tuple[dict, np.ndarray]:
        """Get depth stream data.

        Args:
            frameset (rs.composite_frame): frameset from pipeline.
            storage_paths (Optional[StoragePaths], optional): If not None,
                data from frameset will be stored. Defaults to None.
            use_colorizer (bool): Whether to use rs colorizer.
                Defaults to False.
            save_colormap (bool): Whether to save depth image as colormap.
                Defaults to False.

        Returns:
            Tuple[dict, np.ndarray]: updated frame_dict and data from framset.
        """
        frame_dict = {}
        frame = frameset.first_or_default(rs.stream.depth)
        # frame = post_process_depth_frame(frame)
        timestamp = self.query_frame_timestamp(frame)
        frame_dict['depth_timestamp'] = timestamp
        framedata = np.asanyarray(frame.get_data())
        frame_dict['depth_framedata'] = framedata
        metadata = read_metadata(frame)
        frame_dict['depth_metadata'] = metadata
        if use_colorizer:
            color_framedata = np.asanyarray(
                self._colorizer.colorize(frame).get_data())
            frame_dict['depth_color_framedata'] = color_framedata
        # No storage
        if not self.storage_paths.save:
            return frame_dict, framedata
        # save frame
        filedir = self.storage_paths.depth[device_sn]
        if filedir is not None:
            # arr1 = np.uint8(frame_dict['depth_framedata'] >> 8)
            # arr2 = np.uint8(frame_dict['depth_framedata'])
            # np.save(os.path.join(filedir, f'{ts}_arr1'), arr1)
            # np.save(os.path.join(filedir, f'{ts}_arr2'), arr2)
            np.save(os.path.join(filedir, f'{self.internal_timestamp[device_sn]}'),  # noqa
                    frame_dict['depth_framedata'])
        # save depth colormap
        if save_colormap:
            if not use_colorizer:
                # Apply colormap on depth image
                # (image must be converted to 8-bit per pixel first)
                color_framedata = cv2.applyColorMap(
                    cv2.convertScaleAbs(
                        frame_dict['depth_framedata'], alpha=0.03),
                    cv2.COLORMAP_JET
                )
            image_name = os.path.join(filedir, f'{self.internal_timestamp[device_sn]}.jpg')  # noqa
            cv2.imwrite(image_name, color_framedata)
        # save meta
        filedir = self.storage_paths.depth_metadata[device_sn]
        if filedir is not None:
            with open(os.path.join(filedir, f"{self.internal_timestamp[device_sn]}.json"), 'w') as json_f:  # noqa
                json.dump(frame_dict['depth_metadata'], json_f, indent=4)
        return frame_dict, framedata

    def display_rs_data(self, frames: dict, scale: int = 1) -> bool:
        """Displays both color and depth data from frame.

        Args:
            frames (dict): Dictionary containing frame data.
            scale (int): Scale of the displayed image. Defaults to 1.

        Returns:
            bool: If True, the 'q' button is pressed.
        """
        q_press = False
        for device_sn, data_dict in frames.items():
            # Render images
            if data_dict.get('depth_color_framedata', None) is not None:
                depth_colormap = data_dict['depth_color_framedata']
            else:
                depth_colormap = cv2.applyColorMap(
                    cv2.convertScaleAbs(data_dict['depth_framedata'],
                                        alpha=0.03),
                    cv2.COLORMAP_JET)
            # # Set pixels further than clipping_distance to grey
            # clipping_distance = 10
            # grey_color = 153
            # # depth image is 1 channel, color is 3 channels
            # depth_image_3d = np.dstack(
            #     (depth_image, depth_image, depth_image))
            # bg_removed = np.where(
            #     (depth_image_3d > clipping_distance) | (
            #         depth_image_3d <= 0), grey_color, color_image)
            # images = np.hstack((bg_removed, depth_colormap))
            # images = np.hstack((color_image, depth_colormap))
            images_overlapped = cv2.addWeighted(
                data_dict['color_framedata'], 0.3, depth_colormap, 0.5, 0)
            images = np.hstack((data_dict['color_framedata'],
                                depth_colormap,
                                images_overlapped))
            images = cv2.resize(images, (images.shape[1]//scale,
                                         images.shape[0]//scale))
            cv2.namedWindow(f'{device_sn}', cv2.WINDOW_AUTOSIZE)
            cv2.imshow(f'{device_sn}', images)
            self.key = cv2.waitKey(30)
            # Press esc or 'q' to close the image window
            if self.key & 0xFF == ord('q') or self.key == 27:
                cv2.destroyAllWindows()
                cv2.waitKey(5)
                q_press = True

        return q_press

    def stop(self) -> None:
        """Stops the devices. """
        if len(self.enabled_devices) > 0:
            for device_sn, _ in self.enabled_devices.items():
                self.stop_device(device_sn)
        else:
            print_no_device_enabled("stop()")

    def stop_device(self, device_sn: str) -> None:
        self.check_if_device_is_enabled(device_sn)
        self.enabled_devices[device_sn].pipeline.stop()

    def reset_device_with_frozen_timestamp(self,
                                           frame_dict: dict,
                                           device_sn: str) -> bool:
        """Resets the device if the timestamp is frozen, i.e., device keeps
        returning the same frame.

        Args:
            frame_dict (dict): Dictionary containing framedata.
            device_sn (str): rs camera serial number.

        Returns:
            bool: Whether the device has been resetted.
        """
        ds = device_sn
        reset = False

        for ts in ['color_timestamp', 'depth_timestamp']:
            if frame_dict[ts] > self.last_timestamps[ds][ts]['timestamp']:
                self.last_timestamps[ds][ts]['timestamp'] = frame_dict[ts]
                self.last_timestamps[ds][ts]['count'] = 0
            elif frame_dict[ts] == self.last_timestamps[ds][ts]['timestamp']:
                if self.last_timestamps[ds][ts]['count'] > RESET_LIMIT:
                    printout(f"resetting device after 'color_timestamp' frozen "
                             f"for {RESET_LIMIT}s", 'w')
                    reset = True
                else:
                    self.last_timestamps[ds][ts]['count'] += 1
            else:
                printout(f"Current color_timestamp is smaller than "
                         f"previous value : {frame_dict[ts]} < "
                         f"{self.last_timestamps[ds][ts]['timestamp']}", 'w')
                reset = True

        if reset:
            printout("Resetting device with frozen timestamp...", 'w')
            self.stop(device_sn=device_sn)
            time.sleep(5)
            self.initialize_device(device_sn=device_sn)
            for ts in ['color_timestamp', 'depth_timestamp']:
                self.last_timestamps[device_sn][ts]['timestamp'] = 0
                self.last_timestamps[device_sn][ts]['count'] = 0
        return reset
