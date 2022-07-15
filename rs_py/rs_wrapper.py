# Network part is based on :
# https://github.com/IntelRealSense/librealsense/blob/master/wrappers/python/examples/net_viewer.py

import argparse
import cv2
import json
import numpy as np
import os
import time

from datetime import datetime
from functools import partial
from typing import Optional, Tuple, Union, Any

from rs_py.realsense_device_manager import enumerate_connected_devices
from rs_py.realsense_device_manager import post_process_depth_frame
from rs_py.utils import str2bool
from rs_py.utils import printout

from rs_py import rs
from rs_py import rsnet


def read_metadata(frame: rs.frame) -> dict:
    _FRAME_METADATA_VALUE_LIST = [
        rs.frame_metadata_value.actual_exposure,
        rs.frame_metadata_value.actual_fps,
        rs.frame_metadata_value.auto_exposure,
        rs.frame_metadata_value.auto_white_balance_temperature,
        rs.frame_metadata_value.backend_timestamp,
        rs.frame_metadata_value.backlight_compensation,
        rs.frame_metadata_value.brightness,
        rs.frame_metadata_value.contrast,
        rs.frame_metadata_value.exposure_priority,
        rs.frame_metadata_value.exposure_roi_bottom,
        rs.frame_metadata_value.exposure_roi_left,
        rs.frame_metadata_value.exposure_roi_right,
        rs.frame_metadata_value.exposure_roi_top,
        rs.frame_metadata_value.frame_counter,
        rs.frame_metadata_value.frame_emitter_mode,
        rs.frame_metadata_value.frame_laser_power,
        rs.frame_metadata_value.frame_laser_power_mode,
        rs.frame_metadata_value.frame_led_power,
        rs.frame_metadata_value.frame_timestamp,
        rs.frame_metadata_value.gain_level,
        rs.frame_metadata_value.gamma,
        rs.frame_metadata_value.hue,
        rs.frame_metadata_value.low_light_compensation,
        rs.frame_metadata_value.manual_white_balance,
        rs.frame_metadata_value.power_line_frequency,
        rs.frame_metadata_value.raw_frame_size,
        rs.frame_metadata_value.saturation,
        rs.frame_metadata_value.sensor_timestamp,
        rs.frame_metadata_value.sharpness,
        rs.frame_metadata_value.temperature,
        rs.frame_metadata_value.time_of_arrival,
        rs.frame_metadata_value.white_balance,
    ]
    output = {}
    for i in _FRAME_METADATA_VALUE_LIST:
        if frame.supports_frame_metadata(i):
            output[i.name] = frame.get_frame_metadata(i)
    return output


def str2rsformat(v) -> rs.format:
    _SUPPORTED_FORMATS = {
        'z16': rs.format.z16,
        'bgr8': rs.format.bgr8,
        'rgb8': rs.format.rgb8,
        'yuyv': rs.format.yuyv
    }
    if v.lower() not in _SUPPORTED_FORMATS.keys():
        raise argparse.ArgumentTypeError('unknown stream format')
    else:
        return _SUPPORTED_FORMATS.get(v.lower())


def get_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description='Run RealSense devices.')
    parser.add_argument('--rs-steps',
                        type=int,
                        default=10,
                        help='number of steps to run (in secs)')
    parser.add_argument('--rs-fps',
                        type=int,
                        default=6,
                        help='fps')
    parser.add_argument('--rs-image-width',
                        type=int,
                        default=848,
                        help='image width in px')
    parser.add_argument('--rs-image-height',
                        type=int,
                        default=480,
                        help='image height in px')
    parser.add_argument('--rs-color-format',
                        type=str2rsformat,
                        default=rs.format.bgr8,
                        help='format of color stream')
    parser.add_argument('--rs-depth-format',
                        type=str2rsformat,
                        default=rs.format.z16,
                        help='format of depth stream')
    parser.add_argument('--rs-laser-power',
                        type=int,
                        default=290,
                        help='laser power')
    parser.add_argument('--rs-display-frame',
                        type=int,
                        default=0,
                        help='scale for displaying realsense raw images.')
    parser.add_argument('--rs-save-with-key',
                        type=str2bool,
                        default=False,
                        help='save using key = "c" ')
    parser.add_argument('--rs-save-data',
                        type=str2bool,
                        default=True,
                        help='Whether to save data')
    parser.add_argument('--rs-save-path',
                        type=str,
                        default='/data/realsense',
                        help='path to save realsense frames if --rs-save-data=True.')  # noqa
    parser.add_argument('--rs-save-color',
                        type=str2bool,
                        default=True,
                        help='Whether to save color frames')
    parser.add_argument('--rs-save-depth',
                        type=str2bool,
                        default=True,
                        help='Whether to save depth frames')
    parser.add_argument('--rs-save-stacked',
                        type=str2bool,
                        default=False,
                        help='Whether to save color and depth frames in a stack')  # noqa
    parser.add_argument('--rs-use-one-dev-only',
                        type=str2bool,
                        default=False,
                        help='use 1 rs device only.')
    parser.add_argument('--rs-dev',
                        type=str,
                        help='rs device sn to run')
    parser.add_argument('--rs-ip',
                        # nargs='*',
                        type=str,
                        # default='192.168.100.39',  # 101 LAN
                        # default='192.168.1.216',  # 101 WLAN
                        # default='192.168.1.11',  # 102 WLAN
                        help='ip address')
    parser.add_argument('--rs-test-init-runtime',
                        type=str2bool,
                        default=False,
                        help='test the runtime of device initialization.')
    return parser


class CalibrationConfig:
    def __init__(self):
        self.device_sn = []
        self.color = []
        self.depth = []
        self.T_color_depth = []

    def save(self,
             file_path: str,
             device_sn_idx: Optional[Union[int, str]] = None) -> None:
        self.validate()
        if device_sn_idx is None:
            with open(file_path, 'w') as outfile:
                json.dump(self.__dict__, outfile, indent=4)
        else:
            with open(file_path, 'w') as outfile:
                json.dump(self.get_data(device_sn_idx), outfile, indent=4)

    def load(self, file_path: str) -> None:
        with open(file_path) as calib_file:
            calib_data = json.load(calib_file)
        self.__dict__.update(calib_data)
        self.validate()

    def get_T_color_depth_np(self,
                             idx: Optional[int] = None,
                             T_color_depth: Optional[dict] = None
                             ) -> np.ndarray:
        T_color_depth = np.eye(4)
        if idx is not None:
            T_color_depth[:3, :3] = np.array(self.T_color_depth[idx]['rotation']).reshape(3, 3)  # noqa
            T_color_depth[:3, 3] = self.T_color_depth[idx]['translation']
        elif T_color_depth is not None:
            T_color_depth[:3, :3] = np.array(T_color_depth['rotation']).reshape(3, 3)  # noqa
            T_color_depth[:3, 3] = T_color_depth['translation']
        return T_color_depth

    def get_data(self, device_sn_idx: Union[int, str]):
        if isinstance(device_sn_idx, int):
            idx = device_sn_idx
        elif isinstance(device_sn_idx, str):
            idx = self.device_sn.index(device_sn_idx)
        else:
            raise ValueError("Unknown input argument type...")
        return {
            'color': self.color[idx],
            'depth': self.depth[idx],
            # 'T_color_depth': self.get_T_color_depth(idx),
            'T_color_depth': self.T_color_depth[idx],
        }

    def validate(self) -> bool:
        assert len(self.device_sn) == len(self.color)
        assert len(self.device_sn) == len(self.depth)
        assert len(self.device_sn) == len(self.T_color_depth)


class StoragePaths:
    def __init__(self, device_sn: str = '', base_path: str = '/data/realsense'):

        # flag that can be used to see if data should be saved
        self.save = True

        date_time = datetime.now().strftime("%y%m%d%H%M%S")
        device_path = os.path.join(base_path, f'dev_{device_sn}', date_time)

        # self.color = None
        # self.depth = None
        self.color = os.path.join(device_path, 'color')
        self.depth = os.path.join(device_path, 'depth')
        os.makedirs(self.color, exist_ok=True)
        os.makedirs(self.depth, exist_ok=True)

        # self.color_metadata = None
        # self.depth_metadata = None
        self.color_metadata = os.path.join(device_path, 'color_metadata')
        self.depth_metadata = os.path.join(device_path, 'depth_metadata')
        os.makedirs(self.color_metadata, exist_ok=True)
        os.makedirs(self.depth_metadata, exist_ok=True)

        self.calib = os.path.join(device_path, 'calib')
        os.makedirs(self.calib, exist_ok=True)

        self.timestamp = os.path.join(device_path, 'timestamp')
        self.timestamp_file = os.path.join(self.timestamp, 'timestamp.txt')
        os.makedirs(self.timestamp, exist_ok=True)

        printout("Prepared storage paths...", 'i')


class StreamConfig:
    def __init__(self,
                 stream_type: rs.stream,
                 width: int,
                 height: int,
                 format: rs.format,
                 framerate: int):
        self.stream_type = stream_type  # rs.stream.depth/color
        self.width = width  # 848
        self.height = height  # 480
        self.format = format  # rs.format.z16/bgr8
        self.framerate = framerate  # 30

    @property
    def data(self) -> dict:
        return self.__dict__


# Based on realsense_device_manager.py
class Device:
    def __init__(self, pipeline, pipeline_profile, color_sensor, depth_sensor):
        self.pipeline = pipeline
        self.pipeline_profile = pipeline_profile
        self.color_sensor = color_sensor
        self.depth_sensor = depth_sensor


class RealsenseWrapper:
    """Wrapper to run multiple realsense cameras.

    Code is written based on "realsense_device_manager.py" . Currently the
    code supports only reading depth and color images. Reading of IR stream
    is not implemented.

    """

    def __init__(self,
                 arg: argparse.Namespace,
                 dev_sn: Optional[str] = None,
                 ctx: Optional[rs.context] = None) -> None:
        """Initializes the RealsenseWrapper class object.

        Args:
            arg (argparse.Namespace): argument parsed from cli.
            dev_sn (Optional[str], optional): If not None, only the device
                with this sn will be used. Defaults to None.
            ctx
        """
        # device data
        if arg.rs_ip is not None:
            # No multi ethernet support.
            # https://github.com/IntelRealSense/librealsense/issues/6376
            printout("Network mode", 'i')
            self.network = True
            self.available_devices = []
            self.ctx = ctx if ctx is not None else rs.context()
            dev = rsnet.net_device(arg.rs_ip)
            self.available_devices.append(
                (dev.get_info(rs.camera_info.serial_number), None))
            dev.add_to(self.ctx)
            printout(f"Connected to {arg.rs_ip}", 'i')
        else:
            printout("Local mode", 'i')
            self.network = False
            self.ctx = ctx if ctx is not None else rs.context()
            if dev_sn is None:
                self.available_devices = enumerate_connected_devices(self.ctx)
            else:
                self.available_devices = [(dev_sn, 'D400')]

        # available devices
        if dev_sn is not None:
            self.available_devices = [
                i for i in self.available_devices if i[0] == dev_sn]
        elif arg.rs_use_one_dev_only:
            self.available_devices = self.available_devices[0:1]

        self.available_devices = sorted(self.available_devices)

        # serial numbers of enabled devices
        self.enabled_devices = {}
        self.calib_data = {}

        # rs align method
        self._align = rs.align(rs.stream.color)  # align depth to color frame

        # configurations
        self._rs_cfg = {}
        self.stream_config_color = StreamConfig(
            rs.stream.color, arg.rs_image_width, arg.rs_image_height,
            arg.rs_color_format, arg.rs_fps
        )
        self.stream_config_depth = StreamConfig(
            rs.stream.depth, arg.rs_image_width, arg.rs_image_height,
            arg.rs_depth_format, arg.rs_fps
        )

        # Save paths
        self.timestamp_mode = None
        self.save_stacked = arg.rs_save_stacked
        self.storage_paths_per_dev = {}
        if arg.rs_save_path is not None:
            storage_paths_fn = partial(
                StoragePaths, base_path=arg.rs_save_path)
        else:
            storage_paths_fn = StoragePaths
        for sn, _ in self.available_devices:
            _storage_path = storage_paths_fn(sn)
            if arg.rs_save_data:
                _storage_path.save = True
            else:
                _storage_path.save = False
            if not arg.rs_save_color:
                _storage_path.color = None
                _storage_path.color_metadata = None
            if not arg.rs_save_depth:
                _storage_path.depth = None
                _storage_path.depth_metadata = None
            self.storage_paths_per_dev[sn] = _storage_path

        # internal variables
        self._key = -1
        self._timestamp = 0
        self._temperature_log_interval = 300
        self._temperature_log_counter = {sn: 0
                                         for sn, _ in self.available_devices}
        self._timestamp_per_dev = {
            sn: {'color_timestamp': 0, 'depth_timestamp': 0}
            for sn, _ in self.available_devices
        }

    @property
    def timestamp(self):
        return self._timestamp

    @timestamp.setter
    def timestamp(self, x):
        self._timestamp = x

# [MAIN FUNCTIONS] *************************************************************

    def initialize_device(self,
                          device_sn: str,
                          enable_ir_emitter: bool = True,
                          verbose: bool = True) -> None:
        """Initializes a single device pipeline and starts it.

        Args:
            dev_sn (str): Device to start.
            enable_ir_emitter (bool, optional): Enable the IR for beter
                depth quality. Defaults to True.
            verbose (bool): Whether to printout infos. Defaults to True.
        """
        printout(f"Initializing RealSense devices {device_sn}", 'i')
        self.configure_stream(device_sn,
                              self.stream_config_depth,
                              self.stream_config_color)
        # Pipeline
        if self.network:
            pipeline = rs.pipeline(self.ctx)
        else:
            pipeline = rs.pipeline()
        # Cfg
        cfg = self._rs_cfg[device_sn]
        if not self.network:
            cfg.enable_device(device_sn)
        check = cfg.can_resolve(pipeline)
        printout(f"'cfg' usable with 'pipeline' : {check}", 'i')
        # Pipeline
        pipeline_profile = pipeline.start(cfg)
        color_sensor = pipeline_profile.get_device().first_color_sensor()
        depth_sensor = pipeline_profile.get_device().first_depth_sensor()
        # IR for depth
        if enable_ir_emitter:
            if depth_sensor.supports(rs.option.emitter_enabled):
                depth_sensor.set_option(rs.option.emitter_enabled,
                                        1 if enable_ir_emitter else 0)
                # depth_sensor.set_option(rs.option.laser_power, 330)
        # Stored the enabled devices
        self.enabled_devices[device_sn] = \
            Device(pipeline, pipeline_profile, color_sensor, depth_sensor)
        # Check which timestamp is available.
        if len(self.storage_paths_per_dev) > 0:
            self._query_timestamp_mode(device_sn)
        # Camera info
        if verbose:
            self._print_camera_info(pipeline_profile)
        printout(f"Initialized RealSense devices {device_sn}", 'i')

    def initialize(self,
                   enable_ir_emitter: bool = True,
                   verbose: bool = True) -> None:
        """Initializes the device pipelines and starts them.

        Args:
            enable_ir_emitter (bool): Enable the IR for beter
                depth quality. Defaults to True.
            verbose (bool): Whether to printout infos. Defaults to True.
        """
        for device_sn, product_line in self.available_devices:
            self.initialize_device(device_sn, enable_ir_emitter, verbose)

    def step(self,
             display: int = 0,
             display_and_save_with_key: bool = False,
             save_depth_colormap: bool = False) -> dict:
        """Gets the frames streamed from the enabled rs devices.

        Args:
            display (int, optional): Whether to display the retrieved frames.
                The value corresponds to the scale to visualize the frames.
                0 = no display, 1 = display scale

        Returns:
            dict: Empty dict or {serial_number: {data_type: data}}.
                data_type = color, depth, timestamp, calib
        """
        if len(self.enabled_devices) == 0:
            printout(f"No devices are enabled...", 'w')
            return {}

        frames = {}
        while len(frames) < len(self.enabled_devices.items()):

            for dev_sn, dev in self.enabled_devices.items():

                storage_paths = self._get_storage_paths(
                    dev_sn, display_and_save_with_key)

                streams = dev.pipeline_profile.get_streams()
                frameset = dev.pipeline.poll_for_frames()

                if frameset.size() == len(streams):

                    self.timestamp = time.time()

                    frame_dict = {}
                    frame_dict['calib'] = self.calib_data.get(dev_sn, {})

                    # In the rs_net version, this returns zeros for depth frame
                    aligned_frameset = self._align.process(frameset)
                    # aligned_frameset = frameset

                    for stream in streams:
                        st = stream.stream_type()
                        if st == rs.stream.color:
                            frame_dict, framedata = self._get_color_stream(
                                frameset=aligned_frameset,
                                frame_dict=frame_dict,
                                storage_paths=None if self.save_stacked else storage_paths,  # noqa
                            )
                        elif st == rs.stream.depth:
                            frame_dict, framedata = self._get_depth_stream(
                                frameset=aligned_frameset,
                                frame_dict=frame_dict,
                                storage_paths=None if self.save_stacked else storage_paths,  # noqa
                                save_colormap=save_depth_colormap
                            )
                            self._print_camera_temperature(dev_sn)
                        else:
                            raise ValueError(f"Unsupported stream : {st}")

                    frames[dev_sn] = frame_dict

                    self._save_timestamp(frame_dict, storage_paths)

                    self._reset_device_with_frozen_timestamp(
                        frame_dict, dev_sn)

        # Save data as a stacked array
        if self.save_stacked:
            color_framedata_list = []
            color_timestamp_list = []
            color_metadata_dict = {}
            depth_framedata_list = []
            depth_timestamp_list = []
            depth_metadata_dict = {}
            for dev_sn, frame_dict in frames.items():
                color_framedata_list.append(frame_dict['color_framedata'])
                color_timestamp_list.append(str(frame_dict['color_timestamp']))
                color_metadata_dict[dev_sn] = frame_dict['color_metadata']
                depth_framedata_list.append(frame_dict['depth_framedata'])
                depth_timestamp_list.append(str(frame_dict['depth_timestamp']))
                depth_metadata_dict[dev_sn] = frame_dict['depth_metadata']
            _frame_dict = {}
            _frame_dict['color_framedata'] = np.hstack(color_framedata_list)
            _frame_dict['color_timestamp'] = "_".join(color_timestamp_list)
            _frame_dict['color_metadata'] = color_metadata_dict
            _frame_dict['depth_framedata'] = np.hstack(depth_framedata_list)
            _frame_dict['depth_timestamp'] = "_".join(depth_timestamp_list)
            _frame_dict['depth_metadata'] = depth_metadata_dict
            storage = self.storage_paths_per_dev[self.available_devices[0][0]]
            self._save_color_framedata(_frame_dict, storage)
            self._save_depth_framedata(_frame_dict, storage,
                                       save_depth_colormap)

        if display > 0:
            if self._display_rs_data(frames, display):
                return {}

        return frames

    def stop(self, device_sn: Optional[str] = None) -> None:
        """Stops the devices.

        Args:
            device_sn (Optional[str]): If given, stops only that device.
        """
        if len(self.enabled_devices) == 0:
            printout(f"No devices are enabled...", 'w')
        else:
            if device_sn is not None:
                self.enabled_devices[device_sn].pipeline.stop()
            else:
                for _, dev in self.enabled_devices.items():
                    dev.pipeline.stop()

# [UTIL FUNCTIONS] *************************************************************

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
            self._rs_cfg[device_sn] = cfg

    def set_ir_laser_power(self, power: int = 300):
        """Sets the power of the IR laser. If power value is too high the
        rs connection will crash.

        https://github.com/IntelRealSense/librealsense/issues/1258

        Args:
            power (int, optional): IR power. Defaults to 300.
        """
        for _, dev in self.enabled_devices.items():
            sensor = dev.pipeline_profile.get_device().first_depth_sensor()
            if sensor.supports(rs.option.emitter_enabled):
                ir_range = sensor.get_option_range(rs.option.laser_power)
                if power + 10 > ir_range.max:
                    sensor.set_option(rs.option.laser_power, ir_range.max)
                else:
                    sensor.set_option(rs.option.laser_power, power + 10)

    def dummy_capture(self, num_frames: int = 30) -> None:
        """Dummy capture 'num_frames' frames to give
        autoexposure, etc. a chance to settle.

        Args:
            num_frames (int): Number of dummy frames to skip. Defaults to 30.
        """
        printout(f"Capturing dummy frames...", 'i')
        frames = {}
        for _ in range(num_frames):
            while len(frames) < len(self.enabled_devices.items()):
                for dev_sn, dev in self.enabled_devices.items():
                    streams = dev.pipeline_profile.get_streams()
                    frameset = dev.pipeline.poll_for_frames()
                    if frameset.size() == len(streams):
                        frames[dev_sn] = {}
        printout(f"Finished capturing dummy frames...", 'i')

    def save_calibration(self) -> None:
        """Saves camera calibration. """

        if len(self.enabled_devices) == 0:
            printout(f"No devices are enabled...", 'w')
            return

        calib_config = CalibrationConfig()
        for dev_sn, dev in self.enabled_devices.items():

            storage_paths = self._get_storage_paths(dev_sn, False)
            if storage_paths is None:
                continue

            assert os.path.exists(storage_paths.calib)
            if os.path.isfile(storage_paths.calib):
                save_path = storage_paths.calib
            else:
                filename = f'dev{dev_sn}_calib.txt'
                save_path = os.path.join(storage_paths.calib, filename)

            calib_config.device_sn.append(dev_sn)

            profile = dev.pipeline_profile

            # Intrinsics of color & depth frames -------------------------------
            profile_color = profile.get_stream(rs.stream.color)
            intr_color = profile_color.as_video_stream_profile()
            intr_color = intr_color.get_intrinsics()

            # Fetch stream profile for depth stream
            # Downcast to video_stream_profile and fetch intrinsics
            profile_depth = profile.get_stream(rs.stream.depth)
            intr_depth = profile_depth.as_video_stream_profile()
            intr_depth = intr_depth.get_intrinsics()

            # Extrinsic matrix from color sensor to Depth sensor ---------------
            profile_vid = profile_color.as_video_stream_profile()
            extr = profile_vid.get_extrinsics_to(profile_depth)
            extr_mat = np.eye(4)
            extr_mat[:3, :3] = np.array(extr.rotation).reshape(3, 3)
            extr_mat[:3, 3] = extr.translation

            # Depth scale ------------------------------------------------------
            # for sensor in profile.get_device().query_sensors():
            #     if sensor.is_depth_sensor():
            depth_sensor = profile.get_device().first_depth_sensor()
            depth_sensor = rs.depth_stereo_sensor(depth_sensor)
            depth_scale = depth_sensor.get_depth_scale()
            depth_baseline = depth_sensor.get_stereo_baseline()

            # Write calibration data to json file ------------------------------
            calib_config.color.append({
                'width': intr_color.width,
                'height': intr_color.height,
                'intrinsic_mat': [intr_color.fx, 0, intr_color.ppx,
                                  0, intr_color.fy, intr_color.ppy,
                                  0, 0, 1],
                'model': str(intr_color.model),
                'coeffs': intr_color.coeffs
            })
            calib_config.depth.append({
                'width': intr_depth.width,
                'height': intr_depth.height,
                'intrinsic_mat': [intr_depth.fx, 0, intr_depth.ppx,
                                  0, intr_depth.fy, intr_depth.ppy,
                                  0, 0, 1],
                'model': str(intr_depth.model),
                'coeffs': intr_depth.coeffs,
                'depth_scale': depth_scale,
                'depth_baseline': depth_baseline
            })
            calib_config.T_color_depth.append({
                'rotation': extr.rotation,
                'translation': extr.translation
            })

            calib_config.save(save_path, dev_sn)

        printout(f"Saved camera calibration data...", 'i')

# [PRIVATE FUNCTIONS] **********************************************************

    def _get_storage_paths(self,
                           device_sn: str,
                           display_and_save_with_key: bool):
        storage_paths = self.storage_paths_per_dev.get(device_sn, None)
        if storage_paths is not None:
            if display_and_save_with_key:
                if self._key == -1:
                    storage_paths.save = False
                else:
                    if self._key & 0xFF == ord('c'):
                        storage_paths.save = True
                        self._key = -1
        return storage_paths

    def _query_timestamp_mode(self, device_sn: str):
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
            self.storage_paths_per_dev.pop(device_sn)
            printout("Both sensor_timestamp/frame_timestamp "
                     "are not available. No data will be saved...", 'w')

    def _get_timestamp(self, frame: rs.frame) -> int:
        if self.timestamp_mode is None:
            return -1
        else:
            return frame.get_frame_metadata(self.timestamp_mode)

    def _get_color_stream(self,
                          frameset: rs.composite_frame,
                          frame_dict: dict,
                          storage_paths: Optional[StoragePaths] = None,
                          ) -> Tuple[dict, np.ndarray]:
        """Get color stream data.

        Args:
            frameset (rs.composite_frame): frameset from pipeline.
            frame_dict (dict): dict to save the data from frameset.
            storage_paths (Optional[StoragePaths], optional): If not None,
                data from frameset will be stored. Defaults to None.

        Returns:
            Tuple[dict, np.ndarray]: updated frame_dict and data from framset.
        """
        frame = frameset.first_or_default(rs.stream.color)
        timestamp = self._get_timestamp(frame)
        frame_dict['color_timestamp'] = timestamp
        framedata = np.asanyarray(frame.get_data())
        frame_dict['color_framedata'] = framedata
        metadata = read_metadata(frame)
        frame_dict['color_metadata'] = metadata
        self._save_color_framedata(frame_dict, storage_paths)
        return frame_dict, framedata

    def _get_depth_stream(self,
                          frameset: rs.composite_frame,
                          frame_dict: dict,
                          storage_paths: Optional[StoragePaths] = None,
                          save_colormap: bool = False
                          ) -> Tuple[dict, np.ndarray]:
        """Get depth stream data.

        Args:
            frameset (rs.composite_frame): frameset from pipeline.
            frame_dict (dict): dict to save the data from frameset.
            storage_paths (Optional[StoragePaths], optional): If not None,
                data from frameset will be stored. Defaults to None.
            save_colormap (bool): Whether to save depth image as colormap.
                Defaults to False.

        Returns:
            Tuple[dict, np.ndarray]: updated frame_dict and data from framset.
        """
        frame = frameset.first_or_default(rs.stream.depth)
        # frame = post_process_depth_frame(frame)
        timestamp = self._get_timestamp(frame)
        frame_dict['depth_timestamp'] = timestamp
        framedata = np.asanyarray(frame.get_data())
        frame_dict['depth_framedata'] = framedata
        metadata = read_metadata(frame)
        frame_dict['depth_metadata'] = metadata
        self._save_depth_framedata(frame_dict, storage_paths, save_colormap)
        return frame_dict, framedata

    def _save_timestamp(self, frame_dict: dict, storage_paths: StoragePaths):
        if storage_paths is not None:
            if storage_paths.save:
                ts_file = storage_paths.timestamp_file
                if ts_file is not None:
                    with open(ts_file, 'a+') as f:
                        f.write(f"{self.timestamp}::{frame_dict['color_timestamp']}::{frame_dict['depth_timestamp']}\n")  # noqa

    def _reset_device_with_frozen_timestamp(self,
                                            frame_dict: dict,
                                            device_sn: str):
        ct = 'color_timestamp'
        dt = 'depth_timestamp'
        reset = False
        if frame_dict.get(ct, None) is not None:
            if frame_dict[ct] > self._timestamp_per_dev[device_sn][ct]:
                self._timestamp_per_dev[device_sn][ct] = frame_dict[ct]
            elif frame_dict[ct] == self._timestamp_per_dev[device_sn][ct]:
                reset = True
            else:
                raise ValueError(f"Current color_timestamp is smaller than "
                                 f"previous value : {frame_dict[ct]} < "
                                 f"{self._timestamp_per_dev[device_sn][ct]}")
        if frame_dict.get(dt, None) is not None:
            if frame_dict[dt] > self._timestamp_per_dev[device_sn][dt]:
                self._timestamp_per_dev[device_sn][dt] = frame_dict[dt]
            elif frame_dict[dt] == self._timestamp_per_dev[device_sn][dt]:
                reset = True
            else:
                raise ValueError(f"Current depth_timestamp is smaller than "
                                 f"previous value : {frame_dict[dt]} < "
                                 f"{self._timestamp_per_dev[device_sn][dt]}")
        if reset:
            self.stop(device_sn=device_sn)
            self.initialize_device(device_sn=device_sn)
            self._timestamp_per_dev[device_sn][ct] = 0
            self._timestamp_per_dev[device_sn][dt] = 0

    def _save_color_framedata(self,
                              frame_dict: dict,
                              storage_paths: StoragePaths):
        ts = frame_dict['color_timestamp']
        # No storage
        if storage_paths is None:
            return
        # If save flag is true
        if storage_paths.save:
            # save frame
            filedir = storage_paths.color
            if filedir is not None:
                np.save(os.path.join(filedir, f"{ts}"),
                        frame_dict['color_framedata'])
            # save meta
            filedir = storage_paths.color_metadata
            if filedir is not None:
                with open(os.path.join(filedir, f"{ts}.json"), 'w') as json_f:
                    json.dump(frame_dict['color_metadata'], json_f, indent=4)

    def _save_depth_framedata(self,
                              frame_dict: dict,
                              storage_paths: StoragePaths,
                              save_colormap: bool = False):
        ts = frame_dict['depth_timestamp']
        # No storage
        if storage_paths is None:
            return
        # If save flag is true
        if storage_paths.save:
            # save frame
            filedir = storage_paths.depth
            if filedir is not None:
                # arr1 = np.uint8(frame_dict['depth_framedata'] >> 8)
                # arr2 = np.uint8(frame_dict['depth_framedata'])
                # np.save(os.path.join(filedir, f'{ts}_arr1'), arr1)
                # np.save(os.path.join(filedir, f'{ts}_arr2'), arr2)
                np.save(os.path.join(filedir, f'{ts}'),
                        frame_dict['depth_framedata'])
            # save depth colormap
            if save_colormap:
                # Apply colormap on depth image
                # (image must be converted to 8-bit per pixel first)
                image = cv2.applyColorMap(
                    cv2.convertScaleAbs(
                        frame_dict['depth_framedata'], alpha=0.03),
                    cv2.COLORMAP_JET
                )
                image_name = os.path.join(filedir, f'{ts}.jpg')
                cv2.imwrite(image_name, image)
            # save meta
            filedir = storage_paths.depth_metadata
            if filedir is not None:
                with open(os.path.join(filedir, f"{ts}.json"), 'w') as json_f:
                    json.dump(frame_dict['depth_metadata'], json_f, indent=4)

    def _display_rs_data(self, frames: dict, scale: int) -> bool:
        terminate = False
        for dev_sn, data_dict in frames.items():
            # Render images
            depth_colormap = cv2.applyColorMap(
                cv2.convertScaleAbs(data_dict['depth_framedata'], alpha=0.03),
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
            cv2.namedWindow(f'{dev_sn}', cv2.WINDOW_AUTOSIZE)
            cv2.imshow(f'{dev_sn}', images)
            self._key = cv2.waitKey(30)
            # Press esc or 'q' to close the image window
            if self._key & 0xFF == ord('q') or self._key == 27:
                cv2.destroyAllWindows()
                cv2.waitKey(5)
                terminate = True

        return terminate

    def _print_camera_info(self, profile: rs.pipeline_profile):
        """Prints out info about the camera

        Args:
            profile (rs.pipeline_profile): pipeline_profile of a rs device.
        """
        _info = profile.get_device().get_info
        print("========================================")
        print(">>>>> camera info <<<<<")
        print("========================================")
        print(f'Name          : {_info(rs.camera_info.name)}')
        print(f'Serial Number : {_info(rs.camera_info.serial_number)}')
        try:
            print(f'Product Line  : {_info(rs.camera_info.product_line)}')
        except Exception as e:
            print(f'Product Line  : not available', e)
        try:
            print(f'Firmware      : {_info(rs.camera_info.firmware_version)}')
        except Exception as e:
            print(f'Firmware      : not available', e)
        try:
            print(f'USB type      : {_info(rs.camera_info.usb_type_descriptor)}')  # noqa
        except Exception as e:
            print(f'USB type      : not available', e)
        print("========================================")

    def _print_camera_temperature(self, dev_sn: str):
        _count = self.timestamp // self._temperature_log_interval
        if _count > self._temperature_log_counter[dev_sn]:
            self._temperature_log_counter[dev_sn] = _count
            sensor = self.enabled_devices[dev_sn].depth_sensor
            if sensor.supports(rs.option.asic_temperature):
                temp = sensor.get_option(rs.option.asic_temperature)
                printout(f"{dev_sn} Temperature ASIC : {temp}", 'i')
            if sensor.supports(rs.option.projector_temperature):
                temp = sensor.get_option(rs.option.projector_temperature)
                printout(f"{dev_sn} Temperature Projector : {temp}", 'i')
