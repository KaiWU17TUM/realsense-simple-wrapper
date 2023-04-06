import os
import json
from datetime import datetime
import numpy as np
from typing import Union, Optional, Tuple

from rs_py import rs

from rs_py.utility import printout


class Device:
    pipeline = None
    pipeline_profile = None
    color_sensor = None
    depth_sensor = None
    color_timestamp = 0
    depth_timestamp = 0
    color_reset_counter = 0
    depth_reset_counter = 0
    camera_temp_printout_counter = -1
    num_streams = 0
    sn = ''


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


class DefaultStoragePaths:
    trial = '00000000'
    trial_idx = -1
    _save = False
    device_sns = []
    timestamp = {}
    calib = {}
    color = {}
    depth = {}
    color_metadata = {}
    depth_metadata = {}

    @property
    def save(self):
        return self._save

    @save.setter
    def save(self, x):
        return
    
    def create(self):
        return


class StoragePaths(DefaultStoragePaths):
    _save = True

    @property
    def save(self):
        return self._save

    @save.setter
    def save(self, x):
        self._save = x

    def __init__(self, device_sns: list, base_path: str):
        self.base_path = base_path
        self.device_sns = device_sns
    
    def create(self):
        self.trial = datetime.now().strftime("%y%m%d%H%M%S")
        for device_sn in self.device_sns:
            self._create(device_sn, self.base_path)

    def _create(self, device_sn: str, base_path: str):
        device_path = os.path.join(base_path, device_sn, self.trial)
        # timestamp
        self.timestamp[device_sn] = os.path.join(device_path, 'timestamp')
        os.makedirs(self.timestamp[device_sn], exist_ok=True)
        # calib
        self.calib[device_sn] = os.path.join(device_path, 'calib')
        os.makedirs(self.calib[device_sn], exist_ok=True)
        # color
        self.color[device_sn] = os.path.join(device_path, 'color')
        self.color_metadata[device_sn] = os.path.join(
            device_path, 'color_metadata')
        os.makedirs(self.color[device_sn], exist_ok=True)
        os.makedirs(self.color_metadata[device_sn], exist_ok=True)
        # depth
        self.depth[device_sn] = os.path.join(device_path, 'depth')
        self.depth_metadata[device_sn] = os.path.join(
            device_path, 'depth_metadata')
        os.makedirs(self.depth[device_sn], exist_ok=True)
        os.makedirs(self.depth_metadata[device_sn], exist_ok=True)
        printout("Prepared storage paths...", 'i')

    def show(self, device_sn: str):
        printout("calib : " + self.calib[device_sn], 'i')
        printout("color : " + self.color[device_sn], 'i')
        printout("depth : " + self.depth[device_sn], 'i')
        printout("color_metadata : " + self.color_metadata[device_sn], 'i')
        printout("depth_metadata : " + self.depth_metadata[device_sn], 'i')

    def show_all(self):
        for device_sn in self.device_sns:
            self.show(device_sn)


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


def print_rs2_device_infos(device: rs.device, verbose: bool) -> None:
    """Prints out info about the camera

    Args:
        profile (rs.pipeline_profile): pipeline_profile of a rs device.
    """
    if not verbose:
        return

    print("="*80)
    print(">>>>> RS2_CAMERA_INFO <<<<<")
    print("="*80)
    print(f'Name          : {device.get_info(rs.camera_info.name)}')
    print(f'Serial Number : {device.get_info(rs.camera_info.serial_number)}')
    try:
        print(f'Product Line  : {device.get_info(rs.camera_info.product_line)}')  # noqa
    except Exception as e:
        print(f'Product Line  : not available', e)
    try:
        print(f'Firmware      : {device.get_info(rs.camera_info.firmware_version)}')  # noqa
    except Exception as e:
        print(f'Firmware      : not available', e)
    try:
        print(f'USB type      : {device.get_info(rs.camera_info.usb_type_descriptor)}')  # noqa
    except Exception as e:
        print(f'USB type      : not available', e)
    print("="*80)


def print_camera_temperature(device: Device,
                             printout_interval: int,
                             verbose: bool) -> None:
    """Prints out camera temperature.

    Args:
        device_sn (str): rs camera serial number.
    """
    if not verbose:
        return

    c = device.camera_temp_printout_counter
    if c >= printout_interval or c == -1:
        device.camera_temp_printout_counter = 0
        dss = device.depth_sensor
        if dss.supports(rs.option.asic_temperature):
            temp = dss.get_option(rs.option.asic_temperature)
            printout(f"{device.sn} Temperature ASIC      : {temp}", 'i')
        if dss.supports(rs.option.projector_temperature):
            temp = dss.get_option(rs.option.projector_temperature)
            printout(f"{device.sn} Temperature Projector : {temp}", 'i')


def print_no_device_enabled(function: str = ''):
    printout(f"no device enabled, skipping : {function} ...", 'w')


def save_timestamp(global_timestamp: int,
                   color_timestamp: int,
                   depth_timestamp: int,
                   filename: str):
    """Saves the timestamp.

    Args:
    """
    with open(filename, 'a+') as f:
        f.write(f"{global_timestamp}::{color_timestamp}::{depth_timestamp}\n")


def check_if_color_depth_frames_are_valid(frameset: rs.composite_frame):
    cf = frameset.first_or_default(rs.stream.color)
    df = frameset.first_or_default(rs.stream.depth)
    if cf.get_data_size() == 0 or df.get_data_size() == 0:
        return False
    else:
        return True


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
