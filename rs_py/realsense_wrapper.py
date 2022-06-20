# Network part is based on :
# https://github.com/IntelRealSense/librealsense/blob/master/wrappers/python/examples/net_viewer.py

import argparse
import cv2
import json
import numpy as np
import os

try:
    from pyrealsense2 import pyrealsense2 as rs
except ImportError as e:
    print('Error: pyrealsense2 library could not be found.')
    raise e
try:
    from pyrealsense2 import pyrealsense2_net as rsnet
except ImportError as e:
    print('Error: pyrealsense2_net could not be found.')
    raise e

from datetime import datetime
from functools import partial
from typing import Optional, Tuple, Union

from rs_py.realsense_device_manager import Device
from rs_py.realsense_device_manager import enumerate_connected_devices
from rs_py.realsense_device_manager import post_process_depth_frame


class CalibrationConfig:
    def __init__(self):
        self.device_sn = []
        self.color = []
        self.depth = []
        self.T_color_depth = []

    def save(self, file_path: str) -> None:
        self.validate()
        with open(file_path, 'w') as outfile:
            json.dump(self.__dict__, outfile, indent=4)

    def load(self, file_path: str) -> None:
        with open(file_path) as calib_file:
            calib_data = json.load(calib_file)
        self.__dict__.update(calib_data)
        self.validate()

    def get_T_color_depth(self, idx: int) -> np.ndarray:
        T_color_depth = np.eye(4)
        T_color_depth[:3, :3] = np.array(self.T_color_depth[idx]['rotation']).reshape(3, 3)  # noqa
        T_color_depth[:3, 3] = self.T_color_depth[idx]['translation']
        return T_color_depth

    def get_data(self, device_sn_idx: Union[int, str]):
        if isinstance(device_sn_idx, int):
            idx = device_sn_idx
        elif isinstance(device_sn_idx, str):
            idx = self.device_sn.index(device_sn_idx)
        else:
            raise ValueError("Unknown input arguemtn type...")
        return {
            'color': self.color[idx],
            'depth': self.depth[idx],
            'T_color_depth': self.get_T_color_depth(idx),
        }

    def validate(self) -> bool:
        assert len(self.device_sn) == len(self.color)
        assert len(self.device_sn) == len(self.depth)
        assert len(self.device_sn) == len(self.T_color_depth)


class StoragePaths:
    def __init__(self, device_sn: str = '', base_path: str = '/data/realsense'):
        date_time = datetime.now().strftime("%y%m%d%H%M%S")
        device_path = os.path.join(base_path, f'dev_{device_sn}', date_time)
        self.meta_color = os.path.join(device_path, 'meta_color')
        self.meta_depth = os.path.join(device_path, 'meta_depth')
        self.calib = os.path.join(device_path, 'calib')
        self.color = os.path.join(device_path, 'color')
        self.depth = os.path.join(device_path, 'depth')
        self.timestamp = os.path.join(device_path, 'timestamp')
        self.timestamp_file = os.path.join(self.timestamp, 'timestamp.txt')
        os.makedirs(self.meta_color, exist_ok=True)
        os.makedirs(self.meta_depth, exist_ok=True)
        os.makedirs(self.calib, exist_ok=True)
        os.makedirs(self.color, exist_ok=True)
        os.makedirs(self.depth, exist_ok=True)
        os.makedirs(self.timestamp, exist_ok=True)
        print(f"[INFO] : Prepared storage paths...")


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


class RealsenseWrapper:
    """Wrapper to run multiple realsense cameras.

    Code is written based on "realsense_device_manager.py" . Currently the
    code supports only reading depth and color images. Reading of IR stream
    is not implemented.

    """

    def __init__(self, arg: argparse.Namespace) -> None:
        super().__init__()

        # device data
        self.ctx = rs.context()
        if arg.rs_ip is not None:
            self.network = True
            dev = rsnet.net_device(arg.rs_ip)
            self.available_devices = [
                (dev.get_info(rs.camera_info.serial_number), None)]
            dev.add_to(self.ctx)
            print(f'[INFO] : Network mode')
            print(f'[INFO] : Connected to {arg.rs_ip}')
        else:
            self.network = False
            self.available_devices = enumerate_connected_devices(self.ctx)
            print(f'[INFO] : Local mode')

        if arg.rs_use_one_dev_only:
            self.available_devices = self.available_devices[0:1]

        # serial numbers of enabled devices
        self.enabled_devices = {}
        self.calib_data = {}

        # rs align method
        self._align = rs.align(rs.stream.color)  # align depth to color frame

        # configurations
        self._rs_cfg = {}
        self.stream_config_color = StreamConfig(
            rs.stream.color, arg.rs_image_width, arg.rs_image_height,
            rs.format.bgr8, arg.rs_fps
        )
        self.stream_config_depth = StreamConfig(
            rs.stream.depth, arg.rs_image_width, arg.rs_image_height,
            rs.format.z16, arg.rs_fps
        )

        # Save paths
        self.timestamp_mode = None
        self.storage_paths_per_dev = {}
        if arg.rs_save_data:
            if arg.rs_save_path is not None:
                storage_paths_fn = partial(
                    StoragePaths, base_path=arg.rs_save_path)
            else:
                storage_paths_fn = StoragePaths
            self.storage_paths_per_dev = {sn: storage_paths_fn(sn)
                                          for sn, _ in self.available_devices}

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
        if device_sn is not None:
            cfg = rs.config()
            if stream_config_depth is not None:
                cfg.enable_stream(**stream_config_depth.data)
            if stream_config_color is not None:
                cfg.enable_stream(**stream_config_color.data)
            self._rs_cfg[device_sn] = cfg

    def initialize(self, enable_ir_emitter: bool = True) -> None:
        """Initializes the device pipelines and starts them.

        Args:
            enable_ir_emitter (bool, optional): Enable the IR for beter
                depth quality. Defaults to True.
        """
        if len(self._rs_cfg) == 0:
            self.configure_stream('default',
                                  self.stream_config_depth,
                                  self.stream_config_color)

        for device_serial, product_line in self.available_devices:

            # Pipeline
            if self.network:
                pipeline = rs.pipeline(self.ctx)
            else:
                pipeline = rs.pipeline()

            cfg = self._rs_cfg.get(device_serial, self._rs_cfg['default'])
            if not self.network:
                cfg.enable_device(device_serial)
            check = cfg.can_resolve(pipeline)
            print(f"[INFO] : 'cfg' usable with 'pipeline' : {check}")

            pipeline_profile = pipeline.start(cfg)

            # IR for depth
            if enable_ir_emitter:
                d_sensor = pipeline_profile.get_device().first_depth_sensor()
                if d_sensor.supports(rs.option.emitter_enabled):
                    d_sensor.set_option(rs.option.emitter_enabled,
                                        1 if enable_ir_emitter else 0)
                    # d_sensor.set_option(rs.option.laser_power, 330)

            # Stored the enabled devices
            self.enabled_devices[device_serial] = (
                Device(pipeline, pipeline_profile, product_line))

            print("========================================")
            print(">>>>> camera info <<<<<")
            print("========================================")
            print(f'Name          : {pipeline_profile.get_device().get_info(rs.camera_info.name)}')  # noqa
            print(f'Serial Number : {pipeline_profile.get_device().get_info(rs.camera_info.serial_number)}')  # noqa
            try:
                print(f'Product Line  : {pipeline_profile.get_device().get_info(rs.camera_info.product_line)}')  # noqa
            except Exception as e:
                print(f'Product Line  : not available', e)
            try:
                print(f'Firmware      : {pipeline_profile.get_device().get_info(rs.camera_info.firmware_version)}')  # noqa
            except Exception as e:
                print(f'Firmware      : not available', e)  # noqa
            print("========================================")

            # Check which timestamp is available.
            if len(self.storage_paths_per_dev) > 0:
                wait_flag = True
                while wait_flag:
                    streams = pipeline_profile.get_streams()
                    frameset = pipeline.poll_for_frames()
                    if frameset.size() == len(streams):
                        wait_flag = False
                fmv = rs.frame_metadata_value
                if frameset.supports_frame_metadata(fmv.sensor_timestamp):
                    self.timestamp_mode = fmv.sensor_timestamp
                    print(f'[INFO] : sensor_timestamp is being used...')
                elif frameset.supports_frame_metadata(fmv.frame_timestamp):
                    self.timestamp_mode = fmv.frame_timestamp
                    print(f'[INFO] : frame_timestamp is being used...')
                else:
                    self.storage_paths_per_dev.pop(device_serial)
                    print('[WARN] : Both sensor_timestamp/frame_timestamp '
                          'are not available. No data will be saved...')

        print("[INFO] : Initialized RealSense devices...")

    def set_storage_paths(self, paths: StoragePaths) -> None:
        self.storage_paths_per_dev = {sn: paths(sn)
                                      for sn in self.enabled_devices}

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

    def get_timestamp(self, frame: rs.frame) -> int:
        if self.timestamp_mode is None:
            return -1
        else:
            return frame.get_frame_metadata(self.timestamp_mode)

    def get_color_stream(self,
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
        timestamp = self.get_timestamp(frame)
        frame_dict['timestamp_color'] = timestamp
        frame_data = np.asanyarray(frame.get_data())
        frame_dict['color'] = frame_data
        if storage_paths is not None:
            filepath = storage_paths.color
            if filepath is not None:
                np.save(os.path.join(filepath, f'{timestamp}'), frame_data)
            filepath = storage_paths.meta_color
            if filepath is not None:
                path = os.path.join(filepath, f'{timestamp}.json')
                with open(path, 'w') as json_f:
                    json.dump(read_metadata(frame), json_f, indent=4)
        return frame_dict, frame_data

    def get_depth_stream(self,
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
        frame = post_process_depth_frame(frame)
        timestamp = self.get_timestamp(frame)
        frame_dict['timestamp_depth'] = timestamp
        frame_data = np.asanyarray(frame.get_data())
        frame_dict['depth'] = frame_data
        if storage_paths is not None:
            filepath = storage_paths.depth
            if filepath is not None:
                np.save(os.path.join(filepath, f'{timestamp}'), frame_data)
            if save_colormap:
                # Apply colormap on depth image
                # (image must be converted to 8-bit per pixel first)
                image = cv2.applyColorMap(
                    cv2.convertScaleAbs(frame_data, alpha=0.03),
                    cv2.COLORMAP_JET
                )
                image_name = os.path.join(filepath, f'{timestamp}.jpg')
                cv2.imwrite(image_name, image)
            filepath = storage_paths.meta_depth
            if filepath is not None:
                path = os.path.join(filepath, f'{timestamp}.json')
                with open(path, 'w') as json_f:
                    json.dump(read_metadata(frame), json_f, indent=4)
        return frame_dict, frame_data

    def dummy_capture(self, num_frames: int = 30) -> None:
        """Dummy capture 'num_frames' frames to give
        autoexposure, etc. a chance to settle.

        Args:
            num_frames (int): Number of dummy frames to skip. Defaults to 30.
        """
        print("[INFO] : Capturing dummy frames...")
        frames = {}
        for _ in range(num_frames):
            while len(frames) < len(self.enabled_devices.items()):
                for dev_sn, dev in self.enabled_devices.items():
                    streams = dev.pipeline_profile.get_streams()
                    frameset = dev.pipeline.poll_for_frames()
                    if frameset.size() == len(streams):
                        frames[dev_sn] = {}
        print("[INFO] : Finished capturing dummy frames...")

    def step(self, display: int = 0, save_depth_colormap: bool = False) -> dict:
        """Gets the frames streamed from the enabled rs devices.

        Args:
            display (int, optional): Whether to display the retrieved frames.
                The value corresponds to the scale to visualize the frames.
                Defaults to 0 = no display.

        Returns:
            dict: Empty dict or {serial_number: {data_type: data}}.
                data_type = color, depth, timestamp, calib
        """
        if len(self.enabled_devices) == 0:
            print("[WARN] : No devices are enabled...")
            return {}

        frames = {}
        while len(frames) < len(self.enabled_devices.items()):

            for dev_sn, dev in self.enabled_devices.items():

                storage_paths = self.storage_paths_per_dev.get(dev_sn, None)

                streams = dev.pipeline_profile.get_streams()
                frameset = dev.pipeline.poll_for_frames()

                if frameset.size() == len(streams):
                    frame_dict = {}

                    frame_dict['calib'] = self.calib_data.get(dev_sn, {})

                    # aligned_frameset = self._align.process(frameset)
                    aligned_frameset = frameset
                    for stream in streams:
                        st = stream.stream_type()
                        # if stream.stream_type() == rs.stream.infrared:
                        #     frame = aligned_frameset.get_infrared_frame(
                        #         stream.stream_index())
                        #     key_ = (stream.stream_type(),
                        #             stream.stream_index())
                        # frame = aligned_frameset.first_or_default(st)
                        # frame_data = frame.get_data()
                        # frame_dict[st] = frame_data
                        if st == rs.stream.color:
                            frame_dict, frame_data = self.get_color_stream(
                                frameset=aligned_frameset,
                                frame_dict=frame_dict,
                                storage_paths=storage_paths,
                            )
                        elif st == rs.stream.depth:
                            frame_dict, frame_data = self.get_depth_stream(
                                frameset=aligned_frameset,
                                frame_dict=frame_dict,
                                storage_paths=storage_paths,
                                save_colormap=save_depth_colormap
                            )

                    if storage_paths is not None:
                        ts_file = storage_paths.timestamp_file
                        if ts_file is not None:
                            with open(ts_file, 'a+') as f:
                                f.write(f"{frame_dict['timestamp_color']}::{frame_dict['timestamp_depth']}\n")  # noqa

                    frames[dev_sn] = frame_dict

        if display > 0:
            if self._display_rs_data(frames, display):
                return {}

        return frames

    def stop(self) -> None:
        """Stops the devices. """
        if len(self.enabled_devices) == 0:
            print("[WARN] : No devices are enabled...")
        else:
            for _, dev in self.enabled_devices.items():
                dev.pipeline.stop()

    def _display_rs_data(self, frames: dict, scale: int) -> bool:
        terminate = False
        for dev_sn, data_dict in frames.items():
            # Render images
            depth_colormap = cv2.applyColorMap(
                cv2.convertScaleAbs(data_dict['depth'], alpha=0.03),
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
                data_dict['color'], 0.3, depth_colormap, 0.5, 0)
            images = np.hstack(
                (data_dict['color'], depth_colormap, images_overlapped))
            images = cv2.resize(images, (images.shape[1]//scale,
                                         images.shape[0]//scale))
            cv2.namedWindow(f'{dev_sn}', cv2.WINDOW_AUTOSIZE)
            cv2.imshow(f'{dev_sn}', images)
            key = cv2.waitKey(30)
            # Press esc or 'q' to close the image window
            if key & 0xFF == ord('q') or key == 27:
                cv2.destroyAllWindows()
                cv2.waitKey(5)
                terminate = True

        return terminate

    def save_calibration(self) -> None:
        """Saves camera calibration. """

        if len(self.enabled_devices) == 0:
            print("[WARN] : No devices are enabled...")
            return

        calib_config = CalibrationConfig()
        for dev_sn, dev in self.enabled_devices.items():

            storage_paths = self.storage_paths_per_dev.get(dev_sn, None)
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
                'model': intr_color.model,
                'coeffs': intr_color.coeffs
            })
            calib_config.depth.append({
                'width': intr_depth.width,
                'height': intr_depth.height,
                'intrinsic_mat': [intr_depth.fx, 0, intr_depth.ppx,
                                  0, intr_depth.fy, intr_depth.ppy,
                                  0, 0, 1],
                'model': intr_depth.model,
                'coeffs': intr_depth.coeffs,
                'depth_scale': depth_scale,
                'depth_baseline': depth_baseline
            })
            calib_config.T_color_depth.append({
                'rotation': extr.rotation,
                'translation': extr.translation
            })

            calib_config.save(save_path)

        print("[INFO] : Saved camera calibration data...")


def read_metadata(frame: rs.frame) -> dict:
    frame_metadata_value_list = [
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
    for i in frame_metadata_value_list:
        if frame.supports_frame_metadata(i):
            output[i.name] = frame.get_frame_metadata(i)
    return output


def str2bool(v) -> bool:
    if v.lower() in ('yes', 'true', 't', 'y', '1'):
        return True
    elif v.lower() in ('no', 'false', 'f', 'n', '0'):
        return False
    else:
        raise argparse.ArgumentTypeError('Boolean value expected.')


def get_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description='Run RealSense devices.')
    parser.add_argument('--rs-fps',
                        type=int,
                        default=6,
                        help='fps')
    parser.add_argument('--rs-image-width',
                        type=int,
                        default=640,
                        help='image width in px')
    parser.add_argument('--rs-image-height',
                        type=int,
                        default=480,
                        help='image height in px')
    parser.add_argument('--rs-laser-power',
                        type=int,
                        default=140,
                        help='laser power')
    parser.add_argument('--rs-stream-color',
                        type=str2bool,
                        default=True,
                        help='enable color stream')
    parser.add_argument('--rs-stream-depth',
                        type=str2bool,
                        default=True,
                        help='enable depth stream')
    parser.add_argument('--rs-display-frame',
                        type=int,
                        default=0,
                        help='scale for displaying realsense raw images.')
    parser.add_argument('--rs-save-data',
                        type=str2bool,
                        default=False,
                        help='if true, saves realsense frames.')
    parser.add_argument('--rs-save-path',
                        type=str,
                        default='/data/realsense',
                        help='path to save realsense frames if --rs-save-data=True.')  # noqa
    parser.add_argument('--rs-use-one-dev-only',
                        type=str2bool,
                        default=False,
                        help='use 1 rs device only.')
    parser.add_argument('--rs-ip',
                        nargs='*',
                        # type=str,
                        # default='192.168.100.39',  # 101 LAN
                        # default='192.168.1.216',  # 101 WLAN
                        # default='192.168.1.11',  # 102 WLAN
                        help='ip address')
    return parser
