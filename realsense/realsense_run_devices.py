import argparse
import os

from datetime import datetime
from functools import partial
from typing import Type, Optional

from realsense import RealsenseWrapper
from realsense import StoragePaths


class RealsenseStoragePaths(StoragePaths):
    def __init__(self, device_sn: str = '', base_path: str = '/data/realsense'):
        super().__init__()
        date_time = datetime.now().strftime("%y%m%d%H%M%S")
        self.calib = f'{base_path}/calib/{date_time}_dev{device_sn}'
        self.color = f'{base_path}/color/{date_time}_dev{device_sn}'
        self.depth = f'{base_path}/depth/{date_time}_dev{device_sn}'
        self.timestamp = f'{base_path}/timestamp/{date_time}_dev{device_sn}'
        self.timestamp_file = os.path.join(self.timestamp, 'timestamp.txt')
        os.makedirs(self.calib, exist_ok=True)
        os.makedirs(self.color, exist_ok=True)
        os.makedirs(self.depth, exist_ok=True)
        os.makedirs(self.timestamp, exist_ok=True)


def str2bool(v):
    if v.lower() in ('yes', 'true', 't', 'y', '1'):
        return True
    elif v.lower() in ('no', 'false', 'f', 'n', '0'):
        return False
    else:
        raise argparse.ArgumentTypeError('Boolean value expected.')


def get_parser():
    parser = argparse.ArgumentParser(description='Run RealSense devices.')
    parser.add_argument('--rs-fps',
                        type=int,
                        default=30,
                        help='fps')
    parser.add_argument('--rs-image-width',
                        type=int,
                        default=848,
                        help='image width in px')
    parser.add_argument('--rs-image-height',
                        type=int,
                        default=480,
                        help='image height in px')
    parser.add_argument('--rs-laser-power',
                        type=int,
                        default=150,
                        help='laser power')
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
    return parser


def initialize_rs_devices(
        arg: argparse.Namespace,
        storage_paths: Optional[Type[StoragePaths]] = RealsenseStoragePaths
) -> RealsenseWrapper:
    storage_paths_fn = partial(storage_paths, base_path=arg.rs_save_path)
    rsw = RealsenseWrapper(storage_paths_fn if arg.rs_save_data else None)
    rsw.stream_config.fps = arg.rs_fps
    rsw.stream_config.height = arg.rs_image_height
    rsw.stream_config.width = arg.rs_image_width
    if arg.rs_use_one_dev_only:
        rsw.available_devices = rsw.available_devices[0:1]
    rsw.initialize()
    rsw.set_ir_laser_power(arg.rs_laser_power)
    rsw.save_calibration()
    print("Initialized RealSense devices...")
    return rsw


if __name__ == "__main__":
    arg = get_parser().parse_args()
    rsw = initialize_rs_devices(arg)
    rsw.dummy_capture()

    print("Starting frame capture loop...")
    try:
        c = 0
        while True:
            frames = rsw.run(display=arg.rs_display_frame)
            if not len(frames) > 0:
                print("Empty...")
                continue
            else:
                print("Running...")
            c += 1
            if c > arg.rsnet_fps * 10:
                break

    except:  # noqa
        print("Stopping RealSense devices...")
        rsw.stop()

    finally:
        rsw.stop()
