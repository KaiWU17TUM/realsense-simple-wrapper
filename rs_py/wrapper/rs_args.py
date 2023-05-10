import argparse
import numpy as np

from ..utility import str2bool

from rs_py import rs


DEPTH_SENSOR_AE_LIMIT = 200000.0


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


def get_rs_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description='Run RealSense devices.')
    parser.add_argument('--rs-steps',
                        type=int,
                        default=3000,
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
    parser.add_argument('--rs-display-frame',
                        type=int,
                        default=1,
                        help='scale for displaying realsense raw images.')
    parser.add_argument('--rs-save-with-key',
                        type=str2bool,
                        default=False,
                        help='save using key = "c" ')
    parser.add_argument('--rs-save-data',
                        type=str2bool,
                        default=False,
                        help='Whether to save data')
    parser.add_argument('--rs-save-path',
                        type=str,
                        default='/data/realsense',
                        help='path to save realsense frames if --rs-save-data=True.')  # noqa
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
    parser.add_argument('--rs-verbose',
                        type=str2bool,
                        default=False,
                        help='Whether to printout info')
    parser.add_argument('--rs-autoexposure',
                        type=str2bool,
                        default=True,
                        help='Whether to habe autoexposure')
    parser.add_argument('--rs-depth-sensor-autoexposure-limit',
                        type=float,
                        default=DEPTH_SENSOR_AE_LIMIT,
                        help='autoexposure limit for depth sensor')
    parser.add_argument('--rs-enable-ir-emitter',
                        type=str2bool,
                        default=True,
                        help='Whether to enable ir emitter')
    parser.add_argument('--rs-ir-emitter-power',
                        type=int,
                        default=300,
                        help='laser power')
    return parser
