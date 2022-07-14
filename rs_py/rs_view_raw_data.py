import cv2
import json
import numpy as np
import os
import sys

from typing import Tuple, Optional

from rs_py import printout


class DataContainer:
    def __init__(self,
                 file=None,
                 calib_file=None,
                 timestamp=None,
                 device_sn=None) -> None:
        self.file = file
        self.calib_file = calib_file
        self.timestamp = timestamp
        self.device_sn = device_sn


# https://github.com/IntelRealSense/librealsense/issues/4646
def _get_brg_from_yuv(data_array: np.ndarray) -> np.ndarray:
    # Input: Intel handle to 16-bit YU/YV data
    # Output: BGR8
    UV = np.uint8(data_array >> 8)
    Y = np.uint8((data_array << 8) >> 8)
    YUV = np.zeros((data_array.shape[0], 2), 'uint8')
    YUV[:, 0] = Y
    YUV[:, 1] = UV
    YUV = YUV.reshape(-1, 1, 2)
    BGR = cv2.cvtColor(YUV, cv2.COLOR_YUV2BGR_YUYV)
    BGR = BGR.reshape(-1, 3)
    return BGR


def read_color_file(color_file: str) -> np.ndarray:
    if color_file.endswith('.bin'):
        with open(color_file, 'rb') as f:
            image = np.fromfile(f, np.uint8)
    else:
        image = np.load(color_file)
        if image.dtype == np.dtype(np.uint8):
            pass
        elif image.dtype == np.dtype(np.uint16):
            image = _get_brg_from_yuv(image.reshape(-1))
        else:
            raise ValueError("Unknown data type :", image.dtype)
    return image


def read_depth_file(depth_file: str) -> np.ndarray:
    if depth_file.endswith('.bin'):
        with open(depth_file, 'rb') as f:
            depth = np.fromfile(f, np.uint16)
    else:
        depth = np.fromfile(depth_file, np.uint16)
    return depth


def get_filepaths(base_path: str, sensor: str) -> dict:
    path = {}
    for device in sorted(os.listdir(base_path)):
        path[device] = {}
        device_path = os.path.join(base_path, device)
        for ts in sorted(os.listdir(device_path)):
            path[device][ts] = []
            sensor_path = os.path.join(base_path, device, ts, sensor)
            for filename in sorted(os.listdir(sensor_path)):
                path[device][ts].append(os.path.join(sensor_path, filename))
    return path


def get_filepaths_with_timestamps(base_path: str) -> Tuple[dict, dict, list]:
    # dev > trial(as timestamp) > files(with timestamps in the name)
    dev_trial_color_filepaths = get_filepaths(base_path, 'color')
    dev_trial_depth_filepaths = get_filepaths(base_path, 'depth')
    dev_trial_calib_filepaths = get_filepaths(base_path, 'calib')

    def _get_trial_list(dev_trial_x_filepaths: dict) -> list:
        out = []
        for trial_x_filepaths in dev_trial_x_filepaths.values():
            for trial in trial_x_filepaths.keys():
                if trial not in out:
                    out.append(trial)
        return out

    trial_list = list(set(_get_trial_list(dev_trial_color_filepaths) +
                          _get_trial_list(dev_trial_depth_filepaths)))

    def _ts_from_filepath(path: str) -> int:
        ts = path.split('/')[-1]
        ts = int(ts.split('.')[0])
        return ts

    # trial(as timestamp) > dev > timestamps > [filepath, calib]
    color_dict = {trial: {} for trial in trial_list}
    for dev, trial_color_filepaths in dev_trial_color_filepaths.items():
        for trial, files in trial_color_filepaths.items():
            color_dict[trial][dev] = {
                _ts_from_filepath(file):
                [file, dev_trial_calib_filepaths[dev][trial][0]]
                for file in files
            }
    depth_dict = {trial: {} for trial in trial_list}
    for dev, trial_depth_filepaths in dev_trial_depth_filepaths.items():
        for trial, files in trial_depth_filepaths.items():
            depth_dict[trial][dev] = {
                _ts_from_filepath(file):
                [file, dev_trial_calib_filepaths[dev][trial][0]]
                for file in files
            }

    return color_dict, depth_dict, trial_list


def iterate_over_raw_data(base_path: Optional[str] = None,
                          sync_ts: int = 0,
                          fps: int = 6,
                          scale: float = 1.0,
                          data_process_fn: Optional[callable] = None):

    # trial(as timestamp) > dev > timestamps > [filepath, calib]
    color_dict, depth_dict, trial_list = \
        get_filepaths_with_timestamps(base_path)

    # 1. loop through each trial
    for trial in trial_list:
        # #dev
        color_files, depth_files = [], []

        # 2. prepare data tuple, collect all devices of same trial together
        # dev > #frames > (ts, file, calib)
        for data_tuple in zip(color_dict[trial].items(),
                              depth_dict[trial].items()):
            dev = data_tuple[0][0]
            _color_files = [
                DataContainer(*files, ts, dev)
                for ts, files in data_tuple[0][1].items()
            ]
            _depth_files = [
                DataContainer(*files, ts, dev)
                for ts, files in data_tuple[1][1].items()
            ]
            color_files.append(_color_files)
            depth_files.append(_depth_files)

        if sync_ts == 0:
            ts_max = 0
        elif sync_ts == 1:
            ts_color_max = max([i[0].timestamp for i in color_files])
            ts_depth_max = max([i[0].timestamp for i in depth_files])
            ts_max = max([ts_color_max, ts_depth_max])
            counter_color = [0] * len(color_files)
            counter_depth = [0] * len(depth_files)
        else:
            raise ValueError("Unknown SYNC mode...")

        try:
            while True:

                if sync_ts == 0:
                    _color_ts_idxs = [ts_max for _ in range(len(color_files))]
                    _depth_ts_idxs = [ts_max for _ in range(len(depth_files))]
                    ts_max += 1

                elif sync_ts == 1:
                    _color_ts = []
                    _depth_ts = []
                    _color_ts_idxs = []
                    _depth_ts_idxs = []
                    for idx, file in enumerate(color_files):
                        ts_idx, ts = next(
                            (i, dc.timestamp)
                            for i, dc in enumerate(file[counter_color[idx]:])
                            if dc.timestamp >= ts_max
                        )
                        _color_ts.append(ts)
                        _color_ts_idxs.append(ts_idx)
                    for idx, file in enumerate(depth_files):
                        ts_idx, ts = next(
                            (i, dc.timestamp)
                            for i, dc in enumerate(file[counter_depth[idx]:])
                            if dc.timestamp >= ts_max
                        )
                        _depth_ts.append(ts)
                        _depth_ts_idxs.append(ts_idx)
                    ts_max = max(_color_ts + _depth_ts)

                if data_process_fn is not None:
                    data_process_fn(color_files=color_files,
                                    depth_files=depth_files,
                                    color_ts_idxs=_color_ts_idxs,
                                    depth_ts_idxs=_depth_ts_idxs,
                                    sync_ts=sync_ts,
                                    scale=scale,
                                    fps=fps
                                    )

        except Exception as e:
            printout(e, 'x')
            exit(1)

        finally:
            exit(0)


def data_process_fn(**kwargs):

    color_files = kwargs['color_files']
    depth_files = kwargs['depth_files']
    color_ts_idxs = kwargs['color_ts_idxs']
    depth_ts_idxs = kwargs['depth_ts_idxs']
    sync_ts = kwargs['sync_ts']
    scale = kwargs['scale']
    fps = kwargs['fps']

    imgs = []
    num_dev = len(color_files)
    for dev_idx in range(num_dev):
        # (ts, file, calib)
        color_dc = color_files[dev_idx][color_ts_idxs[dev_idx]]
        depth_dc = depth_files[dev_idx][depth_ts_idxs[dev_idx]]

        with open(color_dc.calib_file, 'r') as f:
            calib_data = json.load(f)

        h_c = calib_data['color']['height']
        w_c = calib_data['color']['width']
        h_d = calib_data['depth']['height']
        w_d = calib_data['depth']['width']

        image = read_color_file(color_dc.file)
        depth = read_depth_file(depth_dc.file)

        image = image.reshape(h_c, w_c, 3)
        depth = depth[-h_d*w_d:].reshape(h_d, w_d)
        # depth_i = depth[-(h_d//3)*(w_d//3+2):].reshape(h_d//3, w_d//3+2)
        # depth = cv2.resize(depth_i, (w_d, h_d))

        depth = cv2.applyColorMap(
            cv2.convertScaleAbs(depth, alpha=0.03),
            cv2.COLORMAP_JET
        )

        cv2.putText(image,
                    f"{color_dc.device_sn} - {color_dc.timestamp}",
                    (10, 30),
                    cv2.FONT_HERSHEY_PLAIN,
                    2,
                    (0, 255, 0),
                    2,
                    cv2.LINE_AA)
        cv2.putText(image,
                    f"{depth_dc.device_sn} - {depth_dc.timestamp}",
                    (10, 80),
                    cv2.FONT_HERSHEY_PLAIN,
                    2,
                    (0, 255, 0),
                    2,
                    cv2.LINE_AA)
        imgs.append(np.vstack([image, depth]))

    output = np.hstack(imgs)
    output = cv2.resize(output, (int(output.shape[1]*scale),
                                 int(output.shape[0]*scale)))

    name = f'output - sync={sync_ts}'
    cv2.namedWindow(name)
    cv2.moveWindow(name, 0, 0)
    cv2.imshow(name, output)
    # cv2.waitKey(1000//fps)
    cv2.waitKey(0)


if __name__ == "__main__":

    PATH = sys.argv[1]
    SYNC = sys.argv[2]
    FPS = sys.argv[3]
    SCALE = sys.argv[4]

    # PATH = '/code/realsense-simple-wrapper/output/calibration_848_480'
    # SYNC = 1
    # FPS = 5
    # SCALE = 0.75

    iterate_over_raw_data(PATH, SYNC, FPS, SCALE,
                          data_process_fn=data_process_fn)
