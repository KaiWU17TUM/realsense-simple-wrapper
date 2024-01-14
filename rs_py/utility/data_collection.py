import csv
import json
import os
from typing import Tuple, Optional
from .utils import printout


def get_filepaths(base_path: str, sensor: str) -> dict:
    path = {}
    for device in sorted(os.listdir(base_path)):
        path[device] = {}
        device_path = os.path.join(base_path, device)
        for ts in sorted(os.listdir(device_path)):
            path[device][ts] = []
            sensor_path = os.path.join(base_path, device, ts, sensor)
            filenames = sorted(os.listdir(sensor_path))
            # filenames.reverse()
            for filename in filenames:
                path[device][ts].append(os.path.join(sensor_path, filename))
    return path


def get_filepaths_with_timestamps(base_path: str) -> Tuple[dict, dict, list]:
    # dev > trial(as timestamp) > files(with timestamps in the name)
    dev_trial_color_filepaths = get_filepaths(base_path, 'color')
    dev_trial_depth_filepaths = get_filepaths(base_path, 'depth')
    dev_trial_calib_filepaths = get_filepaths(base_path, 'calib')
    try:
        dev_trial_skeleton_filepaths = get_filepaths(base_path, 'skeleton_fromheatmap')
        if len(dev_trial_skeleton_filepaths) == 0:
            dev_trial_skeleton_filepaths = get_filepaths(base_path, 'skeleton_fromrgb')
    except:
        try:
            dev_trial_skeleton_filepaths = get_filepaths(base_path, 'skeleton_fromrgb')
        except:
            dev_trial_skeleton_filepaths = None

    def _get_trial_list(dev_trial_x_filepaths: dict) -> list:
        out = []
        for trial_x_filepaths in dev_trial_x_filepaths.values():
            for trial in trial_x_filepaths.keys():
                if trial not in out:
                    out.append(trial)
        return out

    if dev_trial_skeleton_filepaths:
        trial_list = sorted(list(set(_get_trial_list(dev_trial_color_filepaths) +
                                     _get_trial_list(dev_trial_depth_filepaths) +
                                     _get_trial_list(dev_trial_skeleton_filepaths))))
    else:
        trial_list = sorted(list(set(_get_trial_list(dev_trial_color_filepaths) +
                                     _get_trial_list(dev_trial_depth_filepaths))))

    def _ts_from_filepath(path: str) -> int:
        ts = path.split('/')[-1]
        try:
            ts, extension = os.path.splitext(ts)
            # ts = int(ts.split('.')[0])
        except:
            ts = -1
        return ts

    # trial(as timestamp) > dev > timestamps > [filepath, calib]
    try:
        color_dict = {trial: {} for trial in trial_list}
        for dev, trial_color_filepaths in dev_trial_color_filepaths.items():
            for trial, files in trial_color_filepaths.items():
                color_dict[trial][dev] = {
                    _ts_from_filepath(file):
                    [file, dev_trial_calib_filepaths[dev][trial][0]]
                    for file in files
                }
    except:
        color_dict = None
    try:
        depth_dict = {trial: {} for trial in trial_list}
        for dev, trial_depth_filepaths in dev_trial_depth_filepaths.items():
            for trial, files in trial_depth_filepaths.items():
                depth_dict[trial][dev] = {
                    _ts_from_filepath(file):
                    [file, dev_trial_calib_filepaths[dev][trial][0]]
                    for file in files
                }
    except              :
        depth_dict = None
    try:
        skeleton_dict = {trial: {} for trial in trial_list}
        if dev_trial_skeleton_filepaths:
            for dev, trial_skeleton_filepaths in dev_trial_skeleton_filepaths.items():
                for trial, files in trial_skeleton_filepaths.items():
                    skeleton_dict[trial][dev] = {
                        _ts_from_filepath(file):
                            [file, dev_trial_calib_filepaths[dev][trial][0]]
                        for file in files
                    }
    except:
        skeleton_dict = None


    return color_dict, depth_dict, skeleton_dict, trial_list


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


def iterate_over_raw_data(base_path: Optional[str] = None,
                          sync_ts: int = 0,
                          fps: int = 6,
                          scale: float = 1.0,
                          data_process_fn: Optional[callable] = None):

    # trial(as timestamp) > dev > timestamps > [filepath, calib]
    color_dict, depth_dict, _, trial_list = \
        get_filepaths_with_timestamps(base_path)

    # 1. loop through each trial
    for trial in trial_list:
        print(f"trial : {trial}")

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

        next_trial = True
        try:
            while next_trial:

                if sync_ts == 0:
                    _color_ts_idxs = [ts_max for _ in range(len(color_files))]
                    _depth_ts_idxs = [ts_max for _ in range(len(depth_files))]
                    ts_max += 1

                elif sync_ts == 1:
                    # only 1 camera
                    if len(color_files) == 1:
                        continue

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

                for idx, files in zip(_color_ts_idxs, color_files):
                    if idx >= len(files):
                        next_trial = False

                for idx, files in zip(_depth_ts_idxs, depth_files):
                    if idx >= len(files):
                        next_trial = False
                
                if not next_trial:
                    continue

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


def iterate_over_depth_skeleton(base_path: Optional[str] = None,
                          sync_ts: int = 0,
                          fps: int = 6,
                          scale: float = 1.0,
                          data_process_fn: Optional[callable] = None):

    # trial(as timestamp) > dev > timestamps > [filepath, calib]
    _, depth_dict, skeleton_dict, trial_list = \
        get_filepaths_with_timestamps(base_path)

    # 1. loop through each trial
    for trial in trial_list:
        print(f"trial : {trial}")

        # #dev
        depth_files, skeleton_files = [], []

        # 2. prepare data tuple, collect all devices of same trial together
        # dev > #frames > (ts, file, calib)
        for data_tuple in zip(depth_dict[trial].items(), skeleton_dict[trial].items()):
            dev = data_tuple[0][0]
            _depth_files = [
                DataContainer(*files, ts, dev)
                for ts, files in data_tuple[0][1].items()
            ]
            _skeleton_files = [
                DataContainer(*files, ts, dev)
                for ts, files in data_tuple[1][1].items()
            ]
            depth_files.append(_depth_files)
            skeleton_files.append(_skeleton_files)

        if sync_ts == 0:
            ts_max = 0
        elif sync_ts == 1:
            ts_depth_max = max([i[0].timestamp for i in depth_files])
            ts_skeleton_max = max([i[0].timestamp for i in skeleton_files])
            ts_max = max([ts_depth_max, ts_skeleton_max])
            counter_depth = [0] * len(depth_files)
            counter_skeleton = [0] * len(skeleton_files)
        else:
            raise ValueError("Unknown SYNC mode...")

        next_trial = True
        try:
            while next_trial:

                if sync_ts == 0:
                    _depth_ts_idxs = [ts_max for _ in range(len(depth_files))]
                    _skeleton_ts_idxs = [ts_max for _ in range(len(skeleton_files))]
                    ts_max += 1

                elif sync_ts == 1:
                    # only 1 camera
                    if len(depth_files) == 1:
                        continue

                    _depth_ts = []
                    _skeleton_ts = []
                    _depth_ts_idxs = []
                    _skeleton_ts_idxs = []

                    for idx, file in enumerate(depth_files):
                        ts_idx, ts = next(
                            (i, dc.timestamp)
                            for i, dc in enumerate(file[counter_depth[idx]:])
                            if dc.timestamp >= ts_max
                        )
                        _depth_ts.append(ts)
                        _depth_ts_idxs.append(ts_idx)
                    for idx, file in enumerate(skeleton_files):
                        ts_idx, ts = next(
                            (i, dc.timestamp)
                            for i, dc in enumerate(file[counter_skeleton[idx]:])
                            if dc.timestamp >= ts_max
                        )
                        _skeleton_ts.append(ts)
                        _skeleton_ts_idxs.append(ts_idx)
                    ts_max = max(_depth_ts + _skeleton_ts)

                for idx, files in zip(_depth_ts_idxs, depth_files):
                    if idx >= len(files):
                        next_trial = False

                for idx, files in zip(_skeleton_ts_idxs, skeleton_files):
                    if idx >= len(files):
                        next_trial = False

                if not next_trial:
                    continue

                if data_process_fn is not None:
                    data_process_fn(depth_files=depth_files,
                                    skeleton_files=skeleton_files,
                                    depth_ts_idxs=_depth_ts_idxs,
                                    skeleton_ts_idxs=_skeleton_ts_idxs,
                                    sync_ts=sync_ts,
                                    scale=scale,
                                    fps=fps
                                    )

        except Exception as e:
            printout(e, 'x')
            exit(1)