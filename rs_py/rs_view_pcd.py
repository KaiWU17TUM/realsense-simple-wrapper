import cv2
import json
import numpy as np
import os
import sys

import open3d as o3d

# Currently the code only shows the data from the newest timestamp folder.

# PATH = '/code/realsense-simple-wrapper/output/rs_sample_3dev'

BASE_PATH = sys.argv[1]
FPS = sys.argv[2]
SCALE = sys.argv[3]


def get_calib(base_path: str) -> dict:
    path = {}
    for device in sorted(os.listdir(base_path)):
        device_path = os.path.join(base_path, device)
        for ts in sorted(os.listdir(device_path)):
            calib_path = os.path.join(base_path, device, ts, 'calib')
            for filename in sorted(os.listdir(calib_path)):
                path[device] = os.path.join(calib_path, filename)
    return path


def get_filepaths(base_path: str, sensor: str) -> dict:
    out = {}
    for device in sorted(os.listdir(base_path)):
        out[device] = []
        device_path = os.path.join(base_path, device)
        for ts in sorted(os.listdir(device_path)):
            sensor_path = os.path.join(base_path, device, ts, sensor)
            for filename in sorted(os.listdir(sensor_path)):
                out[device].append(os.path.join(sensor_path, filename))
    return out


def get_filepaths_with_timestamps():
    dev_color_files = get_filepaths(BASE_PATH, 'color')
    dev_depth_files = get_filepaths(BASE_PATH, 'depth')
    dev_calib_file = get_calib(BASE_PATH)

    num_data = len(tuple(dev_color_files.values())[0])
    num_dev = len(list(dev_calib_file.keys()))

    devices = list(dev_calib_file.keys())
    devices = [[i]*num_data for i in devices]
    calib_files = list(dev_calib_file.values())
    calib_files = [[i]*num_data for i in calib_files]

    color_dict = {dev: {} for dev in dev_calib_file.keys()}
    depth_dict = {dev: {} for dev in dev_calib_file.keys()}
    for data_tuple in zip(
        *tuple(devices),
        *tuple(calib_files),
        *tuple(dev_color_files.values()),
        *tuple(dev_depth_files.values()),
    ):
        for i in range(num_dev):
            dev = data_tuple[0*num_dev+i]
            calib_file = data_tuple[1*num_dev+i]
            color_file = data_tuple[2*num_dev+i]
            depth_file = data_tuple[3*num_dev+i]

            color_ts = color_file.split('/')[-1]
            color_ts = int(color_ts.split('.')[0])

            depth_ts = depth_file.split('/')[-1]
            depth_ts = int(depth_ts.split('.')[0])

            color_dict[dev][color_ts] = [calib_file, color_file]
            depth_dict[dev][depth_ts] = [calib_file, depth_file]

    return color_dict, depth_dict, num_data, num_dev


if __name__ == "__main__":

    color_dict, depth_dict, num_data, num_dev = get_filepaths_with_timestamps()
    dev_color_ts = {k: list(v.keys()) for k, v in color_dict.items()}
    dev_depth_ts = {k: list(v.keys()) for k, v in depth_dict.items()}

    dev_list = list(color_dict.keys())

    ts_color_max = max([list(i.keys())[0] for i in color_dict.values()])
    ts_depth_max = max([list(i.keys())[0] for i in depth_dict.values()])
    ts_max = max([ts_color_max, ts_depth_max])

    counter_color = [0] * num_dev
    counter_depth = [0] * num_dev

    try:
        while True:

            tmp_color_ts_list = []
            tmp_depth_ts_list = []
            for idx, dev in enumerate(dev_list):
                ts_idx, ts = next(
                    x
                    for x in enumerate(dev_color_ts[dev][counter_color[idx]:])
                    if x[1] >= ts_max
                )
                tmp_color_ts_list.append(ts)
                ts_idx, ts = next(
                    x
                    for x in enumerate(dev_depth_ts[dev][counter_depth[idx]:])
                    if x[1] >= ts_max
                )
                tmp_depth_ts_list.append(ts)
            ts_max = max(tmp_color_ts_list + tmp_depth_ts_list)

            pcd_list = []
            for i, dev in enumerate(dev_list):
                calib_file = color_dict[dev][tmp_color_ts_list[i]][0]
                color_file = color_dict[dev][tmp_color_ts_list[i]][1]
                depth_file = depth_dict[dev][tmp_depth_ts_list[i]][1]

                with open(calib_file, 'r') as f:
                    calib_data = json.load(f)

                h_c = calib_data['color']['height']
                w_c = calib_data['color']['width']
                h_d = calib_data['depth']['height']
                w_d = calib_data['depth']['width']

                if color_file.endswith('.bin'):
                    with open(color_file, 'rb') as f:
                        img = np.fromfile(f, np.uint8).reshape(h_c, w_c, 3)
                else:
                    img = np.load(color_file).reshape(h_c, w_c, 3)
                # img = np.flip(img, -1)

                if depth_file.endswith('.bin'):
                    with open(depth_file, 'rb') as f:
                        depth = np.fromfile(f, np.uint16)[-h_d*w_d:]
                        depth = depth.reshape(h_d, w_d)
                else:
                    depth = np.fromfile(depth_file, np.uint16)[-h_d*w_d:]
                    depth = depth.reshape(h_d, w_d)

                # rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
                #     color, depth, convert_rgb_to_intensity=False)
                # pcd = o3d.geometry.PointCloud.create_from_rgbd_image(
                #     rgbd, pinhole_camera_intrinsic)
                camera_intrinsic_o3d = o3d.camera.PinholeCameraIntrinsic(
                    width=calib_data['depth']['width'],
                    height=calib_data['depth']['height'],
                    fx=calib_data['depth']['intrinsic_mat'][0],
                    fy=calib_data['depth']['intrinsic_mat'][4],
                    cx=calib_data['depth']['intrinsic_mat'][2],
                    cy=calib_data['depth']['intrinsic_mat'][5]
                )

                pcd = o3d.geometry.PointCloud.create_from_depth_image(
                    depth=o3d.geometry.Image(depth),
                    intrinsic=camera_intrinsic_o3d,
                    # extrinsic=(with default value),
                    depth_scale=1/calib_data['depth']['depth_scale'],
                    depth_trunc=5.0,
                    # stride=1
                )

                # flip the orientation, so it looks upright, not upside-down
                pcd.transform([[1, 0, 0, i*10], [0, -1, 0, 0],
                               [0, 0, -1, 0], [0, 0, 0, 1]])
                pcd_list.append(pcd)

            o3d.visualization.draw_geometries(pcd_list, height=768, width=1280)

    except Exception as e:
        print(e)
        exit(1)

    finally:
        exit(0)
