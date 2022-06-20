import cv2
import json
import numpy as np
import os

# Currently the code only shows the data from the newest timestamp folder.

FPS = 5
PATH = '/tmp/testing'


# https://github.com/IntelRealSense/librealsense/issues/4646
def get_brg(data_array):
    # Input: Intel handle to 16-bit YU/YV data
    # Output: BGR8
    UV = np.uint8(data_array >> 8)
    Y = np.uint8((data_array << 8) >> 8)
    YUV = np.zeros((*(data_array.shape), 2), 'uint8')
    YUV[:, :, 0] = Y
    YUV[:, :, 1] = UV
    BGR = cv2.cvtColor(YUV, cv2.COLOR_YUV2BGR_YUYV)
    return BGR


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
    dev_color_files = get_filepaths(PATH, 'color')
    dev_depth_files = get_filepaths(PATH, 'depth')
    dev_calib_file = get_calib(PATH)

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

    dev_color_files = get_filepaths(PATH, 'color')
    dev_depth_files = get_filepaths(PATH, 'depth')
    dev_calib_file = get_calib(PATH)

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

        imgs = []
        for i in range(num_dev):
            dev = data_tuple[0*num_dev+i]
            calib_file = data_tuple[1*num_dev+i]
            color_file = data_tuple[2*num_dev+i]
            depth_file = data_tuple[3*num_dev+i]

            color_ts = color_file.split('/')[-1]
            color_ts = int(color_ts.split('.')[0])

            depth_ts = depth_file.split('/')[-1]
            depth_ts = int(depth_ts.split('.')[0])

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
                img = np.load(color_file)
                if isinstance(img, np.uint8):
                    img = img.reshape(h_c, w_c, 3)
                elif isinstance(img, np.uint16):
                    img = get_brg(img.reshape(h_c, w_c))
                else:
                    raise ValueError("Unknown data type :", img.dtype)
            # img = np.flip(img, -1)

            if depth_file.endswith('.bin'):
                with open(depth_file, 'rb') as f:
                    depth = np.fromfile(f, np.uint16)[-h_d*w_d:]
                    depth = depth.reshape(h_d, w_d)
            else:
                depth = np.fromfile(depth_file, np.uint16)[-h_d*w_d:]
                depth = depth.reshape(h_d, w_d)
            depth = cv2.applyColorMap(
                cv2.convertScaleAbs(depth, alpha=0.03),
                cv2.COLORMAP_JET
            )

            cv2.putText(img, str(color_ts),
                        (10, 30), cv2.FONT_HERSHEY_PLAIN,
                        2, (0, 255, 0), 2, cv2.LINE_AA)
            cv2.putText(img, str(depth_ts),
                        (10, 80), cv2.FONT_HERSHEY_PLAIN,
                        2, (0, 255, 0), 2, cv2.LINE_AA)
            imgs.append(np.vstack([img, depth]))

        output = np.hstack(imgs)
        cv2.imshow('output', output)
        # cv2.waitKey(1000//FPS)
        cv2.waitKey(0)
