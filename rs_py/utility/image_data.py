import csv
import cv2
import numpy as np
import json

from typing import Optional


# https://github.com/IntelRealSense/librealsense/issues/4646
def _get_brg_from_yuv(data_array: np.ndarray) -> np.ndarray:
    # Input: Intel handle to 16-bit YU/YV data, 1 dim array
    # Output: BGR8
    UV = np.uint8(data_array >> 8)
    Y = np.uint8((data_array << 8) >> 8)
    YUV = np.zeros((data_array.shape[0], 2), 'uint8')
    YUV[:, 0] = Y
    YUV[:, 1] = UV
    YUV = YUV.reshape(1, -1, 2)
    BGR = cv2.cvtColor(YUV, cv2.COLOR_YUV2BGR_YUYV)
    return BGR


def read_color_file(filename: str,
                    fileformat: Optional[str] = None) -> np.ndarray:
    if filename.endswith('.bin'):
        with open(filename, 'rb') as f:
            if fileformat.lower() in ['bgr8', 'rgb8']:
                image = np.fromfile(f, np.uint8)
            elif fileformat.lower() in ['yuyv']:
                image = np.fromfile(f, np.uint16)
                image = _get_brg_from_yuv(image.reshape(-1))
            else:
                raise ValueError("Unknown data type :", image.dtype)
    else:
        image = np.load(filename)
        if image.dtype == np.dtype(np.uint8):
            pass
        elif image.dtype == np.dtype(np.uint16):
            image = _get_brg_from_yuv(image.reshape(-1))
        else:
            raise ValueError("Unknown data type :", image.dtype)
    return image


def read_depth_file(filename: str) -> np.ndarray:
    if filename.endswith('.bin'):
        with open(filename, 'rb') as f:
            depth = np.fromfile(f, np.uint16)
    else:
        depth = np.fromfile(filename, np.uint16)
    return depth


def read_calib_file(calib_file: str) -> dict:
    calib_data = {}
    if calib_file.endswith('.json'):
        with open(calib_file, 'r') as f:
            calib_data = json.load(f)
    elif calib_file.endswith('.csv'):
        with open(calib_file, 'r') as f:
            csv_reader = csv.reader(f, delimiter=',')
            line_count = 0
            for row in csv_reader:
                if line_count == 0:
                    calib_data['color'] = {
                        'width': int(row[0]),
                        'height': int(row[1]),
                        'ppx': float(row[2]),
                        'ppy': float(row[3]),
                        'fx': float(row[4]),
                        'fy': float(row[5]),
                        'model': row[6],
                        'coeffs': [float(i) for i in row[7:12]],
                        'format': row[12],
                        'fps': int(row[13]),
                    }
                if line_count == 1:
                    calib_data['depth'] = {
                        'width': int(row[0]),
                        'height': int(row[1]),
                        'ppx': float(row[2]),
                        'ppy': float(row[3]),
                        'fx': float(row[4]),
                        'fy': float(row[5]),
                        'model': row[6],
                        'coeffs': [float(i) for i in row[7:12]],
                        'format': row[12],
                        'fps': int(row[13]),
                    }
                if line_count == 2:
                    calib_data['T_color_depth'] = {
                        'rotation': [float(i) for i in row[0:9]],
                        'translation': [float(i) for i in row[9:12]],
                    }
                if line_count == 3:
                    calib_data['depth']['depth_scale'] = float(row[0])
                    calib_data['depth']['depth_baseline'] = float(row[1])
                line_count += 1
    return calib_data


