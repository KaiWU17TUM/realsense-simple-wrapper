import cv2
import json
import sys
import os

from rs_py.utility import printout
from rs_py.utility import read_color_file
from rs_py.utility import iterate_over_raw_data


def data_process_fn(**kwargs):

    color_files = kwargs['color_files']
    color_ts_idxs = kwargs['color_ts_idxs']

    num_dev = len(color_files)
    for dev_idx in range(num_dev):
        # (ts, file, calib)
        color_dc = color_files[dev_idx][color_ts_idxs[dev_idx]]

        with open(color_dc.calib_file, 'r') as f:
            calib_data = json.load(f)

        h_c = calib_data['color']['height']
        w_c = calib_data['color']['width']

        image = read_color_file(color_dc.file)
        image = image.reshape(h_c, w_c, 3)

        new_dir = "/".join(color_dc.file.split('/')[:-2]) + "/color_png"
        os.makedirs(new_dir, exist_ok=True)

        new_file = color_dc.file.replace('/color/', '/color_png/')
        new_file = os.path.splitext(new_file)[0] + '.png'
        cv2.imwrite(new_file, image)

    printout("-"*80, 'i')
    printout(calib_data, 'i')


if __name__ == "__main__":

    # PATH = '/code/realsense-simple-wrapper/output/rs_3dev_sample2'

    PATH = sys.argv[1]
    SYNC = 0
    FPS = 0
    SCALE = 0

    iterate_over_raw_data(PATH, SYNC, FPS, SCALE,
                          data_process_fn=data_process_fn)
