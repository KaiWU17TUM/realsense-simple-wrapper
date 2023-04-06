import cv2
import numpy as np
import sys

from rs_py.utility import read_calib_file
from rs_py.utility import read_depth_file
from rs_py.utility import read_color_file
from rs_py.utility import iterate_over_raw_data


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

        calib_data = read_calib_file(color_dc.calib_file)

        h_c = calib_data['color']['height']
        w_c = calib_data['color']['width']
        h_d = calib_data['depth']['height']
        w_d = calib_data['depth']['width']

        image = read_color_file(color_dc.file,
                                calib_data['color'].get('format', None))
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
    # cv2.waitKey(0)
    cv2.waitKey(1)


if __name__ == "__main__":

    # PATH = sys.argv[1]
    # SYNC = int(sys.argv[2])
    # FPS = int(sys.argv[3])
    # SCALE = float(sys.argv[4])

    PATH = '/code/realsense-simple-wrapper/data'
    SYNC = 0
    FPS = 5
    SCALE = 0.75

    iterate_over_raw_data(PATH, SYNC, FPS, SCALE,
                          data_process_fn=data_process_fn)
