import json
import numpy as np
import sys

# import matplotlib.pyplot as plt
# from mpl_toolkits import mplot3d
# from mpl_toolkits.mplot3d import Axes3D
# from mpl_toolkits.mplot3d import proj3d

import plotly.graph_objects as go
import plotly.io as pio

import open3d as o3d

from rs_py.rs_view_raw_data import iterate_over_raw_data
from rs_py.rs_view_raw_data import read_color_file
from rs_py.rs_view_raw_data import read_depth_file


PATH = sys.argv[1]
FPS = sys.argv[2]
SCALE = sys.argv[3]
SYNC = sys.argv[4]

# PATH = '/code/realsense-simple-wrapper/output/rs_sample_3dev'
# FPS = 5
# SCALE = 0.75
# SYNC = 0


def data_process_fn(**kwargs):

    color_files = kwargs['color_files']
    depth_files = kwargs['depth_files']
    color_ts_idxs = kwargs['color_ts_idxs']
    depth_ts_idxs = kwargs['depth_ts_idxs']
    sync_ts = kwargs['sync_ts']

    pcd_list = []
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

        x = np.asarray(pcd.points)[::10, 0]
        y = np.asarray(pcd.points)[::10, 1]
        z = np.asarray(pcd.points)[::10, 2]

        if dev_idx == 1:
            pio.renderers.default = 'browser'
            fig = go.Figure(
                data=[
                    go.Scatter3d(
                        x=-x, y=-z, z=-y,
                        mode='markers',
                        marker=dict(
                            size=3,
                            # color='rgba(255, 0, 0, 0.5)',
                            color=-z,
                            colorscale='Jet',
                            # opacity=0.5
                        )
                    )
                ]
            )
            fig.update_layout(
                title={
                    'text': f"{depth_file.split('/')[-1]}, "
                            f"num of points : {z.shape}, "
                            f"yz swapped, x+y+z negated",
                    'y': 0.9,
                    'x': 0.5,
                    'xanchor': 'center',
                    'yanchor': 'top'
                },
                scene=dict(
                    xaxis=dict(range=[-4, 4],),
                    yaxis=dict(range=[0, 6],),
                    zaxis=dict(range=[-2, 2],),
                ),
            )
            fig.show()

        # flip the orientation, so it looks upright, not upside-down
        pcd.transform([[1, 0, 0, dev_idx*10],
                       [0, -1, 0, 0],
                       [0, 0, -1, 0],
                       [0, 0, 0, 1]])
        pcd_list.append(pcd)

    o3d.visualization.draw_geometries(
        pcd_list,
        window_name='PCD',
        width=1280,
        height=768,
        # zoom=0.3,
        # front=np.array([0.25, 0.5, 1], dtype=float),
        # lookat=np.array([15, 0, 0], dtype=float),
        # up=np.array([0, 1, 0], dtype=float)
    )


if __name__ == "__main__":

    iterate_over_raw_data(PATH, SYNC, data_process_fn=data_process_fn)
