# Taken from : librealsense/wrappers/python/examples/net_viewer.py

# License: Apache 2.0. See LICENSE file in root directory.
# Copyright(c) 2021 Intel Corporation. All Rights Reserved.

###############################################
#        Network viewer                      ##
###############################################

import argparse
import cv2
import numpy as np
import os
import time

try:
    from pyrealsense2 import pyrealsense2 as rs
except ImportError as e:
    print('Error: pyrealsense2 could not be found.')
    raise e

try:
    from pyrealsense2 import pyrealsense2_net as rsnet
except ImportError as e:
    print('Error: pyrealsense2_net could not be found.')
    raise e


def str2bool(v):
    if v.lower() in ('yes', 'true', 't', 'y', '1'):
        return True
    elif v.lower() in ('no', 'false', 'f', 'n', '0'):
        return False
    else:
        raise argparse.ArgumentTypeError('Boolean value expected.')


def get_parser():
    parser = argparse.ArgumentParser(
        description='Run RealSense devices over network.')
    parser.add_argument('--rsnet-ip',
                        type=str,
                        # default='192.168.100.39',  # 101 LAN
                        default='192.168.1.216',  # 101 WLAN
                        # default='192.168.1.11',  # 102 WLAN
                        help='ip address')
    parser.add_argument('--rsnet-fps',
                        type=int,
                        default=6,
                        help='fps')
    parser.add_argument('--rsnet-image-width',
                        type=int,
                        default=640,
                        help='image width in px')
    parser.add_argument('--rsnet-image-height',
                        type=int,
                        default=480,
                        help='image height in px')
    parser.add_argument('--rsnet-stream-color',
                        type=str2bool,
                        default=True,
                        help='enable color stream')
    parser.add_argument('--rsnet-stream-depth',
                        type=str2bool,
                        default=False,
                        help='enable depth stream')
    parser.add_argument('--rsnet-save-path',
                        type=str,
                        default='/code/realsense-wrapper-python/output',
                        help='enable depth stream')
    return parser


if __name__ == "__main__":

    arg = get_parser().parse_args()

    ctx = rs.context()

    print(f'[INFO] : Connecting to {arg.rsnet_ip}')
    dev = rsnet.net_device(arg.rsnet_ip)
    print(f'[INFO] : Connected')

    ci = rs.camera_info
    print(f'[INFO] : Using device 0 : {dev.get_info(ci.name)}')
    print(f'[INFO] : Serial number  : {dev.get_info(ci.serial_number)}')

    dev.add_to(ctx)
    pipeline = rs.pipeline(ctx)

    # Start streaming
    print(f'[INFO] : Initialize stream')
    cfg = rs.config()
    if arg.rsnet_stream_depth:
        cfg.enable_stream(stream_type=rs.stream.depth,
                          format=rs.format.z16,
                          framerate=arg.rsnet_fps,
                          width=arg.rsnet_image_width,
                          height=arg.rsnet_image_height)
    if arg.rsnet_stream_color:
        cfg.enable_stream(stream_type=rs.stream.color,
                          format=rs.format.rgb8,
                          framerate=arg.rsnet_fps,
                          width=arg.rsnet_image_width,
                          height=arg.rsnet_image_height)
    check = cfg.can_resolve(pipeline)
    print(f"[INFO] : 'cfg' defined can be used by 'pipeline' : {check}")
    pipeline.start(cfg)
    time.sleep(10)

    print(f"[INFO] : Generate 'save_dir'")
    save_dir = f'{arg.rsnet_save_path}/{time.time()}'
    os.makedirs(save_dir, exist_ok=True)

    print(f"[INFO] : Starting frame capture loop...")
    try:
        c = 0
        while True:

            # Wait for a coherent pair of frames: depth and color
            frames = pipeline.wait_for_frames()
            if arg.rsnet_stream_color:
                color_frame = frames.get_color_frame()
                if not color_frame:
                    print("[WARN] : Empty color frame")
                    continue
            if arg.rsnet_stream_depth:
                depth_frame = frames.get_depth_frame()
                if not depth_frame:
                    print("[WARN] : Empty depth frame")
                    continue

            print("[INFO] : Running...")

            # Convert images to numpy arrays
            if arg.rsnet_stream_color:
                color_image = np.asanyarray(color_frame.get_data())
                color_colormap_dim = color_image.shape

            if arg.rsnet_stream_depth:
                depth_image = np.asanyarray(depth_frame.get_data())
                # Apply colormap on depth image
                # (image must be converted to 8-bit per pixel first)
                depth_colormap = cv2.applyColorMap(
                    cv2.convertScaleAbs(depth_image, alpha=0.03),
                    cv2.COLORMAP_JET
                )
                depth_colormap_dim = depth_colormap.shape

            if arg.rsnet_stream_color and arg.rsnet_stream_depth:
                # If depth and color resolutions are different,
                # resize color image to match depth image for display
                if depth_colormap_dim != color_colormap_dim:
                    resized_color_image = cv2.resize(
                        color_image,
                        dsize=(depth_colormap_dim[1],
                               depth_colormap_dim[0]),
                        interpolation=cv2.INTER_AREA
                    )
                    image = np.hstack(
                        (resized_color_image, depth_colormap))
                else:
                    image = np.hstack((color_image, depth_colormap))

            elif arg.rsnet_stream_color:
                image = color_image

            elif arg.rsnet_stream_depth:
                image = depth_colormap

            timestamp = frames.get_frame_metadata(
                rs.frame_metadata_value.sensor_timestamp)

            image_name = f'{save_dir}/{timestamp}.jpg'
            cv2.imwrite(image_name, image)
            print(f"Saved : {image_name}")

            # # Show images
            # cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
            # cv2.imshow('RealSense', images)
            # k = cv2.waitKey(1) & 0xFF
            # if k == 27:    # Escape
            #     cv2.destroyAllWindows()
            #     break

            c += 1
            if c > arg.rsnet_fps * 10:
                break

    # except:  # noqa
    #     print("[WARN] : Stopping RealSense devices...")
    #     pipeline.stop()
    finally:
        pipeline.stop()

    print("Finished")
