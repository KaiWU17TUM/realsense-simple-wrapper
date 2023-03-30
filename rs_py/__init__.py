try:
    from pyrealsense2 import pyrealsense2 as rs
except ImportError as e:
    print('Error: pyrealsense2 library could not be found.')
    raise e
try:
    from pyrealsense2 import pyrealsense2_net as rsnet
except ImportError as e:
    print('Error: pyrealsense2_net could not be found.')
    raise e
