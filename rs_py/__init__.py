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

from .utils import printout
from .utils import str2bool

from .rs_wrapper import get_parser as get_rs_parser
from .rs_wrapper import RealsenseWrapper
from .rs_wrapper import StoragePaths

from .rs_view_raw_data import get_filepaths
