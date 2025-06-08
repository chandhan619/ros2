import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/chandhan/Documents/ros2/py_ws/install/py_pkg'
