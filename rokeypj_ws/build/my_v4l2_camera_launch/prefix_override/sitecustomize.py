import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/rokey-jw/rokeypj_ws/install/my_v4l2_camera_launch'
