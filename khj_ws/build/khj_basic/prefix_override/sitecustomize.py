import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/khjoo/2026_manipulator/khj_ws/install/khj_basic'
