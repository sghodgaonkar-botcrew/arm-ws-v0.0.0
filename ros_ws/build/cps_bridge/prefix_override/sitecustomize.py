import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/shanto/Documents/arm-ws-v0.0.0/ros_ws/install/cps_bridge'
