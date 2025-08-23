import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/arpan/unitree2/unitree_sdk2/utlidar_slam/src/install/qos_relay'
