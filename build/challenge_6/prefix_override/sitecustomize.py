import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/sofia/ros2_ws/src/ch_6/birria_team/install/challenge_6'
