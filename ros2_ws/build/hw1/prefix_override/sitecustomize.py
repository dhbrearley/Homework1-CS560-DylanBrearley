import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/dylan/ros/Homework1-CS560-DylanBrearley/ros2_ws/install/hw1'
