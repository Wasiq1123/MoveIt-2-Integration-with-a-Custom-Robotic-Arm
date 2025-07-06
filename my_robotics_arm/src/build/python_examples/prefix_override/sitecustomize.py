import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/wasiq/testing_model/my_robotics_arm1/src/install/python_examples'
