import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/davide-pillon/BFMC/luxonis_ws/install/camera_preprocessing'
