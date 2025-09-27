import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/maki/Desktop/GitHub/SpiderRos/WS/install/spider_ros'
