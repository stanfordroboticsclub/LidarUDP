#!/usr/bin/env python3

import time
import signal
from UDPComms import Publisher

# import msgpack
import json
from rplidar import RPLidar

pub = Publisher("data",  "10240s", 8110)
lidar = RPLidar('/dev/ttyUSB1')

def signal_term_handler(signal, frame):
    lidar.stop()
    lidar.stop_motor()
    exit()
 
signal.signal(signal.SIGTERM, signal_term_handler)

for scan in lidar.iter_scans(scan_type='express'):
    pub.send(json.dumps(scan))

