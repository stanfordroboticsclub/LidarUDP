#!/usr/bin/env python3

import time
import signal
from UDPComms import Publisher

import msgpack
from rplidar import RPLidar

pub = Publisher("json",  "4096s", 8110)
lidar = RPLidar('/dev/ttyUSB1')

def signal_term_handler(signal, frame):
    lidar.stop()
    lidar.stop_motor()
    exit()
 
signal.signal(signal.SIGTERM, signal_term_handler)

for scan in lidar.iter_scans(scan_type='express'):
    pub.send(msgpack.dumps(scan))

