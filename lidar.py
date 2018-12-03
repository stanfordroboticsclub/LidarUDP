#!/usr/bin/env python3

import time
import json
import signal
from UDPComms import Publisher

from rplidar import RPLidar

pub = Publisher("json",  "2048s", 8110)
lidar = RPLidar('/dev/ttyUSB0')

def signal_term_handler(signal, frame):
    lidar.stop()
    lidar.stop_motor()
    exit()
 
signal.signal(signal.SIGTERM, signal_term_handler)

for scan in lidar.iter_scans():
    pub.send(bytes(json.dumps(scan), 'utf-8'))

