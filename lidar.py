#!/usr/bin/env python3

import time
import signal
from UDPComms import Publisher

import msgpack
#import json
from rplidar import RPLidar

pub = Publisher("data",  "4096s", 8110)
lidar = RPLidar('/dev/ttyUSB0')

def signal_term_handler(signal, frame):
    lidar.stop()
    lidar.stop_motor()
    exit()
 
signal.signal(signal.SIGTERM, signal_term_handler)

try:
    for scan in lidar.iter_scans():
        #print(scan)
        #print(msgpack.packb(scan))
        #print()
        pub.send(msgpack.packb(scan))
except:
    pass
finally:
    signal_term_handler(None,None)

