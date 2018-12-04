#!/usr/bin/env python3

import time
import signal
from UDPComms import Publisher

import msgpack
import rplidar

pub = Publisher("data",  "4096s", 8110)

use_express = False
possible_ports = ['/dev/ttyUSB0', '/dev/ttyUSB1', '/dev/tty.SLAB_USBtoUART']

for port in possible_ports:
    try:
        lidar = rplidar.RPLidar(port)
        break
    except rplidar.RPLidarException:
        pass
else:
    raise rplidar.RPLidarException("Can't find serial port to connect to sensor")

def signal_term_handler(signal, frame):
    lidar.stop()
    lidar.stop_motor()
    exit()
 
signal.signal(signal.SIGTERM, signal_term_handler)

try:
    for scan in lidar.iter_scans(scan_type = "express" if use_express else "normal"):
        #print(scan)
        pub.send(msgpack.packb(scan))
except:
    pass
finally:
    signal_term_handler(None,None)

