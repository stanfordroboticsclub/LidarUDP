#!/usr/bin/env python3

import time
from UDPComms import Publisher

from rplidar import RPLidar


fields = "t " *240
format_ = "240f"
port = 8110

pub = Publisher(fields, format_, port)

lidar = RPLidar('/dev/ttyUSB0')


for scan in lidar.iter_scans():
    scan[1]

