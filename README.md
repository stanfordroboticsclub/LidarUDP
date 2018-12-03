# LidarUDP
Publishes [UDP packets](https://github.com/stanfordroboticsclub/UDPComms) on port `8110` with data from an RPLidar on `/dev/ttyUSB1`


Install
-------

`bash install.sh`

Usage
-------

| Command | Descripion |
|---------|------------|
| `sudo systemctl status lidar` | tell us what the service is doing right now|
|`sudo systemctl start lidar` | start the service right now|
|`sudo systemctl stop lidar` | stop the service right now|
|`sudo systemctl disable lidar` | stop the service from starting on boot|
|`sudo systemctl enable lidar` | make the service start on boot|
|`journalctl -u lidar` | display the output of the script |
