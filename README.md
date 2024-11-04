# The twr_uwb_driver ROS1 package

This package is reading and parsing lines from a serial port, in order to get the sequence number, ID1, ID2 and raw distance between these UWB devices.
According to a predifined list of anchor and tag IDs the type is determined. 
This information is stored and published as a [uwb_msgs::TwoWayRangeStamped](https://gitlab.aau.at/aau-cns/ros_pkgs/uwb_msgs/-/blob/main/msg/TwoWayRangeStamped.msg) message.


## Dependcies

- [uwb_msgs](https://gitlab.aau.at/aau-cns/ros_pkgs/uwb_msgs)
- [pyserial](https://pypi.org/project/pyserial/) `pip install pyserial`

