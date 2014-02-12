#!/bin/sh

GPS_PORT="/dev/ttyMI1"

#echo "%dM%dm" > $GPS_PORT
#sleep 1
#echo "%dM%dm" > $GPS_PORT
#sleep 1
##echo "%em%em,,nmea/{GGA:0.2,GSA,GST,VTG}:1"  > $GPS_PORT
#echo "%em%em,,nmea/{GGA,GST}:0.2"  > $GPS_PORT

stty speed 115200 <$GPS_PORT
~/ROS/ull-isaatc/src/gpsd/gpsd -n -N -D 0 $GPS_PORT
#/media/VerdinoHD1/usr/local/sbin/gpsd -n -N -D 5 $GPS_PORT
#/media/VerdinoHD1/home/verdino/launch_Dani/gpsd-3.5/gpsd-3.5/gpsd -n -N -D 5 $GPS_PORT
