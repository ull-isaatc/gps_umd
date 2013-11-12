#!/bin/sh

GPS_PORT="/dev/ttyMI4"

#echo "%dM%dm" > $GPS_PORT
#sleep 1
#echo "%dM%dm" > $GPS_PORT
#sleep 1
##echo "%em%em,,nmea/{GGA:0.2,GSA,GST,VTG}:1"  > $GPS_PORT
#echo "%em%em,,nmea/{GGA,GST}:0.2"  > $GPS_PORT

gpsd -n -N -D 0 $GPS_PORT
