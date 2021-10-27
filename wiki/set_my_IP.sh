#!/bin/bash

#Find IP Address (on WIFI)
NETWORK_INTERFACE=wlp0s20f3
#on Ethernet
#NETWORK_INTERFACE=eno1

IP=$(ip -f inet addr show $NETWORK_INTERFACE | grep -Po 'inet \K[\d.]+')
export ROS_IP="$IP"
