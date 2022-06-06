#!/bin/bash
sudo ip link set can0 down
sudo ip link set can0 up type can bitrate 1000000 dbitrate 4000000 fd on loopback on restart-ms 100
