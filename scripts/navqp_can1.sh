#!/bin/bash
sudo ip link set can1 down
sudo ip link set can1 up type can bitrate 1000000 dbitrate 4000000 fd on restart-ms 100
