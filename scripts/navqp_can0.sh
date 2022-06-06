#!/bin/bash
sudo ip link set can0 down
sudo ip link set can0 up type can bitrate 1000000 dbitrate 4000000 loopback off fd on fd-non-iso off restart-ms 100
