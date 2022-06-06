#!/bin/bash

sudo modprobe vcan

sudo ip link add dev vcan0 type vcan

sudo ip link set vcan0 mtu 72

sudo ip link set dev vcan0 up