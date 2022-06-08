#!/usr/bin/python3
import rclpy
import time
import can
import numpy as np
from rclpy.node import Node
from canfd_msgs.msg import OpenCyphalMessage
import argparse

class ROS2CyphalMessageToSocketCAN(Node):

    # Only tested on Ubuntu 22.04, requires python3-can
    # First run the helper shell script ./vcan0_fd.sh to setup your virtual CAN

    def __init__(self):
        super().__init__('ros2_cyphal_message_to_socketcan')
        parser = argparse.ArgumentParser()
        parser.add_argument('--channel', default='can0', help="CAN channel to use, example 'can0'.")
        parser.add_argument('--bitrate', default=1000000, help="SocketCAN bitrate")
        args = parser.parse_args()
        self.bus = can.Bus(channel=str(args.channel), interface='socketcan', fd=True)
        self.SubCyphal = self.create_subscription(OpenCyphalMessage, 'CyphalTransmitFrame', self.TransmitROS2CyphalMessageToSocketCAN, 10)

    def TransmitROS2CyphalMessageToSocketCAN(self, ReceivedOpenCyphalMessage):
        Priority = int(ReceivedOpenCyphalMessage.priority)
        IsAnnonymous = int(ReceivedOpenCyphalMessage.is_annonymous)
        SubjectID = int(ReceivedOpenCyphalMessage.subject_id)
        SourceNodeID = int(ReceivedOpenCyphalMessage.source_node_id)
        if Priority > 7:
            Priority = 7
        elif Priority < 0:
            Priority = 0
        if IsAnnonymous > 1:
            IsAnnonymous = 1
        elif IsAnnonymous < 0:
            IsAnnonymous = 0
        if SubjectID > 8191:
            SubjectID = 8191
        elif SubjectID < 0:
            SubjectID = 0
        if SourceNodeID > 127:
            SourceNodeID = 127
        elif SourceNodeID < 0:
            SourceNodeID = 0
        
        ArbitrationID = (Priority << 26) + (IsAnnonymous << 24) + (3 << 21) + (SubjectID << 8) + SourceNodeID
        SocketCANData = ReceivedOpenCyphalMessage.data
        SocketCANData = np.append(SocketCANData,[ReceivedOpenCyphalMessage.crc], axis=0)
        
        SocketCANMessage = can.Message(arbitration_id=ArbitrationID, data=SocketCANData.tolist(), is_fd=True)
        try:
            self.bus.send(SocketCANMessage)
            print(f"Message sent on {bus.channel_info}")
        except can.CanError:
            print("Message NOT sent")


if __name__ == '__main__':
    rclpy.init()
    R2CMTSC = ROS2CyphalMessageToSocketCAN()
    rclpy.spin(R2CMTSC)
    R2CMTSC.destroy_node()
    rclpy.shutdown()
