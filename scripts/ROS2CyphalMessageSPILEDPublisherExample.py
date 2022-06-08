import rclpy
import numpy as np
from rclpy.node import Node
from canfd_msgs.msg import OpenCyphalMessage
""" 
Conversion of a uORB spi_led message in PX4 to OpenCyphal single frame message. 
Tested on Ubuntu 22.04 for virtual CAN use. Physical CANFD tested on a NavQPlus 
connected over CAN to an NXP UCANS32K146 connected over SPI to APA102C LEDS.
uORB message definition of spi_led:
    uint64 timestamp                # time since system start (microseconds)
    uint16 number_leds              #total number of leds [0,1000]
    uint8 offset_group              #offset group (multiple of 10) of leds to control [0,100]
    uint32[10] led_values           #follows spi 32 bit LED schema for apa102

    # [  PAD 1's  ][ BRIGHT 0:31 ][ BLU 0:255 ][ GRN 0:255 ][ RED 0:255 ]
    # [ 31 ... 29 ][ 28  ...  24 ][ 23 ... 16 ][ 15 ...  8 ][ 7  ...  0 ]
"""

class ROS2CyphalMessageSPILEDPublisherTest(Node):

    def __init__(self):
        super().__init__('ros2_cyphal_message_spi_led_publisher_test')

        self.hz = 30
        self.NumberLeds = 50
        self.LoopCounter = 0
        self.BrightnessUpdatePeriodSec = 3
        self.Timer = self.create_timer(1/self.hz, self.Loop)
        self.InitTime = int(round(self.get_clock().now().nanoseconds/1000.0))
        self.CounterCyphalMsg = 0
        self.PubCyphal = self.create_publisher(OpenCyphalMessage, 'CyphalTransmitFrame', 0)
        self.Brightness = 0
        self.RGBSequenceHex = np.array([0x800000, 0x8B0000, 0xA52A2A, 0xB22222, 0xDC143C, 0xFF0000, 
                                        0xFF6347, 0xFF7F50, 0xCD5C5C, 0xF08080, 0xE9967A, 0xFA8072, 
                                        0xFFA07A, 0xFF4500, 0xFF8C00, 0xFFA500, 0xFFD700, 0xB8860B, 
                                        0xDAA520, 0xEEE8AA, 0xBDB76B, 0xF0E68C, 0x808000, 0xFFFF00, 
                                        0x9ACD32, 0x556B2F, 0x6B8E23, 0x7CFC00, 0x7FFF00, 0xADFF2F, 
                                        0x006400, 0x008000, 0x228B22, 0x00FF00, 0x32CD32, 0x90EE90, 
                                        0x98FB98, 0x8FBC8F, 0x00FA9A, 0x00FF7F, 0x2E8B57, 0x66CDAA, 
                                        0x3CB371, 0x20B2AA, 0x2F4F4F, 0x008080, 0x008B8B, 0x00FFFF, 
                                        0x00FFFF, 0xE0FFFF, 0x00CED1, 0x40E0D0, 0x48D1CC, 0xAFEEEE, 
                                        0x7FFFD4, 0xB0E0E6, 0x5F9EA0, 0x4682B4, 0x6495ED, 0x00BFFF, 
                                        0x1E90FF, 0xADD8E6, 0x87CEEB, 0x87CEFA, 0x191970, 0x000080, 
                                        0x00008B, 0x0000CD, 0x0000FF, 0x4169E1, 0x8A2BE2, 0x4B0082, 
                                        0x483D8B, 0x6A5ACD, 0x7B68EE, 0x9370DB, 0x8B008B, 0x9400D3, 
                                        0x9932CC, 0xBA55D3, 0x800080, 0xD8BFD8, 0xDDA0DD, 0xEE82EE, 
                                        0xFF00FF, 0xDA70D6, 0xC71585, 0xDB7093, 0xFF1493, 0xFF69B4, 
                                        0xFFB6C1, 0xFFC0CB, 0xFAEBD7, 0xF5F5DC, 0xFFE4C4, 0xFFEBCD, 
                                        0xF5DEB3, 0xFFF8DC, 0xFFFACD, 0xFAFAD2, 0xFFFFE0, 0x8B4513, 
                                        0xA0522D, 0xD2691E, 0xCD853F, 0xF4A460, 0xDEB887, 0xD2B48C, 
                                        0xBC8F8F, 0xFFE4B5, 0xFFDEAD, 0xFFDAB9, 0xFFE4E1, 0xFFF0F5, 
                                        0xFAF0E6, 0xFDF5E6, 0xFFEFD5, 0xFFF5EE, 0xF5FFFA, 0x708090, 
                                        0x778899, 0xB0C4DE, 0xE6E6FA, 0xFFFAF0, 0xF0F8FF, 0xF8F8FF, 
                                        0xF0FFF0, 0xFFFFF0, 0xF0FFFF, 0xFFFAFA, 0x000000, 0x696969, 
                                        0x808080, 0xA9A9A9, 0xC0C0C0, 0xD3D3D3, 0xDCDCDC, 0xF5F5F5, 
                                        0xFFFFFF])

    def Loop(self):
        if self.LoopCounter < 100000:
            if self.LoopCounter%int(self.BrightnessUpdatePeriodSec*self.hz) == (int(self.BrightnessUpdatePeriodSec*self.hz)-1):
                self.Brightness = (self.Brightness+1)%32

            if self.NumberLeds > 1001:
                self.NumberLeds = 1001
            elif self.NumberLeds < 1:
                self.NumberLeds = 1
            ColorLoop = self.LoopCounter%len(self.RGBSequenceHex)

            AllLedValArray=np.hstack((self.RGBSequenceHex[ColorLoop:],self.RGBSequenceHex[:ColorLoop]))[:self.NumberLeds]

            

            for OffsetGroup in range(int(np.ceil(self.NumberLeds/10.0))):
                LedValArray=AllLedValArray[OffsetGroup*10:np.min([(OffsetGroup+1)*10,self.NumberLeds])]
                while len(LedValArray) < 10:
                    np.append(LedValArray, [np.uint32(0)], axis=0)

                msg = OpenCyphalMessage()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.priority = int(4)
                msg.is_annonymous = False
                msg.subject_id = int(501)
                msg.source_node_id = int(96)
                msg.data = self.ConvertDataSPILED(OffsetGroup, self.NumberLeds, self.Brightness, LedValArray)
                msg.crc= int(224+(self.CounterCyphalMsg%32))
                self.PubCyphal.publish(msg)
                self.CounterCyphalMsg += 1
            self.LoopCounter += 1
            


    def ConvertDataSPILED(self, OffsetGroup, NumberLeds, Brightness, LedValArray):
        DataArray=np.array([], dtype=np.uint8)

        TimeSinceInit = int(round(self.get_clock().now().nanoseconds/1000.0))-self.InitTime
        for i in range(8):
            DataArray = np.append(DataArray,
                    [np.uint8((TimeSinceInit >> i*8) & 255)], 
                    axis=0)
        if len(LedValArray) <= 10:
            for LedVal in LedValArray:
                DataArray = np.append(DataArray, [
                            np.uint8((LedVal >> 0xFFFF) & 0xFF), #RED
                            np.uint8((LedVal >> 0xFF) & 0xFF), #GREEN
                            np.uint8(LedVal & 0xFF), #BLUE
                            np.uint8((Brightness & 0x1F) + 0xE0)
                            ], axis=0)
        else:
            print("LedValArray too large, max is 10")
        
        DataArray = np.append(DataArray,
                            [np.uint8(NumberLeds & 255),
                             np.uint8(NumberLeds >> 8)], axis=0)
        DataArray = np.append(DataArray,[np.uint8(OffsetGroup & 255)], axis=0)
        
        while len(DataArray) < 63:
            DataArray = np.append(DataArray, 
                               [np.uint8(0)], axis=0)
        return DataArray
                
            
if __name__ == '__main__':
    rclpy.init()
    R2CMSLPT = ROS2CyphalMessageSPILEDPublisherTest()
    rclpy.spin(R2CMSLPT)
    R2CMSLPT.destroy_node()
    rclpy.shutdown()