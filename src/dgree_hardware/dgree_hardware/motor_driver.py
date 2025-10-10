import rclpy 
from rclpy.node import Node

from std_msgs.msg import Float32MultiArray

import numpy as np
import struct

import serial
import time

START = 0xAA
END   = 0x55
TYPE_DATA = 0x01
TYPE_END  = 0xFF

def build_packet(packet_type: int, payload: bytes) -> bytes:
    length = len(payload)
    chk = (packet_type + length + sum(payload)) & 0xFF
    return bytes([START, packet_type, length]) + payload + bytes([chk, END])

class MotorDriverNode(Node):
    def __init__(self):
        super().__init__("motor_driver_node")

        self.subscriber = self.create_subscription(
            Float32MultiArray,
            '/motor/output',
            self.f_callback,
            10
        )
        
        self.py_serial = serial.Serial(
                port = '/dev/ttyACM0',
                baudrate = 9600,
                        timeout = 2
            )
        time.sleep(2)
        
        # self.py_serial.close()
        # self.py_serial.open()
    
    def f_callback(self,msg):
        self.f = 1.0*np.array(msg.data)
        integer_array = np.array([int(154*np.clip(i,-1.0,1.0)+90*np.sign(i))*(np.abs(i)>0.05) for i in self.f])
        self.get_logger().info(f"{integer_array[0]}, {integer_array[1]}\n")
        payload = struct.pack('<hh', int(integer_array[0]), int(integer_array[1]))
        pkt = build_packet(TYPE_DATA, payload)
        self.py_serial.write(pkt)

def main(args=None):
    rclpy.init(args=args)
    node = MotorDriverNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()


