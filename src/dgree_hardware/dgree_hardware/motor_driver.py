import rclpy 
from rclpy.node import Node

from std_msgs.msg import Float32MultiArray

import numpy as np

import serial



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
                baudrate = 9600
            )
    
    def f_callback(self,msg):
        self.f = np.array(msg.data)
        integer_array = np.array([int(255*np.clip(i,-1.0,1.0)) for i in self.f])
        
        self.py_serial.write(f"{integer_array[0]}, {integer_array[1]}\n".encode())

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


