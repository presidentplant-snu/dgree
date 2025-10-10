import rclpy 
from rclpy.node import Node

from std_msgs.msg import Float32MultiArray

import numpy as np

import serial

class MotorDriverNode(Node):
    def __init__(self):
        super().__init__("motor_driver_node")


