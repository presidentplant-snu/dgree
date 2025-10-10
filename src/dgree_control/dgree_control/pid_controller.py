import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32MultiArray

import numpy as np

class PIDController:
    def __init__(self, kp, ki, kd, output_lim=np.inf, anti_windup=np.inf, sample_time=0.01):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_lim = output_lim
        self.anti_windup = anti_windup
        self.sample_time = sample_time

        self.last_error = 0.0
        self.integral = 0.0 

    def update(self, setpoint, current, dt=None):
        error = setpoint - current

        if dt is None:
            dt = self.sample_time

        if dt == 0.0:
            return

        self.integral += error * dt
        self.integral = np.clip(self.ki * self.integral, -self.anti_windup, self.anti_windup) / self.ki
            
        p_term = self.kp * error
        i_term = self.ki * self.integral
        d_term = self.kd * (error - self.last_error) / dt

        self.last_error = error

        output = p_term + i_term + d_term
        output = np.clip(output, -self.output_lim, self.output_lim)

        return output

    def reset(self):
        self.last_error = 0.0 
        self.integral = 0.0 

    def set_gains(self, kp=None, ki=None, kd=None):
        self.kp = self.kp if kp is None else kp
        self.ki = self.ki if ki is None else ki
        self.kd = self.kd if kd is None else kd

class PIDControllerNode(Node):
    def __init__(self):
        super().__init__('pid_controller_node')

        # TODO: Get parameters
        control_rate = 30

        self.pid = PIDController(
                kp = 1.0,
                kd = 0.3,
                ki = 0.2,
                sample_time= 1.0/control_rate
                )

        self.setpoint = np.zeros(2, dtype=np.float32)
        self.current = np.zeros(2, dtype=np.float32)
        self.last_time = self.get_clock().now()

        self.setpoint_sub = self.create_subscription(
                Float32MultiArray,
                '/joints/target_angles',
                self.setpoint_callback,
                10)

        self.current_sub = self.create_subscription(
                Float32MultiArray,
                '/markers/current_angles',
                self.current_callback,
                10)

        self.motor_pub = self.create_publisher(
                Float32MultiArray,
                '/motor/output',
                10)

        self.timer = self.create_timer(1.0/control_rate, self.control_loop)


    def setpoint_callback(self, msg):
        self.setpoint = np.array(msg.data)

    def current_callback(self, msg):
        self.current = np.array(msg.data)

    def control_loop(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time 

        control_output = self.pid.update(self.setpoint, self.current, dt)

        control_msg = Float32MultiArray()
        control_msg.data = control_output 
        self.motor_pub.publish(control_msg)

def main(args=None):
    rclpy.init(args=args)
    node = PIDControllerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

