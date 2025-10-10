import rclpy
from rclpy.node import Node
import numpy as np
from std_msgs.msg import Float32MultiArray

def inverse_kinematics(goal):
    target_x = goal[0]
    target_y = goal[1]

    length1 = 200
    length2 = 200

    dist_sq = target_x**2 + target_y**2
    dist = np.sqrt(dist_sq)
    
    cos_theta2 = (dist_sq - length1**2 - length2**2) / (2*length1 * length2)
    if cos_theta2 > 1:
        print("no solution")
    theta2 = np.arccos(cos_theta2)
    #if elbow up position then negative

    alpha = np.arctan2(target_y, target_x)
    beta = np.arctan2(length2 * np.sin(theta2), length1 + length2 * np.cos(theta2))
    theta1 = alpha - beta
    #if elbow up postion then alpha + beta

    return theta1, theta2

class InverseKinematicsNode(Node):
    def __init__(self):
        super().__init__('inversekinematics_node')
        
        self.goal = np.zeros(2, dtype=np.float32)

        self.goal_sub = self.create_subscription(
            Float32MultiArray,
            '/goal/end_effector_pose',
            self.goal_callback,
            10
        )

        self.target_pub = self.create_publisher(
            Float32MultiArray,
            '/joint/target_angles',
            10
        )

        self.inverse_kinematics(self.goal)

    def goal_callback(self, msg):
        self.goal = np.array(msg.data)

def main(args=None):
    rclpy.init(args=args)
    node = InverseKinematicsNode()
    rclpy.spin(node)

if __name__=='__main__':
    print(inverse_kinematics(200, 200))
