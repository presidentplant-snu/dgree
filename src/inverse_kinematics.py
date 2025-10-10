import rclpy
from rclpy.node import Node
import numpy as np

def inverse_kinematics(target_x, target_y):
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
    #cos_beta = (length1**2 + dist_sq - length2**2) / (2*length1*dist)
    #beta = np.arccos(cos_beta)
    theta1 = alpha - beta
    #if elbow up postion then alpha + beta

    return theta1, theta2

if __name__=='__main__':
    inverse_kinematics(200, 200)