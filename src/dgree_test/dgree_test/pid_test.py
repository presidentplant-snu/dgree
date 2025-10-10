#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import threading
import time


class PIDTester(Node):
    """
    Test node for PID controller that publishes test setpoints and current values,
    monitors control outputs, and visualizes the results in real-time.
    """
    def __init__(self):
        super().__init__('pid_tester_node')
        
        # Test parameters
        self.test_duration = 20.0  # seconds
        self.publish_rate = 100.0  # Hz
        self.num_joints = 2
        
        # Data storage for plotting
        self.max_samples = int(self.test_duration * self.publish_rate)
        self.time_data = []
        self.setpoint_data = [[] for _ in range(self.num_joints)]
        self.current_data = [[] for _ in range(self.num_joints)]
        self.output_data = [[] for _ in range(self.num_joints)]
        self.error_data = [[] for _ in range(self.num_joints)]
        
        # Simulation state
        self.current_angles = np.zeros(self.num_joints, dtype=np.float32)
        self.current_velocities = np.zeros(self.num_joints, dtype=np.float32)
        self.start_time = None
        
        # Plant dynamics (simple mass-spring-damper system)
        self.mass = 1.0
        self.damping = 2.0
        self.dt = 1.0 / self.publish_rate
        
        # Publishers
        self.setpoint_pub = self.create_publisher(
            Float32MultiArray,
            '/joints/target_angles',
            10
        )
        
        self.current_pub = self.create_publisher(
            Float32MultiArray,
            '/markers/current_angles',
            10
        )
        
        # Subscriber to motor output
        self.motor_sub = self.create_subscription(
            Float32MultiArray,
            '/motor/output',
            self.motor_callback,
            10
        )
        
        # Timer for publishing test data
        self.timer = self.create_timer(self.dt, self.test_loop)
        
        self.get_logger().info('PID Tester Node initialized')
        self.get_logger().info(f'Test duration: {self.test_duration}s')
        
    def generate_setpoint(self, t):
        """
        Generate test setpoint trajectories
        Different patterns for each joint to test controller performance
        """
        setpoint = np.zeros(self.num_joints)
        
        # Joint 0: Step response + sinusoid
        if t < 3.0:
            setpoint[0] = 0.0
        elif t < 8.0:
            setpoint[0] = 1.0  # Step input
        else:
            setpoint[0] = 1.0 + 0.5 * np.sin(2 * np.pi * 0.3 * (t - 8.0))
        
        # Joint 1: Ramp + square wave
        if t < 5.0:
            setpoint[1] = 0.2 * t  # Ramp
        elif t < 12.0:
            setpoint[1] = 1.0
        else:
            # Square wave
            setpoint[1] = 1.0 if (int((t - 12.0) / 2.0) % 2 == 0) else -0.5
            
        return setpoint.astype(np.float32)
    
    def simulate_plant(self, control_input, dt):
        """
        Simulate a simple plant (mass-damper system) responding to control input
        This acts as the "real system" being controlled
        
        Equation: m*a = F - damping*v
        where F is the control input (force/torque)
        """
        for i in range(self.num_joints):
            # Calculate acceleration
            acceleration = (control_input[i] - self.damping * self.current_velocities[i]) / self.mass
            
            # Update velocity and position (Euler integration)
            self.current_velocities[i] += acceleration * dt
            self.current_angles[i] += self.current_velocities[i] * dt
    
    def motor_callback(self, msg):
        """Receive motor control output from PID controller"""
        control_output = np.array(msg.data)
        
        # Simulate plant dynamics
        self.simulate_plant(control_output, self.dt)
        
    def test_loop(self):
        """Main test loop - publish setpoints and current values"""
        if self.start_time is None:
            self.start_time = time.time()
            
        current_time = time.time() - self.start_time
        
        # Check if test is complete
        if current_time > self.test_duration:
            self.get_logger().info('Test complete! Generating plots...')
            self.timer.cancel()
            self.generate_plots()
            return
        
        # Generate and publish setpoint
        setpoint = self.generate_setpoint(current_time)
        setpoint_msg = Float32MultiArray()
        setpoint_msg.data = setpoint.tolist()
        self.setpoint_pub.publish(setpoint_msg)
        
        # Publish current state
        current_msg = Float32MultiArray()
        current_msg.data = self.current_angles.tolist()
        self.current_pub.publish(current_msg)
        
        # Store data for plotting
        self.time_data.append(current_time)
        for i in range(self.num_joints):
            self.setpoint_data[i].append(setpoint[i])
            self.current_data[i].append(self.current_angles[i])
            error = setpoint[i] - self.current_angles[i]
            self.error_data[i].append(error)
        
        # Log progress
        if len(self.time_data) % 100 == 0:
            self.get_logger().info(
                f'Time: {current_time:.2f}s | '
                f'SP: [{setpoint[0]:.3f}, {setpoint[1]:.3f}] | '
                f'Curr: [{self.current_angles[0]:.3f}, {self.current_angles[1]:.3f}]'
            )
    
    def generate_plots(self):
        """Generate comprehensive plots of the PID controller performance"""
        time_array = np.array(self.time_data)
        
        # Create figure with subplots
        fig, axes = plt.subplots(3, 2, figsize=(14, 10))
        fig.suptitle('PID Controller Performance Test Results', fontsize=16, fontweight='bold')
        
        joint_names = ['Joint 0', 'Joint 1']
        colors_sp = ['#1f77b4', '#ff7f0e']
        colors_curr = ['#2ca02c', '#d62728']
        
        for i in range(self.num_joints):
            # Position tracking plot
            ax1 = axes[0, i]
            ax1.plot(time_array, self.setpoint_data[i], 
                    label='Setpoint', color=colors_sp[i], linewidth=2)
            ax1.plot(time_array, self.current_data[i], 
                    label='Current', color=colors_curr[i], linewidth=2, alpha=0.8)
            ax1.set_xlabel('Time (s)')
            ax1.set_ylabel('Angle (rad)')
            ax1.set_title(f'{joint_names[i]} - Position Tracking')
            ax1.legend()
            ax1.grid(True, alpha=0.3)
            
            # Error plot
            ax2 = axes[1, i]
            ax2.plot(time_array, self.error_data[i], 
                    color='red', linewidth=1.5)
            ax2.axhline(y=0, color='black', linestyle='--', linewidth=1)
            ax2.set_xlabel('Time (s)')
            ax2.set_ylabel('Error (rad)')
            ax2.set_title(f'{joint_names[i]} - Tracking Error')
            ax2.grid(True, alpha=0.3)
            
            # Performance metrics
            ax3 = axes[2, i]
            errors = np.array(self.error_data[i])
            
            # Calculate metrics
            mae = np.mean(np.abs(errors))
            rmse = np.sqrt(np.mean(errors**2))
            max_error = np.max(np.abs(errors))
            steady_state_error = np.mean(np.abs(errors[-100:])) if len(errors) > 100 else mae
            
            # Find settling time (when error stays within 5% of final value)
            final_setpoint = self.setpoint_data[i][-1]
            settling_threshold = 0.05 * abs(final_setpoint) if final_setpoint != 0 else 0.05
            settling_idx = None
            for j in range(len(errors) - 50, 0, -1):
                if abs(errors[j]) > settling_threshold:
                    settling_idx = j
                    break
            settling_time = time_array[settling_idx] if settling_idx else time_array[0]
            
            metrics_text = (
                f'Performance Metrics:\n\n'
                f'MAE: {mae:.4f} rad\n'
                f'RMSE: {rmse:.4f} rad\n'
                f'Max Error: {max_error:.4f} rad\n'
                f'Steady-State Error: {steady_state_error:.4f} rad\n'
                f'Settling Time (5%): {settling_time:.2f} s'
            )
            
            ax3.text(0.1, 0.5, metrics_text, transform=ax3.transAxes,
                    fontsize=11, verticalalignment='center',
                    bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
            ax3.axis('off')
            ax3.set_title(f'{joint_names[i]} - Metrics')
        
        plt.tight_layout()
        
        plt.show()
        
        # Print summary statistics
        self.get_logger().info('=== Test Summary ===')
        for i in range(self.num_joints):
            errors = np.array(self.error_data[i])
            mae = np.mean(np.abs(errors))
            rmse = np.sqrt(np.mean(errors**2))
            self.get_logger().info(
                f'{joint_names[i]}: MAE={mae:.4f}, RMSE={rmse:.4f}'
            )


def main(args=None):
    rclpy.init(args=args)
    
    tester_node = PIDTester()
    
    try:
        rclpy.spin(tester_node)
    except KeyboardInterrupt:
        pass
    finally:
        tester_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
