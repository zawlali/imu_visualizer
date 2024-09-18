#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np
import threading

class IMUVisualizer(Node):
    def __init__(self):
        super().__init__('imu_visualizer')
        self.subscription = self.create_subscription(
            Imu,
            '/imu',
            self.imu_callback,
            10)
        
        self.get_logger().info(f"Subscribed to topic: {self.subscription.topic_name}")
        self.subscription  # prevent unused variable warning

        # Initialize data storage
        self.time_data = []
        self.accel_x_data = []
        self.accel_y_data = []
        self.accel_z_data = []
        self.gyro_x_data = []
        self.gyro_y_data = []
        self.gyro_z_data = []

        # Set up the plot
        self.fig, (self.ax1, self.ax2) = plt.subplots(2, 1, figsize=(10, 10))
        self.lines_accel = [self.ax1.plot([], [], label=label)[0] for label in ['X', 'Y', 'Z']]
        self.lines_gyro = [self.ax2.plot([], [], label=label)[0] for label in ['X', 'Y', 'Z']]

        self.ax1.set_ylabel('Acceleration (m/s^2)')
        self.ax2.set_ylabel('Angular Velocity (rad/s)')
        self.ax2.set_xlabel('Time (s)')

        self.ax1.legend()
        self.ax2.legend()

        self.ax1.set_title('IMU Data Visualization')

        # Set up the animation
        self.anim = FuncAnimation(self.fig, self.update_plot, interval=100, blit=True)

    def imu_callback(self, msg):
        self.get_logger().info("Received IMU data")
        current_time = self.get_clock().now().to_msg().sec + self.get_clock().now().to_msg().nanosec * 1e-9

        self.time_data.append(current_time)
        self.accel_x_data.append(msg.linear_acceleration.x)
        self.accel_y_data.append(msg.linear_acceleration.y)
        self.accel_z_data.append(msg.linear_acceleration.z)
        self.gyro_x_data.append(msg.angular_velocity.x)
        self.gyro_y_data.append(msg.angular_velocity.y)
        self.gyro_z_data.append(msg.angular_velocity.z)

        # Keep only the last 100 data points
        if len(self.time_data) > 100:
            self.time_data = self.time_data[-100:]
            self.accel_x_data = self.accel_x_data[-100:]
            self.accel_y_data = self.accel_y_data[-100:]
            self.accel_z_data = self.accel_z_data[-100:]
            self.gyro_x_data = self.gyro_x_data[-100:]
            self.gyro_y_data = self.gyro_y_data[-100:]
            self.gyro_z_data = self.gyro_z_data[-100:]

    def update_plot(self, frame):
        accel_data = [self.accel_x_data, self.accel_y_data, self.accel_z_data]
        gyro_data = [self.gyro_x_data, self.gyro_y_data, self.gyro_z_data]

        for line, data in zip(self.lines_accel, accel_data):
            line.set_data(self.time_data, data)

        for line, data in zip(self.lines_gyro, gyro_data):
            line.set_data(self.time_data, data)

        self.ax1.relim()
        self.ax1.autoscale_view()
        self.ax2.relim()
        self.ax2.autoscale_view()

        return self.lines_accel + self.lines_gyro

def main(args=None):
    rclpy.init(args=args)

    imu_visualizer = IMUVisualizer()
    imu_visualizer.get_logger().info(f"Available topics: {imu_visualizer.get_topic_names_and_types()}")

    # Create a separate thread for ROS 2 spinning
    spin_thread = threading.Thread(target=rclpy.spin, args=(imu_visualizer,))
    spin_thread.start()

    plt.show()

    # Clean up
    rclpy.shutdown()
    spin_thread.join()

    imu_visualizer.destroy_node()

if __name__ == '__main__':
    main()