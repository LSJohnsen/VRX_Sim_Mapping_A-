#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32
from sensor_msgs.msg import NavSatFix, Imu, PointCloud2
from sensor_msgs.point_cloud2 import read_points
from PID_controller import PIDController
import math

class WamvControlNode:
    def __init__(self):
        rospy.init_node('wamv_control')
        
        # PID controllers for surge and yaw
        self.surge_pid = PIDController(kp=1.0, ki=0.1, kd=0.05)
        self.yaw_pid = PIDController(kp=1.0, ki=0.1, kd=0.05)
        
        # Publishers for thruster control
        self.left_thrust_pub = rospy.Publisher('/wamv/thrusters/left_thrust_cmd', Float32, queue_size=10)
        self.right_thrust_pub = rospy.Publisher('/wamv/thrusters/right_thrust_cmd', Float32, queue_size=10)
        
        # Subscribers for sensor data
        rospy.Subscriber('/wamv/sensors/gps/gps/fix', NavSatFix, self.gps_callback)
        rospy.Subscriber('/wamv/sensors/imu/imu', Imu, self.imu_callback)
        rospy.Subscriber('/wamv/sensors/lidars/sensorname/points', PointCloud2, self.lidar_callback)
        
        self.target_position = (0, 0)  # Replace with actual target coordinates
        self.current_position = (0, 0)
        self.current_yaw = 0.0

        self.control_loop()

    def gps_callback(self, msg):
        # Update the current position from GPS data
        self.current_position = (msg.latitude, msg.longitude)

    def imu_callback(self, msg):
        # Update the current yaw from IMU data
        orientation = msg.orientation
        self.current_yaw = self.quaternion_to_yaw(orientation)

    def lidar_callback(self, msg):
        # Process LiDAR data for obstacle avoidance
        for p in read_points(msg, skip_nans=True):
            x, y, z = p[:3]
            distance = math.sqrt(x**2 + y**2)
            if distance < 5.0:
                rospy.logwarn("Obstacle detected within 5 meters!")

    def quaternion_to_yaw(self, orientation):
        # Converts quaternion to yaw angle
        siny_cosp = 2 * (orientation.w * orientation.z + orientation.x * orientation.y)
        cosy_cosp = 1 - 2 * (orientation.y ** 2 + orientation.z ** 2)
        return math.atan2(siny_cosp, cosy_cosp)

    def calculate_distance_to_target(self):
        # Calculate Euclidean distance to target
        lat, lon = self.current_position
        target_lat, target_lon = self.target_position
        return math.sqrt((target_lat - lat) ** 2 + (target_lon - lon) ** 2)

    def control_loop(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            distance_to_target = self.calculate_distance_to_target()
            surge_command = self.surge_pid.calculate_control(
                setpoint=1.0,  # desired distance
                measured_value=distance_to_target
            )

            yaw_command = self.yaw_pid.calculate_control(
                setpoint=math.atan2(self.target_position[1] - self.current_position[1],
                                    self.target_position[0] - self.current_position[0]),
                measured_value=self.current_yaw
            )

            self.publish_control_commands(surge_command, yaw_command)
            rate.sleep()

    def publish_control_commands(self, surge_cmd, yaw_cmd):
        left_cmd = Float32()
        right_cmd = Float32()

        left_cmd.data = surge_cmd - yaw_cmd
        right_cmd.data = surge_cmd + yaw_cmd

        self.left_thrust_pub.publish(left_cmd)
        self.right_thrust_pub.publish(right_cmd)

if __name__ == '__main__':
    try:
        WamvControlNode()
    except rospy.ROSInterruptException:
        pass
