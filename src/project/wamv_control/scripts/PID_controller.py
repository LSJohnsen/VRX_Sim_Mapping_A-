#!/usr/bin/env python3
import rospy
import math

class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.previous_error = 0
        self.integral = 0
        self.previous_time = rospy.Time.now().to_sec()

        self.integrator_limits = [0, 10]  # Limits the integrator in both regulators

    def calculate_control(self, setpoint, measured_value):
        current_time = rospy.Time.now().to_sec()
        sample_time = current_time - self.previous_time
        self.previous_time = current_time

        error = setpoint - measured_value
        self.integral += error * sample_time if sample_time > 0 else 0
        self.integral = max(min(self.integral, self.integrator_limits[1]), self.integrator_limits[0])

        derivative = (error - self.previous_error) / sample_time if sample_time > 0 else 0
        self.previous_error = error

        output = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
        return output




