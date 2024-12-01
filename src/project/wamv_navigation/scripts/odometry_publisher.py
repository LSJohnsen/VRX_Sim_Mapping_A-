#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import NavSatFix, Imu
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geodesy.utm import fromLatLong 



class OdometryPublisher:
    def __init__(self):

        """
        subscribe to gps and odom to transform and publish as odom data
        """

        self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)
        rospy.Subscriber('/wamv/sensors/gps/gps/fix', NavSatFix, self.gps_callback)
        rospy.Subscriber('/wamv/sensors/imu/imu/data', Imu, self.imu_callback)



        self.initial_utm = None
        self.north = 0.0
        self.east = 0.0
        self.yaw = 0.0

    
  
    def gps_callback(self, msg):
        
        """
        get GPS coordinates as UTM points
        """
        
        current_utm = fromLatLong(msg.latitude, msg.longitude)

    
        if self.initial_utm is None:
            self.initial_utm = current_utm
            return

  
        self.north = current_utm.northing - self.initial_utm.northing
        self.east = current_utm.easting - self.initial_utm.easting 

        self.publish_odometry()
    

    def imu_callback(self, msg):
        """
        gets rotation around axes:
        
        Gazebo uses quaternion, transforms to radians

        Can negelct roll & pitch 
        """

        current_orientation = msg.orientation
        
        self.roll, self.pitch, self.yaw = euler_from_quaternion([current_orientation.x, current_orientation.y, current_orientation.z, current_orientation.w])



    def publish_odometry(self):
        """
        The odom is published between odom and body frame
        
        The coordinates are set in the ENU frame considerign the WAM-V configuration
        
        yaw is published as a quaternion from the determined yaw
        """
        
        odom = Odometry()
        odom.header.stamp = rospy.Time.now()
       
        odom.header.frame_id = "odom"
        odom.child_frame_id = "wamv/base_link" 
     
        odom.pose.pose.position.x = self.east
        odom.pose.pose.position.y = self.north
        odom.pose.pose.position.z = 0.0
     
        quat = quaternion_from_euler(0, 0, self.yaw)
        odom.pose.pose.orientation.x = quat[0]
        odom.pose.pose.orientation.y = quat[1]
        odom.pose.pose.orientation.z = quat[2]
        odom.pose.pose.orientation.w = quat[3] 



        self.odom_pub.publish(odom)
        rospy.loginfo_throttle(1.0, f"Current position: North = {self.north:.2f}, East = {self.east:.2f}, Yaw = {self.yaw:.2f} radians")
