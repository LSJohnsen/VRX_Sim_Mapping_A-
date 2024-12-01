#!/usr/bin/env python3

import rospy 
import numpy as np


from sensor_msgs.msg import PointCloud2 
from sensor_msgs.point_cloud2 import read_points 

from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid 

from std_msgs.msg import Header
from geometry_msgs.msg import TransformStamped 

import tf2_ros
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud

class MapGenerator:
    def __init__(self):

        """
        Subscribes to lidar and odom by odometry_publisher.py

        transformbroadcaster will broadcast the transofrmations between the inertial (map) and body frame (base_link)
        with listener listening to tf data and buffer storing it.

        Initialize occupancygrid as 300x300 map with usv origin position in the middle with the frame id "map"
        
        """


        rospy.init_node('map_generator')

        self.map_pub = rospy.Publisher('/grid_map', OccupancyGrid, queue_size=10)

        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        rospy.Subscriber('/wamv/sensors/lidars/lidar_wamv_vrx/points', PointCloud2, self.lidar_callback)

   
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()      
        self.transform_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(30)) 
        self.transform_listener = tf2_ros.TransformListener(self.transform_buffer)

        self.map_resolution = 1.0  
        self.map_size = (300, 300) 
        self.map_data = np.zeros(self.map_size, dtype=int) 

        self.origin_x = -self.map_size[0] / 2 * self.map_resolution
        self.origin_y = -self.map_size[1] / 2 * self.map_resolution
      
        self.grid = OccupancyGrid() #Create map
        self.grid.header = Header() #Timestamp in frame std_msgs/msg/Header.msg
        self.grid.header.frame_id = "map" #point of frame to timestamp
        self.grid.info.resolution = self.map_resolution 
        self.grid.info.width = self.map_size[0]
        self.grid.info.height = self.map_size[1]
        self.grid.info.origin.position.x = self.origin_x
        self.grid.info.origin.position.y = self.origin_y


    def odom_callback(self, msg):

        """
        Calls the odom data from other script and updates the base_link position and heading in the map inertial frame
        stamp => when msg is created to align 
        """
     
        map_base_transform = TransformStamped()
        map_base_transform.header.stamp = rospy.Time.now()
        map_base_transform.header.frame_id = "map"
        map_base_transform.child_frame_id = "wamv/base_link"

      
        map_base_transform.transform.translation.x = msg.pose.pose.position.x
        map_base_transform.transform.translation.y = msg.pose.pose.position.y
        map_base_transform.transform.translation.z = 0

      
        map_base_transform.transform.rotation = msg.pose.pose.orientation

        self.tf_broadcaster.sendTransform(map_base_transform)



    def lidar_callback(self, msg):

        """
        Transforms between the lidar from the base_link (body) reading to the map (inertial) frame
            lookup_transform determines the transform between two frames 
            do_transform_cloud completes a transformation between frame with the lidar pointcloud

        Filter points by distance and height to prevent distant water and usv self marking

        Set marked grids to 100
        """

       
        sensor_map_transform = self.transform_buffer.lookup_transform("map", msg.header.frame_id, msg.header.stamp, rospy.Duration(1.0))
        transformed_cloud_map = do_transform_cloud(msg, sensor_map_transform)

    
        for point in read_points(transformed_cloud_map, field_names=("x", "y", "z"), skip_nans=True):

            distance = np.sqrt((point[0] ** 2 + point[1] ** 2 + point[2] ** 2)) 
            if 0.5 < distance and abs(point[2]) < 1: 
                map_x, map_y = self.point_to_map_coords(point[0], point[1])
                if self.cords_in_map(map_x, map_y):
                    self.map_data[map_y, map_x] = 100 

    def point_to_map_coords(self, x, y):

        """
        Calculate map_x and map_y position relative to the map center
        """

        map_x = int((x / self.map_resolution) + (self.map_size[0] // 2))
        map_y = int((y / self.map_resolution) + (self.map_size[1] // 2))
        return map_x, map_y

    def cords_in_map(self, x, y):
        
        
        return 0 <= x < self.map_size[0] and 0 <= y < self.map_size[1]

    
    def publish_map(self):

        """
        Flatten map to single array to work with occupancygrid before publishing
        """

       
        self.grid.data = self.map_data.flatten().tolist()
        self.grid.header.stamp = rospy.Time.now()
        self.map_pub.publish(self.grid)

    def map_to_odom_transform(self):

        """
        Set a static transform between the odometry frame and the map frame. 
        The map and base_link are offset by 1rad. 

        Could possibyl be done directly in odometry_publisher.py by offseting yaw by -1 (not tested). 
        """


        map_odom_transform = TransformStamped()
        map_odom_transform.header.stamp = rospy.Time.now()
        map_odom_transform.header.frame_id = "map"
        map_odom_transform.child_frame_id = "odom"
        map_odom_transform.transform.translation.x = 0.0
        map_odom_transform.transform.translation.y = 0.0
        map_odom_transform.transform.translation.z = 0.0
        map_odom_transform.transform.rotation.x = 0.0
        map_odom_transform.transform.rotation.y = 0.0
        map_odom_transform.transform.rotation.z = 0.0
        map_odom_transform.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(map_odom_transform)


if __name__ == "__main__":
    rospy.init_node('map_generator')
    map_generator = MapGenerator()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
            map_generator.map_to_odom_transform()
            map_generator.publish_map()
            rate.sleep()
