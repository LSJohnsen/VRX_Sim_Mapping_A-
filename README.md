<h1> Project ACIT4820 - Obstacle Detection and Path Planning for Unmanned Surface Vessels in ROS and Gazebo:<h6> 

1. This project details the implementation of Lidar based 2D mapping, applying the map for A* path planning implementation, and is based on the VRX classic Gazebo environment using a T-configured WAM-V USV. The main contributions are found under /src/project/wamv_navigation

2. The Sydneyregatta world was customized with the VRX buoy objects, while the WAM-V model was modified to remove the ball shooter.

4. Downloading workspace with dependencies is done by following the wiki for the Gazebo Classic for ROS 1 Noetic in https://github.com/osrf/vrx

5. For Lidar mapping launch vrx_gazebo sydneyregatta.launch before running the map generation script with RViz. The path planner is directly launched with the dedicated launch file once a map has been generated. 
