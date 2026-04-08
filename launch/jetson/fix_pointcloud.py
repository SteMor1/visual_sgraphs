#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2

class FixPointCloud(Node):
    def __init__(self):
        super().__init__('fix_pointcloud')
        self.sub = self.create_subscription(PointCloud2, '/camera/depth/points', self.cb, 10)
        self.pub = self.create_publisher(PointCloud2, '/camera/depth/points_organized', 10)

    def cb(self, msg):
        #Just correcting the header and dimensions, as isaac_ros_depth_image_proc outputs unorganized pointclouds
        msg.height = 480
        msg.width = 640
        msg.row_step = msg.point_step * 640
        self.pub.publish(msg)

rclpy.init()
rclpy.spin(FixPointCloud())