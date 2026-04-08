#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from message_filters import ApproximateTimeSynchronizer, Subscriber

class SyncConverter(Node):
    def __init__(self):
        super().__init__('sync_converter')
        self.sub_img = Subscriber(self, Image, '/camera/rgb/image_color')
        self.sub_depth = Subscriber(self, Image, '/camera/depth/image')
        self.sub_info = Subscriber(self, CameraInfo, '/camera/rgb/camera_info')
        self.sync = ApproximateTimeSynchronizer(
            [self.sub_img, self.sub_depth, self.sub_info], 10, 0.1)
        self.sync.registerCallback(self.cb)

        self.pub_img = self.create_publisher(Image, '/camera/rgb/image_sync', 10)
        self.pub_depth = self.create_publisher(Image, '/camera/depth/image_sync', 10)
        self.pub_info = self.create_publisher(CameraInfo, '/camera/rgb/camera_info_sync', 10)

    def cb(self, img_msg, depth_msg, info_msg):
        out = Image()
        out.header = img_msg.header
        out.height = img_msg.height
        out.width = img_msg.width
        out.is_bigendian = img_msg.is_bigendian
        out.step = img_msg.step
        out.encoding = img_msg.encoding
        out.data = img_msg.data
        self.pub_img.publish(out)
        depth_msg.header = img_msg.header
        info_msg.header = img_msg.header
        self.pub_depth.publish(depth_msg)
        self.pub_info.publish(info_msg)

rclpy.init()
rclpy.spin(SyncConverter())