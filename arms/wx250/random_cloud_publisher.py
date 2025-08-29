#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from std_msgs.msg import Header
import random

class RandomCloudPublisher(Node):
    def __init__(self):
        super().__init__('random_cloud_publisher')
        # Publish on topic /random_cloud
        self.pub = self.create_publisher(PointCloud2, 'random_cloud', 10)
        # 1 Hz timer
        self.create_timer(1.0, self.publish_cloud)

    def publish_cloud(self):
        # generate 1000 random points in a cube [-1,1]×[-1,1]×[0,1]
        points = [
            [random.uniform(-1,1), random.uniform(-1,1), random.uniform(0,1)]
            for _ in range(1000)
        ]
        hdr = Header()
        hdr.stamp = self.get_clock().now().to_msg()
        hdr.frame_id = 'wx250/base_link'

        cloud_msg = pc2.create_cloud_xyz32(hdr, points)
        self.pub.publish(cloud_msg)
        self.get_logger().info('Published random point cloud')

def main(args=None):
    rclpy.init(args=args)
    node = RandomCloudPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

