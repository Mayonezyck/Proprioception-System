#!/usr/bin/env python3

import sys
import rclpy
from rclpy.node import Node

# Import the service and message types
from interbotix_xs_msgs.srv import RobotInfo          # Get robot info service :contentReference[oaicite:5]{index=5}
from interbotix_xs_msgs.msg import JointGroupCommand  # Joint group command message :contentReference[oaicite:6]{index=6}

class MoveHomeNode(Node):
    def __init__(self):
        super().__init__('move_wx250_home')
        self.robot_name = 'wx250'  # adjust if you used a different robot_name :contentReference[oaicite:7]{index=7}

        # 1) Create a service client for RobotInfo
        svc_name = f'/{self.robot_name}/get_robot_info'
        self.cli = self.create_client(RobotInfo, svc_name)
        if not self.cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(f'Service {svc_name} not available.')
            sys.exit(1)

        # 2) Call the service to get the home positions
        req = RobotInfo.Request()
        req.cmd_type = 'group'  # we want the group info :contentReference[oaicite:8]{index=8}
        req.name = 'arm'        # default joint group name :contentReference[oaicite:9]{index=9}
        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        res = future.result()
        if res is None:
            self.get_logger().error('RobotInfo service call failed')
            sys.exit(1)

        # 3) Prepare publisher for JointGroupCommand
        topic = f'/{self.robot_name}/commands/joint_group'
        pub = self.create_publisher(JointGroupCommand, topic, 10)  # publisher queue size 10 :contentReference[oaicite:10]{index=10}

        # 4) Construct and publish the home-position command
        cmd = JointGroupCommand()
        cmd.name = 'arm'
        cmd.cmd = res.joint_sleep_positions  # the “home” positions returned by the service :contentReference[oaicite:11]{index=11}
        pub.publish(cmd)
        self.get_logger().info(f'Published home pose to {topic}: {cmd.cmd}')

        # Allow some time for the message to be sent before shutdown
        self.create_timer(1.0, lambda: rclpy.shutdown())

def main(args=None):
    rclpy.init(args=args)
    node = MoveHomeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

