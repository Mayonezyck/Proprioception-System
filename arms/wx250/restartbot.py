#!/usr/bin/env python3
"""
ROS2 node to disable and enable torque (i.e. "shutdown" and "restart") on all WidowX WX250 motors via service calls.
"""

import rclpy
from rclpy.node import Node
from interbotix_xs_msgs.srv import TorqueEnable
import time


def main():
    # Initialize ROS 2
    rclpy.init()
    node = rclpy.create_node('wx250_motor_control')
    srv_name = '/wx250/torque_enable'
    client = node.create_client(TorqueEnable, srv_name)

    node.get_logger().info(f'Waiting for service {srv_name}...')
    if not client.wait_for_service(timeout_sec=5.0):
        node.get_logger().error(f'Service {srv_name} not available.')
        rclpy.shutdown()
        return

    # Disable torque on all joints
    req = TorqueEnable.Request()
    req.cmd_type = 'group'
    req.name = 'all'
    req.enable = False
    node.get_logger().info('Disabling torque on all joints...')
    future = client.call_async(req)
    rclpy.spin_until_future_complete(node, future)
    if future.result() is not None:
        node.get_logger().info('All motors disabled.')
    else:
        node.get_logger().error(f'Disable call failed: {future.exception()}')
        rclpy.shutdown()
        return

    # Brief pause to ensure shutdown
    time.sleep(10)

    # Re-enable torque on all joints
    req.enable = True
    node.get_logger().info('Enabling torque on all joints...')
    future = client.call_async(req)
    rclpy.spin_until_future_complete(node, future)
    if future.result() is not None:
        node.get_logger().info('All motors enabled. Restart complete.')
    else:
        node.get_logger().error(f'Enable call failed: {future.exception()}')

    # Cleanup
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

