#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from interbotix_xs_msgs.srv import TorqueEnable
import tkinter as tk
import threading
import sys

def ros_spin_thread(node):
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

class TorqueToggle(Node):
    def __init__(self):
        super().__init__('torque_toggle')
        # Service client for torque_enable
        self.cli = self.create_client(TorqueEnable, '/wx250/torque_enable')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service /wx250/torque_enable...')
        self.req = TorqueEnable.Request()
        self.torque_enabled = False

    def set_torque(self, enable: bool):
        self.req.cmd_type = 'group'
        self.req.name = 'all'
        self.req.enable = enable
        future = self.cli.call_async(self.req)
        # Wait for future to complete without blocking spin thread
        while rclpy.ok() and not future.done():
            rclpy.spin_once(self, timeout_sec=0.1)
        result = future.result() if future.done() else None
        if result is not None:
            self.torque_enabled = enable
            state = 'enabled' if enable else 'disabled'
            self.get_logger().info(f'Torque {state}')
        else:
            self.get_logger().error('Service call failed')


def main():
    rclpy.init()
    node = TorqueToggle()
    # Start ROS spin in background thread
    thread = threading.Thread(target=ros_spin_thread, args=(node,), daemon=True)
    thread.start()

    # GUI setup (must run in main thread)
    root = tk.Tk()
    root.title("Torque Control")

    def on_enable():
        threading.Thread(target=lambda: node.set_torque(True), daemon=True).start()
        enable_button.config(state=tk.DISABLED)
        disable_button.config(state=tk.NORMAL)

    def on_disable():
        threading.Thread(target=lambda: node.set_torque(False), daemon=True).start()
        enable_button.config(state=tk.NORMAL)
        disable_button.config(state=tk.DISABLED)

    enable_button = tk.Button(root, text="Enable Torque", command=on_enable, width=20, height=2)
    enable_button.pack(padx=20, pady=5)
    disable_button = tk.Button(root, text="Disable Torque", command=on_disable, width=20, height=2)
    disable_button.pack(padx=20, pady=5)
    # Initially disable "Disable" button
    disable_button.config(state=tk.DISABLED)

    def on_close():
        # Disable torque and shutdown
        if node.torque_enabled:
            node.set_torque(False)
        rclpy.get_default_context().shutdown()
        root.destroy()
        sys.exit(0)

    root.protocol("WM_DELETE_WINDOW", on_close)
    root.mainloop()

if __name__ == '__main__':
    main()

