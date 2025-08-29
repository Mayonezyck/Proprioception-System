#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque
import threading
import time

class JointCurrentListener(Node):
    def __init__(self, buffer_size=200, num_plots=8):
        super().__init__('joint_current_listener')
        self.subscription = self.create_subscription(
            JointState,
            '/wx250/joint_states',
            self.listener_callback,
            10)
        self.buffer_size = buffer_size
        self.num_plots = num_plots
        # Deques to hold efforts for each joint
        self.effort_buffers = [deque(maxlen=self.buffer_size) for _ in range(self.num_plots)]
        self.joint_names = [f'joint{i}' for i in range(self.num_plots)]
        self.initialized = False

    def listener_callback(self, msg: JointState):
        # On first message, record joint names and mark initialized
        if not self.initialized:
            # Use the names from the message for labeling
            self.joint_names = list(msg.name)[:self.num_plots]
            self.initialized = True
        # Append efforts for each joint (up to num_plots)
        for i in range(self.num_plots):
            if i < len(msg.effort):
                self.effort_buffers[i].append(msg.effort[i])
            else:
                self.effort_buffers[i].append(0.0)


def ros_spin_thread(node):
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


def main():
    # Initialize ROS2
    rclpy.init()
    listener = JointCurrentListener(buffer_size=200, num_plots=8)

    # Start ROS2 spin in a separate thread
    thread = threading.Thread(target=ros_spin_thread, args=(listener,), daemon=True)
    thread.start()

    # Set up matplotlib figure and axes
    fig, ax = plt.subplots()
    lines = []
    for i in range(listener.num_plots):  # Plot each joint
        line, = ax.plot([], [], label=listener.joint_names[i])
        lines.append(line)
    ax.set_xlim(0, listener.buffer_size)
    ax.set_xlabel('Sample Index (most recent 200)')
    ax.set_ylabel('Effort (Current)')
    ax.set_title('WX250 Joint Currents (Effort) - Real-time')
    ax.legend(loc='upper right')

    def update_plot(frame):
        if not listener.initialized:
            return lines

        # x-axis: 0 to buffer_size-1
        x_vals = list(range(listener.buffer_size))
        # Collect all current data to adjust y-axis
        all_values = []
        for i, line in enumerate(lines):
            data_len = len(listener.effort_buffers[i])
            y_vals = list(listener.effort_buffers[i])
            # Pad y_vals if shorter than buffer_size
            if data_len < listener.buffer_size:
                pad = [0.0] * (listener.buffer_size - data_len)
                y_vals = pad + y_vals
            line.set_data(x_vals, y_vals)
            # Update label in case joint_names were updated
            line.set_label(listener.joint_names[i] if i < len(listener.joint_names) else f'joint{i}')
            all_values.extend(y_vals)

        # Dynamically adjust y-axis based on visible data
        if all_values:
            ymin = min(all_values)
            ymax = max(all_values)
            # Add some margin
            margin = (ymax - ymin) * 0.1 if ymax != ymin else 1.0
            ax.set_ylim(ymin - margin, ymax + margin)

        # Update legend with new labels
        ax.legend(loc='upper right')

        return lines

    ani = animation.FuncAnimation(fig, update_plot, interval=100)

    plt.tight_layout()
    plt.show()

    # Keep the program alive to listen until user closes the plot window
    try:
        while rclpy.ok():
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    main()

