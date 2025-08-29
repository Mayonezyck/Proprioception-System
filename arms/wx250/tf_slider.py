#!/usr/bin/env python3
import tkinter as tk
from math import sin, cos, radians
import rclpy
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster


def euler_to_quaternion(roll, pitch, yaw):
    r = radians(roll)
    p = radians(pitch)
    y = radians(yaw)
    cy = cos(y * 0.5)
    sy = sin(y * 0.5)
    cp = cos(p * 0.5)
    sp = sin(p * 0.5)
    cr = cos(r * 0.5)
    sr = sin(r * 0.5)
    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return qx, qy, qz, qw


def send_static_transform(x, y, z, roll, pitch, yaw):
    qx, qy, qz, qw = euler_to_quaternion(roll, pitch, yaw)
    t = TransformStamped()
    t.header.stamp = node.get_clock().now().to_msg()
    t.header.frame_id = 'camera_color_optical_frame'
    t.child_frame_id = 'wx250/base_link'
    t.transform.translation.x = x
    t.transform.translation.y = y
    t.transform.translation.z = z
    t.transform.rotation.x = qx
    t.transform.rotation.y = qy
    t.transform.rotation.z = qz
    t.transform.rotation.w = qw
    broadcaster.sendTransform(t)


def update_tf():
    x = x_var.get()
    y = y_var.get()
    z = z_var.get()
    roll = roll_var.get()
    pitch = pitch_var.get()
    yaw = yaw_var.get()
    send_static_transform(x, y, z, roll, pitch, yaw)


def spin_ros():
    rclpy.spin_once(node, timeout_sec=0.1)
    root.after(100, spin_ros)


def on_close():
    rclpy.shutdown()
    root.destroy()


# Initialize ROS 2
rclpy.init()
node = rclpy.create_node('static_tf_gui_node')
broadcaster = StaticTransformBroadcaster(node)

# GUI setup
root = tk.Tk()
root.title("Static TF Publisher")

# Variables with defaults
x_var = tk.DoubleVar(value=-0.15)
y_var = tk.DoubleVar(value=0.07)
z_var = tk.DoubleVar(value=0.54)
roll_var = tk.DoubleVar(value=4.03)
pitch_var = tk.DoubleVar(value=-116.87)
yaw_var = tk.DoubleVar(value=83.0)

params = [
    ("X (m)", x_var, -5.0, 5.0),
    ("Y (m)", y_var, -5.0, 5.0),
    ("Z (m)", z_var, 0.0, 2.0),
    ("Roll (°)", roll_var, -180, 180),
    ("Pitch (°)", pitch_var, -180, 180),
    ("Yaw (°)", yaw_var, -180, 180),
]

# Create sliders and bind updates on change
for idx, (label, var, lo, hi) in enumerate(params):
    tk.Label(root, text=label).grid(row=idx, column=0, sticky="w")
    tk.Scale(root, variable=var, from_=lo, to=hi, orient="horizontal",
             resolution=0.01, length=300).grid(row=idx, column=1)
    var.trace_add("write", lambda *args: update_tf())

# Publish initial transform
update_tf()

# Handle window close
root.protocol("WM_DELETE_WINDOW", on_close)

# Start ROS spinning loop
root.after(100, spin_ros)

# Enter GUI main loop
root.mainloop()

