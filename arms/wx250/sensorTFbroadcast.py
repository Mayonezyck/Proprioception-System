#!/usr/bin/env python3
import sys
import math
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QHBoxLayout, QLabel, QDoubleSpinBox
from PyQt5.QtCore import QTimer
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster
from tf_transformations import quaternion_from_euler

class SensorTFNode(Node):
    def __init__(self):
        super().__init__('sensor_tf_gui_broadcaster_node')
        self.broadcaster = StaticTransformBroadcaster(self)

    def broadcast(self, tx, ty, tz, roll, pitch, yaw):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'wx250/left_finger_link'
        t.child_frame_id = 'wx250/left_sensor'
        t.transform.translation.x = tx
        t.transform.translation.y = ty
        t.transform.translation.z = tz
        q = quaternion_from_euler(roll, pitch, yaw)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        self.broadcaster.sendTransform(t)

class SensorTFGUI(QWidget):
    def __init__(self, node: SensorTFNode):
        super().__init__()
        self.node = node
        self.init_ui()
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_transform)
        self.timer.start(100)  # update at 10 Hz

    def init_ui(self):
        self.setWindowTitle('Sensor TF GUI Broadcaster')
        layout = QVBoxLayout()

        # Translation controls (meters)
        for axis in ['X', 'Y', 'Z']:
            h = QHBoxLayout()
            lbl = QLabel(f"{axis} (m)")
            spin = QDoubleSpinBox()
            spin.setRange(-1.0, 1.0)
            spin.setSingleStep(0.01)
            spin.setDecimals(3)
            h.addWidget(lbl)
            h.addWidget(spin)
            layout.addLayout(h)
            setattr(self, f'trans_{axis.lower()}', spin)

        # Rotation controls (degrees)
        for angle in ['Roll', 'Pitch', 'Yaw']:
            h = QHBoxLayout()
            lbl = QLabel(f"{angle} (°)")
            spin = QDoubleSpinBox()
            spin.setRange(-180.0, 180.0)
            spin.setSingleStep(0.01)
            spin.setDecimals(2)
            h.addWidget(lbl)
            h.addWidget(spin)
            layout.addLayout(h)
            setattr(self, f'rot_{angle.lower()}', spin)

        self.setLayout(layout)

    def update_transform(self):
        # read GUI values
        tx = self.trans_x.value()
        ty = self.trans_y.value()
        tz = self.trans_z.value()
        roll = math.radians(self.rot_roll.value())
        pitch = math.radians(self.rot_pitch.value())
        yaw = math.radians(self.rot_yaw.value())

        # broadcast updated transform
        self.node.broadcast(tx, ty, tz, roll, pitch, yaw)

        # process ROS callbacks
        rclpy.spin_once(self.node, timeout_sec=0)


def main():
    rclpy.init()
    node = SensorTFNode()
    app = QApplication(sys.argv)
    gui = SensorTFGUI(node)
    gui.show()

    try:
        app.exec_()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
