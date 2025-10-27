#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import Float32
from sensor_msgs.msg import JointState
from control_msgs.action import GripperCommand

class PadDistanceServo(Node):
    """
    Drive the gripper so that /pad_centers_distance ~= target distance.

    Inputs:
      - /pad_centers_distance : std_msgs/Float32   (meters)
      - /joint_states         : sensor_msgs/JointState
      - /target_pad_distance  : std_msgs/Float32   (optional live target update)

    Action:
      - /gripper_controller/gripper_cmd : control_msgs/action/GripperCommand

    Parameters (declare via ROS params or command line):
      - tick_rate (float, Hz)                 [default: 30.0]
      - target_distance (float, m)            [default: 0.015]
      - tolerance (float, m)                  [default: 0.0005]
      - kp (float, m/m)                       [default: 0.5]    # proportional gain
      - max_step (float, m per tick)          [default: 0.0004]
      - joint_index (int)                     [default: 6]      # index of left finger in JointState.position
      - opening_pos (float, m)                [default: 0.040]
      - closing_pos (float, m)                [default: 0.0001]
      - max_effort (float)                    [default: 10.0]
      - force_limit (float, N)                [default: -2.0]   # same convention as your code; open if below
      - enable_force_safety (bool)            [default: True]
    """

    def __init__(self):
        super().__init__('pad_distance_servo')

        # --- Params ---
        self.declare_parameter('tick_rate', 60.0)
        self.declare_parameter('target_distance', 0.038)
        self.declare_parameter('tolerance', 0.0001)
        self.declare_parameter('kp', 0.5)
        self.declare_parameter('max_step', 0.00005)
        self.declare_parameter('joint_index', 6)
        self.declare_parameter('opening_pos', 0.040)
        self.declare_parameter('closing_pos', 0.0001)
        self.declare_parameter('max_effort', 10.0)
        self.declare_parameter('force_limit', -4.0)
        self.declare_parameter('enable_force_safety', True)

        tick_rate = float(self.get_parameter('tick_rate').value)
        period = 1.0 / tick_rate

        # --- State holders ---
        self.current_distance = None
        self.current_joint_pos = None
        self.force_z_sum = None
        self.target_distance = float(self.get_parameter('target_distance').value)
        self.joint_index = int(self.get_parameter('joint_index').value)

        # --- Subs ---
        self.sub_dist = self.create_subscription(Float32, '/pad_centers_distance', self._dist_cb, 10)
        self.sub_js   = self.create_subscription(JointState, '/joint_states', self._js_cb, 10)
        self.sub_fz   = self.create_subscription(Float32, '/feats/force_z_sum', self._fz_cb, 10)
        # Optional live target updates:
        self.sub_target = self.create_subscription(Float32, '/target_pad_distance', self._target_cb, 10)

        # --- Pub (optional error for monitoring) ---
        self.pub_error = self.create_publisher(Float32, '/pad_distance_error', 10)

        # --- Action client ---
        self.gripper_client = ActionClient(self, GripperCommand, '/gripper_controller/gripper_cmd')
        self.get_logger().info('Waiting for gripper action server...')
        self.gripper_client.wait_for_server()
        self.get_logger().info('Gripper action server ready.')

        # --- Timer ---
        self.timer = self.create_timer(period, self._on_tick)

        # --- For rate limiting commands ---
        self.last_cmd_pos = None

    # ----------------- Callbacks -----------------
    def _dist_cb(self, msg: Float32):
        self.current_distance = float(msg.data)

    def _js_cb(self, msg: JointState):
        try:
            self.current_joint_pos = float(msg.position[self.joint_index])
        except Exception:
            # Fallback: try to find finger joint by name if present
            if 'name' in msg.__slots__ and len(msg.name) == len(msg.position):
                try:
                    idx = msg.name.index('left_finger_joint')  # adjust if your joint name differs
                    self.current_joint_pos = float(msg.position[idx])
                    self.joint_index = idx
                except ValueError:
                    pass

    def _fz_cb(self, msg: Float32):
        self.force_z_sum = float(msg.data)

    def _target_cb(self, msg: Float32):
        self.target_distance = float(msg.data)
        self.get_logger().info(f'New target distance: {self.target_distance:.4f} m')

    # ----------------- Control loop -----------------
    def _on_tick(self):
        if self.current_distance is None or self.current_joint_pos is None:
            return  # need both to act

        # Safety: force threshold (same convention as your stepper)
        if self.get_parameter('enable_force_safety').value and self.force_z_sum is not None:
            if float(self.force_z_sum) < float(self.get_parameter('force_limit').value):
                self.get_logger().warn('Force safety triggered: opening gripper.')
                self._send_gripper(self.get_parameter('opening_pos').value)
                return

        target = float(self.get_parameter('target_distance').value) if self.target_distance is None else self.target_distance
        tol = float(self.get_parameter('tolerance').value)
        kp = float(self.get_parameter('kp').value)
        max_step = float(self.get_parameter('max_step').value)
        opening_pos = float(self.get_parameter('opening_pos').value)
        closing_pos = float(self.get_parameter('closing_pos').value)

        # Error > 0  => pads too far apart => close gripper (decrease finger pos)
        error = self.current_distance - target
        self.pub_error.publish(Float32(data=float(error)))

        if math.fabs(error) <= tol:
            # close enough; no command
            return

        # Proportional step, clamped
        raw_step = kp * error
        step = max(-max_step, min(max_step, raw_step))

        # Map distance correction to finger position change:
        # Positive error => close => decrease position
        desired_pos = self.current_joint_pos - step

        # Clamp to mechanical limits
        desired_pos = max(closing_pos, min(opening_pos, desired_pos))

        # Avoid spamming the same command
        if self.last_cmd_pos is None or abs(desired_pos - self.last_cmd_pos) > 1e-6:
            self._send_gripper(desired_pos)
            self.last_cmd_pos = desired_pos

    # ----------------- Action helper -----------------
    def _send_gripper(self, position: float):
        goal = GripperCommand.Goal()
        goal.command.position = float(position)
        goal.command.max_effort = float(self.get_parameter('max_effort').value)
        self.gripper_client.send_goal_async(goal)
        self.get_logger().info(f'Sent gripper goal: pos={position:.5f} m')

def main(args=None):
    rclpy.init(args=args)
    node = PadDistanceServo()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
