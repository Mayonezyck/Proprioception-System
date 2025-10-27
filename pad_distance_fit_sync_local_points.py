#!/usr/bin/env python3
import math
from datetime import datetime
import os
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from std_msgs.msg import Float32
from sensor_msgs.msg import JointState, Image
from control_msgs.action import GripperCommand
from geometry_msgs.msg import PointStamped

from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2 as pc2
from visualization_msgs.msg import Marker  # only if you also want to reuse pad debug markers later
import tf2_ros
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy
from std_msgs.msg import Int32MultiArray, Float32MultiArray


import numpy as np


class PadDistanceFitAndSync(Node):
    """
    Steps toward a target pad distance, records time-series, aligns
    (force, depth_avg, distance) by stamp, computes linear fit (F = m*x + b),
    plots data+fit, returns gripper to original position.

    Force is derived from an Image topic (header provides the stamp).
    Depth avg is read from /gs/depth_avg_stamped (preferably stamped).
    """

    def __init__(self):
        super().__init__('pad_distance_fit_and_sync')

        # --- stepping / safety parameters ---
        self.declare_parameter('target_distance', 0.028)
        self.declare_parameter('tolerance', 0.0001)
        self.declare_parameter('step_size', 0.00005)
        self.declare_parameter('max_steps', 500)
        self.declare_parameter('opening_pos', 0.040)
        self.declare_parameter('closing_pos', 0.0001)
        self.declare_parameter('max_effort', 30.0)
        self.declare_parameter('force_limit', -15.0)
        self.declare_parameter('enable_force_safety', True)
        self.declare_parameter('joint_index', 6)
        self.declare_parameter('data_force_threshold', self.get_parameter('force_limit').value)
        self.declare_parameter('tick_rate', 60.0)
        self.declare_parameter('return_to_original_on_finish', True)

        # --- topics (tweak as needed) ---
        self.declare_parameter('force_image_topic', '/feats/force_z')   # Image with header
        self.declare_parameter('depth_avg_topic', '/gs/depth_avg_stamped')    # should have header if possible
        self.declare_parameter('pad_distance_topic', '/pad_centers_distance') # Float32 (no header)

        # --- synchronization / joining ---
        self.declare_parameter('join_slop_ms', 20.0)  # nearest-neighbor slop for stamp joining

        # --- regression parameters (force vs distance) ---
        self.declare_parameter('reg_force_min', float('-inf'))
        self.declare_parameter('reg_force_max', float('inf'))
        self.declare_parameter('reg_use_last_n', 0)       # 0 => use all
        self.declare_parameter('reg_outlier_sigma', 0.0)  # 0 => no trimming
        self.declare_parameter('segment_size', 50)  # segmented fit chunk size; 0 disables segmented plotting

        # ---- contact / cloud params ----
        self.declare_parameter('input_cloud_topic', '/input_cloud')
        self.declare_parameter('left_pad_frame', 'left_gelsight_pad')
        self.declare_parameter('right_pad_frame', 'right_gelsight_pad')
        self.declare_parameter('left.size_xyz',  [0.020, 0.020, 0.010])   # meters
        self.declare_parameter('right.size_xyz', [0.020, 0.020, 0.010])   # meters
        self.declare_parameter('left.offset_xyz',  [0.0, 0.0, 0.0])       # meters
        self.declare_parameter('right.offset_xyz', [0.0, 0.0, 0.0])       # meters
        self.declare_parameter('contact_tolerance', 0.0005)               # meters
        self.declare_parameter('tf_lookup_timeout_sec', 0.25)

        # ---- update interface to shape_cloud_publisher ----
        self.declare_parameter('update.by_index.indices_topic', '/stiffness_update/indices')
        self.declare_parameter('update.by_index.values_topic',  '/stiffness_update/values')
        self.declare_parameter('update.by_points.topic',        '/stiffness_update/points')  # PointCloud2 x,y,z,stiffness
        self.declare_parameter('update.by_points.enabled',      False)  # index updates recommended

        # --- outputs ---
        default_run_dir = f"pad_fit_run_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
        self.declare_parameter('output_dir', default_run_dir)
        default_summary = f"pad_fit_summary_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        self.declare_parameter('summary_csv_path', default_summary)  # set "" to disable saving
        self.declare_parameter('fig_path', f"pad_fit_scatter_{datetime.now().strftime('%Y%m%d_%H%M%S')}.png")




        # --- fetch params ---
        self.target_distance = float(self.get_parameter('target_distance').value)
        self.tolerance = float(self.get_parameter('tolerance').value)
        self.step_size = float(self.get_parameter('step_size').value)
        self.max_steps = int(self.get_parameter('max_steps').value)
        self.opening_pos = float(self.get_parameter('opening_pos').value)
        self.closing_pos = float(self.get_parameter('closing_pos').value)
        self.max_effort = float(self.get_parameter('max_effort').value)
        self.force_limit = float(self.get_parameter('force_limit').value)
        self.enable_force_safety = bool(self.get_parameter('enable_force_safety').value)
        self.data_force_threshold = float(self.get_parameter('data_force_threshold').value)
        self.joint_index = int(self.get_parameter('joint_index').value)
        self.return_to_original_on_finish = bool(self.get_parameter('return_to_original_on_finish').value)

        self.force_image_topic = str(self.get_parameter('force_image_topic').value)
        self.depth_avg_topic = str(self.get_parameter('depth_avg_topic').value)
        self.pad_distance_topic = str(self.get_parameter('pad_distance_topic').value)

        self.join_slop_ns = int(float(self.get_parameter('join_slop_ms').value) * 1e6)

        self.reg_force_min = float(self.get_parameter('reg_force_min').value)
        self.reg_force_max = float(self.get_parameter('reg_force_max').value)
        self.reg_use_last_n = int(self.get_parameter('reg_use_last_n').value)
        self.reg_outlier_sigma = float(self.get_parameter('reg_outlier_sigma').value)
        self.segment_size = int(self.get_parameter('segment_size').value)


        self.input_cloud_topic = str(self.get_parameter('input_cloud_topic').value)
        self.left_pad_frame  = str(self.get_parameter('left_pad_frame').value)
        self.right_pad_frame = str(self.get_parameter('right_pad_frame').value)
        #import numpy as np
        self.left_size   = np.array(self.get_parameter('left.size_xyz').value,   dtype=np.float32)
        self.right_size  = np.array(self.get_parameter('right.size_xyz').value,  dtype=np.float32)
        self.left_offset = np.array(self.get_parameter('left.offset_xyz').value, dtype=np.float32)
        self.right_offset= np.array(self.get_parameter('right.offset_xyz').value,dtype=np.float32)
        self.contact_eps = float(self.get_parameter('contact_tolerance').value)
        self.tf_timeout  = float(self.get_parameter('tf_lookup_timeout_sec').value)


        self.idx_topic   = str(self.get_parameter('update.by_index.indices_topic').value)
        self.val_topic   = str(self.get_parameter('update.by_index.values_topic').value)
        self.pts_topic   = str(self.get_parameter('update.by_points.topic').value)
        self.pts_enabled = bool(self.get_parameter('update.by_points.enabled').value)

        self.summary_csv_path = str(self.get_parameter('summary_csv_path').value)
        self.fig_path = str(self.get_parameter('fig_path').value)
        # --- output directory & path normalization ---
        self.output_dir = str(self.get_parameter('output_dir').value)
        os.makedirs(self.output_dir, exist_ok=True)

        # Always write outputs inside output_dir, preserving filenames
        self.summary_csv_path = os.path.join(self.output_dir, os.path.basename(self.summary_csv_path)) if self.summary_csv_path else ""
        self.fig_path = os.path.join(self.output_dir, os.path.basename(self.fig_path)) if self.fig_path else ""

        self.get_logger().info(f"[IO] Outputs will be saved under: {os.path.abspath(self.output_dir)}")



        # --- state ---
        self.current_distance = None
        self.force_z_sum = None
        self.joint_pos = None
        self.original_pos = None
        self.steps_taken = 0
        self.finished = False
        self.collection_stopped = False
        self.sent_any_motion = False
        self.depth_baseline = None



        # time-series buffers (for quick visualization & regression in distance domain)
        self.buf_time = []
        self.buf_dist = []
        self.buf_force = []
        self.buf_joint = []

        # stamp-indexed caches
        self.force_by_stamp = {}  # {(sec, nsec): float}
        self.depth_by_stamp = {}  # {(sec, nsec): float}

        # ---- latest cloud cache ----
        self.latest_cloud_pts = None      # np.ndarray (N,3)
        self.latest_cloud_header = None   # std_msgs/Header
        self.latest_cloud_frame = None

        # --- subs ---
        # Force from an Image; we compute a proxy force (replace with your converter)
        self.sub_force_img = self.create_subscription(Image, self.force_image_topic, self._force_img_cb, 20)

        # Depth avg; try to read header if present; else fallback to node time
        # Using Float32 because it's common; if your msg has .header + .data, this still works.
        self.sub_depth = self.create_subscription(PointStamped, self.depth_avg_topic, self._depth_avg_cb, 50)

        # Distance (no header)
        self.sub_d = self.create_subscription(Float32, self.pad_distance_topic, self._dist_cb, 50)

        # Joint states
        self.sub_j = self.create_subscription(JointState, '/joint_states', self._js_cb, 50)

        # ---- subscribe cloud ----
        qos_cloud = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=5, reliability=QoSReliabilityPolicy.RELIABLE)
        self.sub_cloud = self.create_subscription(PointCloud2, self.input_cloud_topic, self._on_cloud, qos_cloud)

        # --- action client ---
        self.client = ActionClient(self, GripperCommand, '/gripper_controller/gripper_cmd')
        self.get_logger().info('Waiting for gripper action server...')
        self.client.wait_for_server()
        self.get_logger().info('Gripper action server ready.')

        # ---- stiffness map publisher (latched) ----
        # qos_lat = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=1,
        #                     reliability=QoSReliabilityPolicy.RELIABLE,
        #                     durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        # self.pub_stiff_map = self.create_publisher(PointCloud2, '/stiffness_map_points', qos_lat)

        qos_up = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=5, reliability=QoSReliabilityPolicy.RELIABLE)
        self.pub_idx = self.create_publisher(Int32MultiArray,  self.idx_topic, qos_up)
        self.pub_val = self.create_publisher(Float32MultiArray, self.val_topic, qos_up)


        # ---- TF buffer/listener ----
        self.tf_buffer = tf2_ros.Buffer(cache_time=rclpy.duration.Duration(seconds=5.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

                # --- timers ---
        tick_rate = float(self.get_parameter('tick_rate').value)
        self.timer = self.create_timer(1.0 / tick_rate, self._on_tick)

        self.get_logger().info(
            f"[INIT] target={self.target_distance:.6f} m, tol={self.tolerance:.6f}, step={self.step_size:.6f}, "
            f"slop={self.join_slop_ns/1e6:.1f} ms, "
            f"force_topic={self.force_image_topic}, depth_topic={self.depth_avg_topic}, dist_topic={self.pad_distance_topic}"
        )

    # ------------- utility: stamps & nearest -------------
    def _stamp_key(self, stamp) -> tuple:
        return (int(stamp.sec), int(stamp.nanosec))

    def _find_nearest_within(self, key_set, target_key, slop_ns):
        if not key_set:
            return None
        t_ns = target_key[0]*1_000_000_000 + target_key[1]
        best = None
        best_dt = None
        for k in key_set:
            k_ns = k[0]*1_000_000_000 + k[1]
            dt = abs(k_ns - t_ns)
            if best_dt is None or dt < best_dt:
                best_dt = dt; best = k
        return best if (best_dt is not None and best_dt <= slop_ns) else None



    # ------------- callbacks -------------
    def _force_img_cb(self, msg: Image):
        """
        Compute scalar force as the sum of the force_z image.
        Uses the image's header.stamp as the cache key for synchronization.
        """
        try:
            fz = self._img32_from_ros(msg)            # HxW float32
            force = float(np.nansum(fz))              # sum of all pixels]
            print(force)
        except Exception as e:
            self.get_logger().warn(f"[FORCE] decode/sum failed: {e}")
            force = float('nan')

        k = self._stamp_key(msg.header.stamp)
        self.force_z_sum = force
        self.force_by_stamp[k] = force
        if (not self.finished) and (not self.collection_stopped):
            if math.isfinite(force) and (force < self.data_force_threshold):
                self.collection_stopped = True
                self.get_logger().warn(
                    f"[FORCE] Threshold reached (force={force:.6f} < {self.data_force_threshold:.6f}). Stopping collection and finishing."
                )
                self._finish(); return
        # Also push to continuous buffers for distance-domain fit/plot
        self._maybe_push_sample()

        # Debug every ~25 frames
        if (len(self.force_by_stamp) % 25) == 0:
            self.get_logger().info(f"[FORCE] cached={len(self.force_by_stamp)} stamp={k} sum={force:.6f}")
            print(f"[FORCE] stamp={k} sum={force:.6f}")  # stdout


    def _depth_avg_cb(self, msg: PointStamped):
        # Use the exact ROS timestamp from the header
        k = self._stamp_key(msg.header.stamp)

        # You are publishing depth in point.x
        depth_raw = float(msg.point.x)

        # Guard against bad values
        if not math.isfinite(depth_raw):
            self.get_logger().warn(f"[DEPTH] Non-finite depth at stamp={k}: {depth_raw}; skipping.")
            return

        # Initialize baseline on the first good sample
        if self.depth_baseline is None:
            self.depth_baseline = depth_raw
            self.get_logger().info(f"[DEPTH] Baseline captured: {self.depth_baseline:.6f} m")

        # Store Δdepth = depth_raw - baseline
        depth_delta = depth_raw - self.depth_baseline
        self.depth_by_stamp[k] = depth_delta

        # Debug stdout + periodic ROS log
        print(f"[DEPTH] stamp={k} depth_raw={depth_raw:.6f} m, Δdepth={depth_delta:.6f} m")
        if (len(self.depth_by_stamp) % 50) == 0:
            self.get_logger().info(
                f"[DEPTH] cached={len(self.depth_by_stamp)} latest_k={k} Δdepth={depth_delta:.6f} m"
            )


    def _dist_cb(self, msg: Float32):
        self.current_distance = float(msg.data)

    def _img32_from_ros(self, msg: Image) -> np.ndarray:
        """Expect 32FC1; fallback to common encodings with conversion to float32."""
        enc = (msg.encoding or "").lower()
        H, W, step = int(msg.height), int(msg.width), int(msg.step)
        buf = msg.data if isinstance(msg.data, (bytes, bytearray)) else bytes(msg.data)

        if enc in ('32fc1', '32fc'):
            arr = np.frombuffer(buf, dtype=np.float32)
            if arr.size == H * W:
                return arr.reshape(H, W)
            # If step padding: reshape via step (bytes/row = step)
            rows = np.frombuffer(buf, dtype=np.uint8).reshape(H, step)
            # take first W*4 bytes of each row and view as float32
            view = rows[:, : W * 4].reshape(H * W * 4)
            return np.frombuffer(view.tobytes(), dtype=np.float32).reshape(H, W)

        elif enc in ('16uc1', '16uc'):
            arr = np.frombuffer(buf, dtype=np.uint16)
            arr = arr.reshape(H, W) if arr.size == H * W else arr[: H * W].reshape(H, W)
            return arr.astype(np.float32)

        elif enc in ('mono8', '8uc1'):
            arr = np.frombuffer(buf, dtype=np.uint8)
            arr = arr.reshape(H, W) if arr.size == H * W else arr[: H * W].reshape(H, W)
            return arr.astype(np.float32)

        else:
            self.get_logger().warn_once(f"[FORCE] Unexpected encoding '{enc}', trying 32FC1 layout fallback.")
            arr = np.frombuffer(buf, dtype=np.float32)
            if arr.size == H * W:
                return arr.reshape(H, W)
            return arr.astype(np.float32)

    def _on_cloud(self, msg: PointCloud2):
        try:
            pts = np.array([[p[0], p[1], p[2]]
                            for p in pc2.read_points(msg, field_names=('x','y','z'), skip_nans=True)],
                        dtype=np.float32)
            if pts.size == 0:
                return
            self.latest_cloud_pts = pts
            self.latest_cloud_header = msg.header
            self.latest_cloud_frame = msg.header.frame_id
        except Exception as e:
            self.get_logger().warn(f"[CLOUD] Failed to decode PointCloud2: {e}")



    def _js_cb(self, msg: JointState):
        try:
            self.joint_pos = float(msg.position[self.joint_index])
        except Exception:
            try:
                if hasattr(msg, 'name') and len(msg.name) == len(msg.position):
                    idx = msg.name.index('left_finger_joint')  # adjust for your naming
                    self.joint_pos = float(msg.position[idx])
                    self.joint_index = idx
            except Exception:
                pass

    def _maybe_push_sample(self):
        if self.collection_stopped or self.finished:
            return
        if self.current_distance is None or self.force_z_sum is None or self.joint_pos is None:
            return
        t = self.get_clock().now().nanoseconds * 1e-9
        self.buf_time.append(t)
        self.buf_dist.append(self.current_distance)
        self.buf_force.append(self.force_z_sum)
        self.buf_joint.append(self.joint_pos)

    # ------------- stepping loop -------------
    def _on_tick(self):
        if self.finished:
            return
        if self.current_distance is None or self.joint_pos is None:
            return
        if self.collection_stopped:
            return

        if not self.sent_any_motion and self.original_pos is None:
            self.original_pos = float(self.joint_pos)
            self.get_logger().info(f"[STEP] original_pos set to {self.original_pos:.6f} m")

        # Safety
        if self.enable_force_safety and self.force_z_sum is not None and self.force_z_sum < self.force_limit:

            self.get_logger().warn('[STEP] Force safety triggered. Opening and finishing.')
            self._send_goal(self.opening_pos, mark_motion=False)
            self._finish()
            return

        error = self.current_distance - self.target_distance
        if math.fabs(error) <= self.tolerance:
            self.get_logger().info(f"[STEP] Target reached: |{error:.6f}| <= {self.tolerance:.6f}")
            self._finish()
            return

        if self.steps_taken >= self.max_steps:
            self.get_logger().warn(f"[STEP] Max steps reached ({self.max_steps}). Stopping.")
            self._finish()
            return

        next_pos = self.joint_pos - self.step_size if error > 0.0 else self.joint_pos + self.step_size
        next_pos = min(max(next_pos, self.closing_pos), self.opening_pos)

        self._send_goal(next_pos, mark_motion=True)
        self.steps_taken += 1

        if (self.steps_taken % 20) == 0:
            self.get_logger().info(f"[STEP] step={self.steps_taken} cmd_pos={next_pos:.6f} dist={self.current_distance:.6f}")

        if next_pos in (self.closing_pos, self.opening_pos):
            self.get_logger().warn('[STEP] Hit mechanical limit. Finishing.')
            self._finish()

    # ------------- action helpers -------------
    def _send_goal(self, position_m: float, mark_motion: bool):
        goal = GripperCommand.Goal()
        goal.command.position = float(position_m)
        goal.command.max_effort = float(self.max_effort)
        self.client.send_goal_async(goal)
        if mark_motion:
            self.sent_any_motion = True

    def _send_goal_and_wait(self, position_m: float, timeout_sec: float = 5.0) -> bool:
        goal = GripperCommand.Goal()
        goal.command.position = float(position_m)
        goal.command.max_effort = float(self.max_effort)

        send_future = self.client.send_goal_async(goal)
        ok = rclpy.spin_until_future_complete(self, send_future, timeout_sec=timeout_sec)
        if not ok:
            self.get_logger().warn(f"[ACTION] Timeout sending goal ({timeout_sec}s).")
            return False

        goal_handle = send_future.result()
        if not goal_handle or (hasattr(goal_handle, "accepted") and not goal_handle.accepted):
            self.get_logger().warn("[ACTION] Goal not accepted.")
            return False

        result_future = goal_handle.get_result_async()
        ok = rclpy.spin_until_future_complete(self, result_future, timeout_sec=timeout_sec)
        if not ok:
            self.get_logger().warn(f"[ACTION] Timeout waiting for result ({timeout_sec}s). Cancelling...")
            try:
                cancel_future = goal_handle.cancel_goal_async()
                rclpy.spin_until_future_complete(self, cancel_future, timeout_sec=1.0)
            except Exception:
                pass
            return False

        return True

    # -------------- contact map helper
    def _points_in_frame(self, target_frame: str, source_frame: str, P_source: np.ndarray):
        """Transform points from source_frame -> target_frame. Returns (ok, P_target)."""
        from rclpy.duration import Duration
        try:
            ok = self.tf_buffer.can_transform(target_frame=target_frame,
                                            source_frame=source_frame,
                                            time=rclpy.time.Time(),
                                            timeout=Duration(seconds=self.tf_timeout))
            if not ok:
                self.get_logger().warn_once(f"[TF] Not available yet: {source_frame} -> {target_frame}")
                return False, np.empty((0,3), dtype=np.float32)
            tf = self.tf_buffer.lookup_transform(target_frame=target_frame,
                                                source_frame=source_frame,
                                                time=rclpy.time.Time(),
                                                timeout=Duration(seconds=self.tf_timeout))
        except Exception as e:
            self.get_logger().warn_once(f"[TF] lookup failed {source_frame}->{target_frame}: {e}")
            return False, np.empty((0,3), dtype=np.float32)
        R, p = _tf_to_RT(tf)
        return True, (R @ P_source.T).T + p

    def _contact_mask_box(self, P_local: np.ndarray, size_xyz: np.ndarray, offset_xyz: np.ndarray, eps: float):
        """Axis-aligned box centered at 'offset_xyz' in the local frame."""
        Q = P_local - offset_xyz.reshape(1,3)
        hx, hy, hz = (size_xyz * 0.5).tolist()
        return (np.abs(Q[:,0]) <= hx + eps) & (np.abs(Q[:,1]) <= hy + eps) & (np.abs(Q[:,2]) <= hz + eps)

    def _publish_stiffness_map(self, slope: float):
        """
        Build a new PointCloud2 containing ONLY points currently in contact with either pad,
        with fields: x,y,z,stiffness,rgb. Colors: -10000 (red) .. 0 (blue).
        Latched on /stiffness_map_points so RViz can see it after we exit.
        """
        if self.latest_cloud_pts is None or self.latest_cloud_header is None or self.latest_cloud_frame is None:
            self.get_logger().warn("[STIFFMAP] No latest cloud available; skipping.")
            return

        # Transform latest cloud into left/right pad frames
        L_ok, P_L = self._points_in_frame(self.left_pad_frame,  self.latest_cloud_frame, self.latest_cloud_pts)
        R_ok, P_R = self._points_in_frame(self.right_pad_frame, self.latest_cloud_frame, self.latest_cloud_pts)

        if not (L_ok or R_ok):
            self.get_logger().warn("[STIFFMAP] Neither pad TF available; skipping.")
            return

        # Contact masks in each pad local frame
        Lmask = self._contact_mask_box(P_L, self.left_size,  self.left_offset,  self.contact_eps) if L_ok else np.zeros(len(self.latest_cloud_pts), dtype=bool)
        Rmask = self._contact_mask_box(P_R, self.right_size, self.right_offset, self.contact_eps) if R_ok else np.zeros(len(self.latest_cloud_pts), dtype=bool)
        M = Lmask | Rmask
        count = int(M.sum())
        if count == 0:
            self.get_logger().warn("[STIFFMAP] No contacting points found; publishing empty cloud.")
            # Publish an empty cloud with same frame so RViz can clear
            empty = pc2.create_cloud(self.latest_cloud_header, [
                PointField(name='x', offset=0,  datatype=PointField.FLOAT32, count=1),
                PointField(name='y', offset=4,  datatype=PointField.FLOAT32, count=1),
                PointField(name='z', offset=8,  datatype=PointField.FLOAT32, count=1),
                PointField(name='stiffness', offset=12, datatype=PointField.FLOAT32, count=1),
                PointField(name='rgb', offset=16, datatype=PointField.FLOAT32, count=1),
            ], [])
            self.pub_stiff_map.publish(empty)
            return

        # Subset points; assign stiffness & color
        pts = self.latest_cloud_pts[M]
        rgb_val = _stiffness_to_rgb(slope)
        stiff = float(slope)

        idx = np.nonzero(M)[0].astype(np.int32)
        if idx.size > 0:
            msg_i = Int32MultiArray(data=idx.tolist())
            msg_v = Float32MultiArray(data=[stiff]*idx.size)
            self.pub_idx.publish(msg_i)
            self.pub_val.publish(msg_v)
            self.get_logger().info(f"[UPDATE:index] sent {idx.size} indices to {self.idx_topic}/{self.val_topic}")


        # # Build (x,y,z,stiffness,rgb)
        # out_iter = ((float(x), float(y), float(z), stiff, rgb_val) for (x,y,z) in pts)
        # fields = [
        #     PointField(name='x', offset=0,  datatype=PointField.FLOAT32, count=1),
        #     PointField(name='y', offset=4,  datatype=PointField.FLOAT32, count=1),
        #     PointField(name='z', offset=8,  datatype=PointField.FLOAT32, count=1),
        #     PointField(name='stiffness', offset=12, datatype=PointField.FLOAT32, count=1),
        #     PointField(name='rgb', offset=16, datatype=PointField.FLOAT32, count=1),
        # ]
        # # Note: create_cloud packs points in given order; keep latest header/frame
        # out_msg = pc2.create_cloud(self.latest_cloud_header, fields, out_iter)
        # self.pub_stiff_map.publish(out_msg)
        # self.get_logger().info(f"[STIFFMAP] Published {count} contacting points with slope={stiff:.6f} to /stiffness_map_points")




    # ------------- finish: fit, join, plot, return, shutdown -------------
    def _finish(self):
        if self.finished:
            return
        self.finished = True

        # Fit force vs distance (distance domain, from continuous buffers)
        slope, intercept, r2, n_used, idx_used, t_all, d_all, f_all = self._fit_force_vs_distance()
        if n_used == 0 or any(map(lambda x: math.isnan(x), [slope, intercept, r2])):
            self.get_logger().warn('[FIT] Not enough valid samples to fit a line.')
        else:
            self.get_logger().info(
                f"[FIT] Linear fit (F = m*x + b): m={slope:.6f}, b={intercept:.6f}, R^2={r2:.4f}, N={n_used}"
            )

        # Align by stamp (exact then nearest) to compute total_deformation = distance + depth
        d_pad, z_depth, f_force = self._build_aligned_samples()
        if d_pad.size > 0:
            total_def = d_pad + z_depth
            self.get_logger().info(
                f"[JOIN] Aligned samples: {d_pad.size}. "
                f"total_def[0..2]={total_def[:3] if total_def.size>=3 else total_def}"
            )
        else:
            self.get_logger().warn("[JOIN] No aligned samples found (check stamped topics & slop).")

        # Plot: distance-domain scatter + regression line (same x-domain as fit)
        # Plot: two views using aligned arrays
        self._plot_force_vs_delta_d(d_pad, z_depth, f_force, with_depth=True)
        self._plot_force_vs_delta_d(d_pad, z_depth, f_force, with_depth=False)
        # Segmented trendlines (if enabled by segment_size)
        self._plot_force_vs_delta_d_segmented(d_pad, z_depth, f_force, with_depth=True,  segment_size=self.segment_size)
        self._plot_force_vs_delta_d_segmented(d_pad, z_depth, f_force, with_depth=False, segment_size=self.segment_size)
        self._plot_segment_slope_stairs(d_pad, z_depth, f_force, with_depth=True,  segment_size=self.segment_size)
        self._plot_segment_slope_stairs(d_pad, z_depth, f_force, with_depth=False, segment_size=self.segment_size)
        # Plot: alignment of the data
        self._plot_alignment_timeseries()
        self._plot_synced_alignment(d_pad, z_depth, f_force)




        # Optional one-line summary CSV
        if self.summary_csv_path:
            try:
                with open(self.summary_csv_path, 'w') as f:
                    f.write('timestamp_iso,slope,intercept,r2,n_used,steps_taken,original_pos\n')
                    f.write(f"{datetime.now().isoformat()},{slope},{intercept},{r2},{n_used},{self.steps_taken},{self.original_pos}\n")
                self.get_logger().info(f"[IO] Summary saved to {self.summary_csv_path}")
            except Exception as e:
                self.get_logger().warn(f"[IO] Could not save summary CSV: {e}")

        # ----- Save numpy bundle (.npz) of raw & aligned data -----
        try:
            npz_name = os.path.join(self.output_dir, f"pad_fit_data_{datetime.now().strftime('%Y%m%d_%H%M%S')}.npz")

            # Pack stamped dicts into arrays for portability
            force_keys = np.array(list(self.force_by_stamp.keys()), dtype=object)
            force_vals = np.array([self.force_by_stamp[k] for k in self.force_by_stamp], dtype=float)

            depth_keys = np.array(list(self.depth_by_stamp.keys()), dtype=object)
            depth_vals = np.array([self.depth_by_stamp[k] for k in self.depth_by_stamp], dtype=float)

            np.savez_compressed(
                npz_name,
                # continuous buffers (node time)
                buf_time=np.asarray(self.buf_time, dtype=float),
                buf_dist=np.asarray(self.buf_dist, dtype=float),
                buf_force=np.asarray(self.buf_force, dtype=float),
                buf_joint=np.asarray(self.buf_joint, dtype=float),
                # stamped caches
                force_keys=force_keys,
                force_vals=force_vals,
                depth_keys=depth_keys,
                depth_vals=depth_vals,
                # aligned arrays (on force stamps)
                aligned_dist=d_pad,
                aligned_depth=z_depth,          # Δdepth (baseline removed)
                aligned_force=f_force,
                # metadata
                target_distance=self.target_distance,
                original_pos=(self.original_pos if self.original_pos is not None else np.nan),
                step_size=self.step_size,
                join_slop_ns=self.join_slop_ns,
                segment_size=self.segment_size
            )
            self.get_logger().info(f"[IO] NPZ saved to {npz_name}")
        except Exception as e:
            self.get_logger().warn(f"[IO] Failed to save NPZ: {e}")


        # Return to original position
        if self.return_to_original_on_finish and self.original_pos is not None:
            target = min(max(self.original_pos, self.closing_pos), self.opening_pos)
            self.get_logger().info(f"[ACTION] Returning gripper to original position: {target:.6f} m")
            ok = self._send_goal_and_wait(target, timeout_sec=5.0)
            if not ok:
                self.get_logger().warn("[ACTION] Return-to-origin timed out / not accepted.")
            self.get_logger().info('[ACTION] Return-to-original complete.')

        try:
            self._publish_stiffness_map(slope)
        except Exception as e:
            self.get_logger().warn(f"[STIFFMAP] Failed to publish stiffness map: {e}")


        # Shutdown
        self.get_logger().info('[EXIT] Shutting down node.')
        try:
            self.destroy_node()
        finally:
            rclpy.shutdown()

    # ------------- building aligned samples -------------
    def _build_aligned_samples(self):
        """Return arrays aligned on FORCE stamps:
       distance (m), Δdepth (m, baseline-corrected), force (units/N)."""
        force_keys = list(self.force_by_stamp.keys())
        if not force_keys:
            return np.array([]), np.array([]), np.array([])

        d_list, z_list, f_list = [], [], []
        depth_keys = set(self.depth_by_stamp.keys())

        # We don't have stamped distance; use nearest available distance *at force time*
        # by reading the closest distance sample from the continuous buffer (approximation).
        # If you can publish a stamped pad-distance, swap this with another cached-by-stamp dict.
        for k in force_keys:
            f = self.force_by_stamp[k]

            # Depth: exact or nearest within slop
            if k in depth_keys:
                z = self.depth_by_stamp[k]
            else:
                k2 = self._find_nearest_within(depth_keys, k, self.join_slop_ns)
                if k2 is None:
                    # no matching depth near this force stamp
                    continue
                z = self.depth_by_stamp[k2]

            # Distance approximation: nearest distance in time-series (latest samples)
            if len(self.buf_time) == 0:
                continue
            t_force = k[0] + k[1]*1e-9
            # find index with nearest time in buf_time
            idx = int(np.argmin(np.abs(np.asarray(self.buf_time) - t_force)))
            d = float(self.buf_dist[idx])

            d_list.append(d); z_list.append(z); f_list.append(f)

        return np.asarray(d_list), np.asarray(z_list), np.asarray(f_list)

    # ------------- regression -------------
    def _fit_force_vs_distance(self):
        import numpy as np

        d_all = np.asarray(self.buf_dist, dtype=float)
        f_all = np.asarray(self.buf_force, dtype=float)
        t_all = np.asarray(self.buf_time, dtype=float)

        # Base finite mask over the full buffers (kept as indices into the original arrays)
        base_mask = np.isfinite(d_all) & np.isfinite(f_all)
        if not np.any(base_mask):
            return float('nan'), float('nan'), float('nan'), 0, np.array([], dtype=int), t_all, d_all, f_all

        idx = np.nonzero(base_mask)[0]
        d = d_all[idx]; f = f_all[idx]

        # Optionally keep only the last N (preserve original indices)
        if self.reg_use_last_n > 0 and len(d) > self.reg_use_last_n:
            idx = idx[-self.reg_use_last_n:]
            d = d_all[idx]; f = f_all[idx]

        # Force range filter
        range_mask = (f >= self.reg_force_min) & (f <= self.reg_force_max)
        idx = idx[range_mask]
        d = d_all[idx]; f = f_all[idx]
        if len(d) < 2:
            return float('nan'), float('nan'), float('nan'), 0, np.array([], dtype=int), t_all, d_all, f_all

        # Initial fit
        A = np.vstack([d, np.ones_like(d)]).T
        m, b = np.linalg.lstsq(A, f, rcond=None)[0]

        # Optional outlier trimming
        if self.reg_outlier_sigma > 0.0 and len(d) >= 5:
            resid = f - (m * d + b)
            sigma = np.std(resid)
            keep = np.abs(resid) <= (self.reg_outlier_sigma * (sigma if sigma > 0 else 1e-12))
            idx = idx[keep]
            d = d_all[idx]; f = f_all[idx]
            if len(d) >= 2:
                A = np.vstack([d, np.ones_like(d)]).T
                m, b = np.linalg.lstsq(A, f, rcond=None)[0]

        f_hat = m * d + b
        ss_res = np.sum((f - f_hat) ** 2)
        ss_tot = np.sum((f - np.mean(f)) ** 2)
        r2 = 1.0 - (ss_res / ss_tot) if ss_tot > 0 else float('nan')

        # Return extra info: the indices used and the original time/dist/force arrays for convenience
        return float(m), float(b), float(r2), int(len(d)), idx.astype(int), t_all, d_all, f_all


    # ------------- plotting -------------
    def _plot_distance_vs_force_with_fit(self, slope: float, intercept: float):
        try:
            import matplotlib.pyplot as plt

            d = np.asarray(self.buf_dist, dtype=float)
            f = np.asarray(self.buf_force, dtype=float)

            mask = np.isfinite(d) & np.isfinite(f)
            if not np.any(mask):
                self.get_logger().warn("[PLOT] No finite samples to plot.")
                return
            d = d[mask]; f = f[mask]

            order = np.argsort(d)
            d = d[order]; f = f[order]

            plt.figure()
            # Data in distance domain; convert x to mm for readability
            plt.scatter(d * 1000, f, s=10, label='Data points')

            if np.isfinite(slope) and np.isfinite(intercept) and len(d) >= 2:
                f_fit = slope * d + intercept
                plt.plot(d * 1000, f_fit, 'r-', label=f'Fit: F = {slope:.2f}·x + {intercept:.2f}')

            plt.xlabel("Pad distance [mm]")  # (absolute, not relative to start)
            plt.ylabel("Force (units/N)")
            plt.title("Force vs. Pad Distance (data + linear fit)")
            plt.legend()
            plt.tight_layout()

            if self.fig_path:
                plt.savefig(self.fig_path, dpi=150)
                self.get_logger().info(f"[PLOT] Figure saved to {self.fig_path}")
            plt.close()
        except Exception as e:
            self.get_logger().warn(f"[PLOT] Failed to generate figure: {e}")
            
    def _plot_force_vs_delta_d(self, d_pad: np.ndarray, z_depth: np.ndarray, f_force: np.ndarray, with_depth: bool):
        """
        Plot F vs Δd, where:
        - with_depth=True:  Δd = (d_start - d_pad) - z_depth
        - with_depth=False: Δd = (d_start - d_pad)
        Uses aligned arrays (d_pad, z_depth, f_force) keyed by FORCE stamps.
        Saves to <fig_path>_delta_with_depth.png or <fig_path>_delta_no_depth.png
        """
        try:
            import os
            import matplotlib.pyplot as plt
            import numpy as np

            # Need aligned arrays (f_force & d_pad at minimum)
            if d_pad.size == 0 or f_force.size == 0:
                self.get_logger().warn("[PLOT] No aligned samples for Δd plot.")
                return

            # Choose starting pad distance (prefer first finite from continuous buffer)
            d_buf = np.asarray(self.buf_dist, dtype=float)
            mask_buf = np.isfinite(d_buf)
            if np.any(mask_buf):
                d_start = float(d_buf[mask_buf][0])
            else:
                mask_aligned = np.isfinite(d_pad)
                if not np.any(mask_aligned):
                    self.get_logger().warn("[PLOT] No finite starting distance available.")
                    return
                d_start = float(d_pad[mask_aligned][0])

            # Compute Δd in meters
            if with_depth:
                if z_depth.size == 0:
                    self.get_logger().warn("[PLOT] No depth available for Δd(with_depth); skipping.")
                    return
                delta_d = (d_start - d_pad) - z_depth
            else:
                delta_d = (d_start - d_pad)

            # Filter finite pairs
            finite = np.isfinite(delta_d) & np.isfinite(f_force)
            if not np.any(finite):
                self.get_logger().warn("[PLOT] No finite Δd/force pairs to plot.")
                return

            x = delta_d[finite]
            y = f_force[finite]

            # Sort by x for a clean line
            order = np.argsort(x)
            x = x[order]; y = y[order]

            # Linear regression: F = m*Δd + b (fit in meters)
            A = np.vstack([x, np.ones_like(x)]).T
            m, b = np.linalg.lstsq(A, y, rcond=None)[0]

            # R²
            y_hat = m * x + b
            ss_res = float(np.sum((y - y_hat) ** 2))
            ss_tot = float(np.sum((y - np.mean(y)) ** 2))
            r2 = 1.0 - (ss_res / ss_tot) if ss_tot > 0 else float('nan')

            # Debug
            tag = "with_depth" if with_depth else "no_depth"
            print(f"[PLOT:{tag}] Δd-fit: m={m:.6f}, b={b:.6f}, R^2={r2:.4f}, N={len(x)}")
            self.get_logger().info(f"[PLOT:{tag}] F = m*Δd + b: m={m:.6f}, b={b:.6f}, R^2={r2:.4f}, N={len(x)}")

            # Build output filename(s)
            base, ext = os.path.splitext(self.fig_path or f"pad_fit_{datetime.now().strftime('%Y%m%d_%H%M%S')}.png")
            out_path = f"{base}_delta_{tag}{ext or '.png'}"

            # Plot (x shown in mm for readability)
            plt.figure()
            plt.scatter(x * 1000.0, y, s=10, label='Data points')
            plt.plot(x * 1000.0, y_hat, 'r-', label=f'Fit: F = {m:.2f}·Δd + {b:.2f}')
            xlabel = "Δd = (d_start - d_pad)" + (" - Δdepth" if with_depth else "") + "  [mm]"
            plt.xlabel(xlabel)
            plt.ylabel("Force (units/N)")
            plt.title("Force vs Δd (data + linear fit)")
            plt.legend()
            plt.tight_layout()
            plt.savefig(out_path, dpi=150)
            self.get_logger().info(f"[PLOT:{tag}] Figure saved to {out_path}")
            plt.close()

        except Exception as e:
            self.get_logger().warn(f"[PLOT] Failed to generate Δd ({'with' if with_depth else 'no'} depth) figure: {e}")

    def _plot_alignment_timeseries(self):
        """
        Plot time alignment of the three streams with timestamp on the x-axis:
        - force (from /feats/force_z Image sums; stamped)
        - distance_between_pad (continuous buffer; node-clock time)
        - gel_pad_depth (PointStamped; stamped)

        Saves to <fig_path>_alignment.png
        """
        try:
            import os
            import numpy as np
            import matplotlib.pyplot as plt

            # ---- collect force (stamped) ----
            if len(self.force_by_stamp) > 0:
                k_force = list(self.force_by_stamp.keys())
                t_force = np.array([k[0] + k[1] * 1e-9 for k in k_force], dtype=float)
                v_force = np.array([self.force_by_stamp[k] for k in k_force], dtype=float)
                order = np.argsort(t_force)
                t_force, v_force = t_force[order], v_force[order]
            else:
                t_force = np.array([], dtype=float)
                v_force = np.array([], dtype=float)

            # ---- collect depth (stamped) ----
            if len(self.depth_by_stamp) > 0:
                k_depth = list(self.depth_by_stamp.keys())
                t_depth = np.array([k[0] + k[1] * 1e-9 for k in k_depth], dtype=float)
                v_depth = np.array([self.depth_by_stamp[k] for k in k_depth], dtype=float)
                order = np.argsort(t_depth)
                t_depth, v_depth = t_depth[order], v_depth[order]
            else:
                t_depth = np.array([], dtype=float)
                v_depth = np.array([], dtype=float)

            # ---- collect distance (node-clock buffered) ----
            t_dist = np.asarray(self.buf_time, dtype=float)
            v_dist = np.asarray(self.buf_dist, dtype=float)

            # ---- compute a common time origin ----
            candidates = []
            if t_force.size: candidates.append(t_force[0])
            if t_depth.size: candidates.append(t_depth[0])
            if t_dist.size:  candidates.append(t_dist[0])
            if len(candidates) == 0:
                self.get_logger().warn("[ALIGN] No data available to plot alignment.")
                return
            t0 = float(min(candidates))

            # relative time (seconds)
            if t_force.size: t_force_rel = t_force - t0
            else:            t_force_rel = t_force
            if t_depth.size: t_depth_rel = t_depth - t0
            else:            t_depth_rel = t_depth
            if t_dist.size:  t_dist_rel  = t_dist  - t0
            else:            t_dist_rel  = t_dist

            # ---- debug counts ----
            self.get_logger().info(
                f"[ALIGN] plotting alignment: "
                f"force_n={t_force.size}, depth_n={t_depth.size}, dist_n={t_dist.size}, t0={t0:.3f}s"
            )
            print(f"[ALIGN] timespans (s): "
                f"force={((t_force_rel[-1]-t_force_rel[0]) if t_force_rel.size>1 else 0):.3f}, "
                f"depth={((t_depth_rel[-1]-t_depth_rel[0]) if t_depth_rel.size>1 else 0):.3f}, "
                f"dist={((t_dist_rel[-1]-t_dist_rel[0]) if t_dist_rel.size>1 else 0):.3f}")

            # ---- make figure with three subplots (shared x) ----
            fig, axs = plt.subplots(3, 1, figsize=(10, 7), sharex=True)

            # Distance
            if t_dist_rel.size:
                axs[0].plot(t_dist_rel, v_dist, label="Pad distance (m)")
            axs[0].set_ylabel("Distance (m)")
            axs[0].grid(True, alpha=0.3)
            axs[0].legend(loc="best")

            # Depth
            if t_depth_rel.size:
                axs[1].plot(t_depth_rel, v_depth, label="GelSight depth (m)")
            axs[1].set_ylabel("ΔDepth (m)")
            axs[1].grid(True, alpha=0.3)
            axs[1].legend(loc="best")

            # Force
            if t_force_rel.size:
                axs[2].plot(t_force_rel, v_force, label="Force (sum of force_z)", zorder=3)
            axs[2].set_ylabel("Force (units/N)")
            axs[2].set_xlabel("Time since start (s)")
            axs[2].grid(True, alpha=0.3)
            axs[2].legend(loc="best")

            plt.tight_layout()

            # save
            base, ext = os.path.splitext(self.fig_path or f"pad_fit_{datetime.now().strftime('%Y%m%d_%H%M%S')}.png")
            out_path = f"{base}_alignment{ext or '.png'}"
            plt.savefig(out_path, dpi=150)
            plt.close()
            self.get_logger().info(f"[ALIGN] Alignment figure saved to {out_path}")

        except Exception as e:
            self.get_logger().warn(f"[ALIGN] Failed to plot alignment: {e}")
    def _plot_force_vs_delta_d_segmented(self, d_pad: np.ndarray, z_depth: np.ndarray,
                                        f_force: np.ndarray, with_depth: bool, segment_size: int):
        """
        Like _plot_force_vs_delta_d, but computes per-segment linear fits of size `segment_size`
        and overlays trendlines for each segment. Also logs the list of slopes.
        """
        try:
            import os
            import numpy as np
            import matplotlib.pyplot as plt

            if segment_size is None or segment_size <= 0:
                self.get_logger().info("[SEG] segment_size<=0 -> segmented plotting disabled.")
                return

            if d_pad.size == 0 or f_force.size == 0:
                self.get_logger().warn("[SEG] No aligned samples for segmented Δd plot.")
                return

            # Starting pad distance
            d_buf = np.asarray(self.buf_dist, dtype=float)
            mask_buf = np.isfinite(d_buf)
            if np.any(mask_buf):
                d_start = float(d_buf[mask_buf][0])
            else:
                mask_aligned = np.isfinite(d_pad)
                if not np.any(mask_aligned):
                    self.get_logger().warn("[SEG] No finite starting distance available.")
                    return
                d_start = float(d_pad[mask_aligned][0])

            # Δd in meters
            if with_depth:
                if z_depth.size == 0:
                    self.get_logger().warn("[SEG] No depth available for Δd(with_depth); skipping.")
                    return
                delta_d = (d_start - d_pad) - z_depth
            else:
                delta_d = (d_start - d_pad)

            x = np.asarray(delta_d, dtype=float)
            y = np.asarray(f_force, dtype=float)

            finite = np.isfinite(x) & np.isfinite(y)
            x = x[finite]; y = y[finite]
            if x.size < 2:
                self.get_logger().warn("[SEG] Not enough finite points for segmented fitting.")
                return

            # Do NOT sort by x; keep acquisition order for segmentation windows.
            n = len(x)
            slopes = []
            seg_ranges = []

            # Prepare plot
            base, ext = os.path.splitext(self.fig_path or f"pad_fit_{datetime.now().strftime('%Y%m%d_%H%M%S')}.png")
            tag = "with_depth" if with_depth else "no_depth"
            out_path = f"{base}_delta_{tag}_seg{segment_size}{ext or '.png'}"

            plt.figure()
            # scatter (x in mm)
            plt.scatter(x * 1000.0, y, s=10, label='Data points')

            # Segment loop
            for start in range(0, n, segment_size):
                end = min(start + segment_size, n)
                if end - start < 2:
                    break

                xs = x[start:end]
                ys = y[start:end]

                # Fit y = m*x + b
                A = np.vstack([xs, np.ones_like(xs)]).T
                m, b = np.linalg.lstsq(A, ys, rcond=None)[0]
                slopes.append(float(m))
                seg_ranges.append((start, end))

                # Draw trendline segment over this window
                # Plot in physical order with solid line; x in mm
                x_line = np.array([xs[0], xs[-1]])
                y_line = m * x_line + b
                plt.plot(x_line * 1000.0, y_line, '-', linewidth=2)

            # Log slopes
            self.get_logger().info(f"[SEG:{tag}] segment_size={segment_size}, segments={len(slopes)}")
            for i, (rng, m) in enumerate(zip(seg_ranges, slopes)):
                self.get_logger().info(f"[SEG:{tag}] seg {i}: idx[{rng[0]}:{rng[1]}] slope={m:.6f}")

            # Axis labels
            xlabel = "Δd = (d_start - d_pad)" + (" - Δdepth" if with_depth else "") + "  [mm]"
            plt.xlabel(xlabel)
            plt.ylabel("Force (units/N)")
            plt.title(f"Force vs Δd — segmented linear fits (size={segment_size})")
            plt.legend()
            plt.tight_layout()
            plt.savefig(out_path, dpi=150)
            plt.close()
            self.get_logger().info(f"[SEG:{tag}] Segmented figure saved to {out_path}")

            # Optionally also save slopes list to a small txt for quick inspection
            try:
                slopes_txt = f"{base}_delta_{tag}_seg{segment_size}_slopes.txt"
                with open(slopes_txt, "w") as f:
                    f.write("# segment_index,start,end,slope\n")
                    for i, ((s, e), m) in enumerate(zip(seg_ranges, slopes)):
                        f.write(f"{i},{s},{e},{m}\n")
                self.get_logger().info(f"[SEG:{tag}] Slopes saved to {slopes_txt}")
            except Exception as e:
                self.get_logger().warn(f"[SEG:{tag}] Failed to save slopes txt: {e}")

        except Exception as e:
            self.get_logger().warn(f"[SEG] Failed segmented Δd plotting: {e}")



    def _plot_segment_slope_stairs(self, d_pad: np.ndarray, z_depth: np.ndarray,
                                f_force: np.ndarray, with_depth: bool, segment_size: int):
        """
        Build a stair-like graph of per-segment slope vs. distance range.
        - Segments are the same (non-overlapping) windows used for segmented fitting.
        - 'Distance' on the x-axis is the Δd domain used for that segmented fit:
            Δd = (d_start - d_pad) - Δdepth  (with_depth=True)
            Δd = (d_start - d_pad)           (with_depth=False)
        - For each segment, we compute slope m and the interval [Δd_min, Δd_max] in that segment,
        then draw a horizontal bar at y=m spanning that interval.
        """
        try:
            import os
            import numpy as np
            import matplotlib.pyplot as plt

            if segment_size is None or segment_size <= 0:
                self.get_logger().info("[STAIRS] segment_size<=0 -> disabled.")
                return
            if d_pad.size == 0 or f_force.size == 0:
                self.get_logger().warn("[STAIRS] No aligned samples.")
                return

            # Starting pad distance (same as other Δd plots)
            d_buf = np.asarray(self.buf_dist, dtype=float)
            mask_buf = np.isfinite(d_buf)
            if np.any(mask_buf):
                d_start = float(d_buf[mask_buf][0])
            else:
                mask_aligned = np.isfinite(d_pad)
                if not np.any(mask_aligned):
                    self.get_logger().warn("[STAIRS] No finite starting distance available.")
                    return
                d_start = float(d_pad[mask_aligned][0])

            # Δd domain (meters)
            if with_depth:
                if z_depth.size == 0:
                    self.get_logger().warn("[STAIRS] No depth available (with_depth=True).")
                    return
                delta_d = (d_start - d_pad) - z_depth
            else:
                delta_d = (d_start - d_pad)

            x = np.asarray(delta_d, dtype=float)
            y = np.asarray(f_force, dtype=float)
            finite = np.isfinite(x) & np.isfinite(y)
            x = x[finite]; y = y[finite]
            n = len(x)
            if n < 2:
                self.get_logger().warn("[STAIRS] Not enough finite points.")
                return

            # Segment in acquisition order (same as segmented plot)
            slopes = []
            intervals = []  # list of (x_left, x_right) in Δd domain
            for start in range(0, n, segment_size):
                end = min(start + segment_size, n)
                if end - start < 2:
                    break
                xs = x[start:end]
                ys = y[start:end]

                # Fit y = m*x + b
                A = np.vstack([xs, np.ones_like(xs)]).T
                m, b = np.linalg.lstsq(A, ys, rcond=None)[0]
                x_left = float(np.nanmin(xs))
                x_right = float(np.nanmax(xs))
                if not (np.isfinite(x_left) and np.isfinite(x_right)):
                    continue

                slopes.append(float(m))
                intervals.append((x_left, x_right))

            if not slopes:
                self.get_logger().warn("[STAIRS] No valid segments produced.")
                return

            # Prepare output filenames
            base, ext = os.path.splitext(self.fig_path or f"pad_fit_{datetime.now().strftime('%Y%m%d_%H%M%S')}.png")
            tag = "with_depth" if with_depth else "no_depth"
            out_png = f"{base}_segstairs_{tag}_seg{segment_size}{ext or '.png'}"
            out_csv = f"{base}_segstairs_{tag}_seg{segment_size}.csv"

            # Make the stair plot (convert x to mm for readability)
            plt.figure(figsize=(9, 4.5))
            for (x_l, x_r), m in zip(intervals, slopes):
                plt.plot([x_l*1000.0, x_r*1000.0], [m, m], linewidth=3)

            # Beautify
            xlabel = "Δd = (d_start - d_pad)" + (" - Δdepth" if with_depth else "") + "  [mm]"
            plt.xlabel(xlabel)
            plt.ylabel("Slope m  [force / meter]")
            plt.title(f"Segment slopes vs. distance (size={segment_size}, {tag})")
            plt.grid(True, alpha=0.3)
            plt.tight_layout()
            plt.savefig(out_png, dpi=150)
            plt.close()
            self.get_logger().info(f"[STAIRS:{tag}] Stair plot saved to {out_png}")

            # Save a CSV summary: one row per segment
            try:
                with open(out_csv, "w") as f:
                    f.write("segment_index,x_left_mm,x_right_mm,slope\n")
                    for i, ((x_l, x_r), m) in enumerate(zip(intervals, slopes)):
                        f.write(f"{i},{x_l*1000.0},{x_r*1000.0},{m}\n")
                self.get_logger().info(f"[STAIRS:{tag}] CSV saved to {out_csv}")
            except Exception as e:
                self.get_logger().warn(f"[STAIRS:{tag}] Failed to save CSV: {e}")

        except Exception as e:
            self.get_logger().warn(f"[STAIRS] Failed to build stair plot: {e}")

    def _plot_synced_alignment(self, d_pad: np.ndarray, z_depth: np.ndarray, f_force: np.ndarray):
        """
        Plot only the synchronized samples used for regression/alignment.
        X-axis is sample index (or relative time if available).
        Each subplot shows force, pad distance, and GelSight depth on the same index.
        Saves to <fig_path>_alignment_synced.png
        """
        try:
            import os
            import numpy as np
            import matplotlib.pyplot as plt

            n = len(f_force)
            if n == 0:
                self.get_logger().warn("[ALIGN:SYNC] No synchronized samples to plot.")
                return

            # Relative sample index (since the timestamps are matched)
            idx = np.arange(n)

            # Optional: scale for easier visualization
            d_mm = d_pad * 1000.0
            z_mm = z_depth * 1000.0

            fig, axs = plt.subplots(3, 1, figsize=(10, 7), sharex=True)

            # Pad distance
            axs[0].plot(idx, d_mm, '-o', label="Pad distance (mm)", markersize=3)
            axs[0].set_ylabel("Distance [mm]")
            axs[0].grid(True, alpha=0.3)
            axs[0].legend(loc="best")

            # Depth
            axs[1].plot(idx, z_mm, '-o', color='tab:green', label="GelSight depth (mm)", markersize=3)
            axs[1].set_ylabel("ΔDepth [mm]")
            axs[1].grid(True, alpha=0.3)
            axs[1].legend(loc="best")

            # Force
            axs[2].plot(idx, f_force, '-o', color='tab:red', label="Force (sum of force_z)", markersize=3)
            axs[2].set_ylabel("Force [units/N]")
            axs[2].set_xlabel("Synchronized sample index")
            axs[2].grid(True, alpha=0.3)
            axs[2].legend(loc="best")

            plt.tight_layout()
            base, ext = os.path.splitext(self.fig_path or f"pad_fit_{datetime.now().strftime('%Y%m%d_%H%M%S')}.png")
            out_path = f"{base}_alignment_synced{ext or '.png'}"
            plt.savefig(out_path, dpi=150)
            plt.close()
            self.get_logger().info(f"[ALIGN:SYNC] Synced alignment figure saved to {out_path}")

            # Small debug summary
            print(f"[ALIGN:SYNC] plotted {n} synchronized samples")

        except Exception as e:
            self.get_logger().warn(f"[ALIGN:SYNC] Failed to plot synced alignment: {e}")

def _pack_rgb(r: int, g: int, b: int) -> float:
    import struct
    return struct.unpack('f', struct.pack('I', (int(r)<<16) | (int(g)<<8) | int(b)))[0]

def _quat_to_R(x, y, z, w):
    xx, yy, zz = x*x, y*y, z*z
    xy, xz, yz = x*y, x*z, y*z
    wx, wy, wz = w*x, w*y, w*z
    import numpy as np
    return np.array([
        [1-2*(yy+zz),   2*(xy-wz),     2*(xz+wy)],
        [  2*(xy+wz), 1-2*(xx+zz),     2*(yz-wx)],
        [  2*(xz-wy),   2*(yz+wx),   1-2*(xx+yy)],
    ], dtype=np.float32)

def _tf_to_RT(t):
    R = _quat_to_R(t.transform.rotation.x, t.transform.rotation.y,
                   t.transform.rotation.z, t.transform.rotation.w)
    import numpy as np
    p = np.array([t.transform.translation.x,
                  t.transform.translation.y,
                  t.transform.translation.z], dtype=np.float32)
    return R, p

def _stiffness_to_rgb(stiff: float) -> float:
    """
    Map stiffness in [-10000, 0] to a blue→red gradient.
      0      -> blue (0,0,255)
      -10000 -> red  (255,0,0)
    Values < -10000 clamp to red; > 0 clamp to blue.
    """
    v = max(-10000.0, min(0.0, float(stiff)))
    t = 1.0 - (v / -10000.0)   # 0 at -10000, 1 at 0
    r = int(255 * (1.0 - t))   # red at -10000 -> 255, at 0 -> 0
    g = 0
    b = int(255 * t)           # blue at 0 -> 255, at -10000 -> 0
    return _pack_rgb(r, g, b)



def main(args=None):
    rclpy.init(args=args)
    node = PadDistanceFitAndSync()
    try:
        rclpy.spin(node)
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        rclpy.shutdown()


if __name__ == '__main__':
    main()

