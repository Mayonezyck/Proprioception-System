#!/usr/bin/env python3

import cv2
import numpy as np
import open3d as o3d
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from std_msgs.msg import Header

from config import GSConfig
from utilities.reconstruction import Reconstruction3D
from utilities.visualization import Visualize3D
from utilities.gelsightmini import GelSightMini
from utilities.image_processing import (
    stack_label_above_image,
    apply_cmap,
    color_map_from_txt,
    normalize_array,
    trim_outliers,
)
from utilities.logger import log_message

import tf2_ros
from tf_transformations import translation_matrix, quaternion_matrix
import time

def dbg(msg):
    print(f"[{time.strftime('%H:%M:%S')}] {msg}")


def UpdateView(
    image: np.ndarray,
    cam_stream: GelSightMini,
    reconstruction: Reconstruction3D,
    visualizer3D: Visualize3D,
    cmap: np.ndarray,
    config,
    window_title: str,
    prev_depth_norm: np.ndarray = None,
    change_threshold: float = 0.02,
):
    """
    Computes depth and identifies changed pixels based on threshold.
    Returns (normalized depth, pts_sensor).
    """
    depth_map, contact_mask, grad_x, grad_y = reconstruction.get_depthmap(
        image=image,
        markers_threshold=(config.marker_mask_min, config.marker_mask_max),
    )
    if np.isnan(depth_map).any():
        return prev_depth_norm, np.empty((0, 3), dtype=float)

    # Trim outliers and normalize
    depth_trimmed = trim_outliers(depth_map, 1, 99)
    depth_norm = normalize_array(array=depth_trimmed, min_divider=10)

    # Detect changed pixels
    if prev_depth_norm is None:
        changed_mask = np.zeros_like(depth_norm, dtype=bool)
    else:
        diff = np.abs(depth_norm - prev_depth_norm)
        changed_mask = diff > change_threshold

    ys, xs = np.nonzero(changed_mask)
    zs = depth_map[ys, xs]
    pts_sensor = np.stack([xs.astype(float), ys.astype(float), zs], axis=1)

    # Visualization panes
    depth_rgb = apply_cmap(data=depth_norm, cmap=cmap)
    contact_gray = (contact_mask * 255).astype(np.uint8)
    contact_rgb = cv2.cvtColor(contact_gray, cv2.COLOR_GRAY2BGR)
    change_gray = (changed_mask.astype(np.uint8) * 255)
    change_rgb = cv2.cvtColor(change_gray, cv2.COLOR_GRAY2BGR)

    frame_labeled = stack_label_above_image(
        image, f"Camera Feed {int(cam_stream.fps)} FPS", 30
    )
    contact_labeled = stack_label_above_image(contact_rgb, "Contact Mask", 30)
    depth_labeled = stack_label_above_image(depth_rgb, "Depth", 30)
    change_labeled = stack_label_above_image(change_rgb, "Changed Area", 30)

    spacer = np.zeros((frame_labeled.shape[0], 30, 3), dtype=np.uint8)
    top_row = np.hstack([
        frame_labeled, spacer,
        contact_labeled, spacer,
        depth_labeled, spacer,
        change_labeled,
    ])
    display = cv2.resize(
        top_row,
        (
            int(top_row.shape[1] * config.cv_image_stack_scale),
            int(top_row.shape[0] * config.cv_image_stack_scale),
        ),
        interpolation=cv2.INTER_NEAREST,
    )
    cv2.imshow(window_title, display.astype(np.uint8))

    return depth_norm, pts_sensor


class GelsightCloudPublisher(Node):
    def __init__(self, config, change_threshold, static_transform_matrix,
                 min_points, min_depth, require_tf=False):
        super().__init__('gelsight_cloud_publisher')
        self.config = config
        self.change_threshold = change_threshold
        self.static_transform_matrix = static_transform_matrix
        self.min_points = min_points
        self.min_depth = min_depth
        self.require_tf = require_tf

        # TF2 setup
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.source_frame = "wx250/ee_gripper_link"  # sensor frame
        self.target_frame = "base_link"  # published frame / world base

        # ROS2 publisher
        self.pub = self.create_publisher(PointCloud2, 'world_cloud', 10)

        # Reconstruction network
        self.reconstruction = Reconstruction3D(
            image_width=config.camera_width,
            image_height=config.camera_height,
            use_gpu=config.use_gpu,
        )
        if self.reconstruction.load_nn(config.nn_model_path) is None:
            log_message("Failed to load model.")
            self.failed_init = True
            return
        self.failed_init = False

        # Open3D visualizer
        self.visualizer3D = None
        if config.pointcloud_enabled:
            self.visualizer3D = Visualize3D(
                pointcloud_size_x=config.camera_width,
                pointcloud_size_y=config.camera_height,
                save_path="",
                window_width=int(config.pointcloud_window_scale * config.camera_width),
                window_height=int(config.pointcloud_window_scale * config.camera_height),
            )

        # Persistent world cloud (in target_frame)
        self.world_cloud = np.zeros((0, 3), dtype=float)
        self.world_pcd = o3d.geometry.PointCloud()

        # Colormap
        self.cmap = color_map_from_txt(
            path=config.cmap_txt_path,
            is_bgr=config.cmap_in_BGR_format,
        )

        # Camera stream
        self.cam_stream = GelSightMini(
            target_width=config.camera_width,
            target_height=config.camera_height,
        )
        self.cam_stream.select_device(config.default_camera_index)
        self.cam_stream.start()

    def run(self):
        if getattr(self, "failed_init", False):
            dbg("[ERROR] Initialization failed; exiting run loop.")
            return

        window_title = f"World Cloud (th={self.change_threshold}, dmin={self.min_depth})"
        prev_depth = None

        try:
            while rclpy.ok():
            	dbg("[info] i'm here guys'")
            	
                frame = self.cam_stream.update(dt=0)
                if frame is None:
                    continue
                frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
		dbg('rclpyok')
                prev_depth, pts_sensor = UpdateView(
                    image=frame,
                    cam_stream=self.cam_stream,
                    reconstruction=self.reconstruction,
                    visualizer3D=self.visualizer3D,
                    cmap=self.cmap,
                    config=self.config,
                    window_title=window_title,
                    prev_depth_norm=prev_depth,
                    change_threshold=self.change_threshold,
                )

                key = cv2.waitKey(1) & 0xFF

                # Clear on 'z'
                if key == ord('z'):
                    self.world_cloud = np.zeros((0, 3), dtype=float)
                    self.world_pcd.clear()
                    if self.visualizer3D:
                        vis = self.visualizer3D.visualizer
                        vis.clear_geometries()
                    continue

                # Filter by depth
                if pts_sensor.size > 0:
                    mask = pts_sensor[:, 2] >= self.min_depth
                    pts_sensor = pts_sensor[mask]

                # Accumulate and publish
                if pts_sensor.shape[0] >= self.min_points and self.visualizer3D:
                    print('here')
                    homo = np.hstack([pts_sensor, np.ones((pts_sensor.shape[0], 1))])
                    use_world_pts = None
                    used_tf = False
                    try:
                        tf_stamped = self.tf_buffer.lookup_transform(
                            self.target_frame,
                            self.source_frame,
                            Time(),
                            timeout=Duration(seconds=0.1),
                        )
                        trans = tf_stamped.transform.translation
                        rot = tf_stamped.transform.rotation
                        T = translation_matrix([trans.x, trans.y, trans.z]) @ quaternion_matrix([rot.w, rot.x, rot.y, rot.z])
                        world_pts = (T @ homo.T).T[:, :3]
                        use_world_pts = world_pts
                        used_tf = True
                        print(f"[TF SUCCESS] translation=({trans.x:.3f}, {trans.y:.3f}, {trans.z:.3f}), rotation=({rot.w:.3f}, {rot.x:.3f}, {rot.y:.3f}, {rot.z:.3f})")
                    except Exception as e:
                        dbg(f"[TF FAIL] {e}")
                        if self.static_transform_matrix is not None:
                            world_pts = (self.static_transform_matrix @ homo.T).T[:, :3]
                            use_world_pts = world_pts
                            print("[FALLBACK] using static transform")
                        else:
                            use_world_pts = pts_sensor
                            print("[FALLBACK] using raw sensor points (no transform)")

                    if self.require_tf and not used_tf:
                        print("[ERROR] TF required but unavailable; skipping this batch of points.")
                        continue

                    self.world_cloud = np.vstack([self.world_cloud, use_world_pts])
                    self.world_pcd.points = o3d.utility.Vector3dVector(self.world_cloud)

                    # Open3D render
                    vis = self.visualizer3D.visualizer
                    vis.clear_geometries()
                    vis.add_geometry(self.world_pcd)
                    vis.poll_events()
                    vis.update_renderer()

                    # Publish to ROS2
                    hdr = Header()
                    hdr.stamp = self.get_clock().now().to_msg()
                    hdr.frame_id = self.target_frame
                    cloud_msg = pc2.create_cloud_xyz32(
                        hdr,
                        self.world_cloud.tolist()
                    )
                    self.pub.publish(cloud_msg)
                    dbg(f"[INFO] Published {self.world_cloud.shape[0]} what thefuck (used_tf={used_tf})")
                    dbg(f"[INFO] Published {self.world_cloud.shape[0]} points (used_tf={used_tf})")

                # Exit on 'q'
                if key == ord('q') or cv2.getWindowProperty(window_title, cv2.WND_PROP_VISIBLE) < 1:
                    break

        except KeyboardInterrupt:
            log_message("Exiting...")
        finally:
            if self.cam_stream.camera:
                self.cam_stream.camera.release()
            cv2.destroyAllWindows()
            if self.visualizer3D:
                self.visualizer3D.visualizer.destroy_window()


def main():
    rclpy.init()
    parser = __import__('argparse').ArgumentParser(
        description="Gelsight world cloud ROS2 publisher"
    )
    parser.add_argument("--gs-config", type=str, default=None)
    parser.add_argument("--change-threshold", type=float, default=0.02)
    parser.add_argument(
        "--transform-matrix", type=float, nargs=16,
        default=[1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1]
    )
    parser.add_argument("--min-points", type=int, default=50)
    parser.add_argument("--min-depth", type=float, default=0.0)
    parser.add_argument("--require-tf", action='store_true', help="If set, skip points when TF lookup fails.")
    args = parser.parse_args()

    cfg = GSConfig(args.gs_config or "default_config.json").config
    static_tfm = np.array(args.transform_matrix, dtype=float).reshape((4, 4))

    node = GelsightCloudPublisher(cfg, args.change_threshold, static_tfm,
                                  args.min_points, args.min_depth, require_tf=args.require_tf)
    node.run()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

