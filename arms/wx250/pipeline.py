import sys
import cv2
import numpy as np
import rclpy
from gsrobotics.gelsight_point import GelsightCloudPublisher, UpdateView
from gsrobotics.config import GSConfig
from interbotix_common_modules.common_robot.robot import robot_shutdown, robot_startup
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS

# --- CONFIG ---
TRIALS = 1
CSV_FILE = "repeatability_log.csv"
GS_CONFIG = "default_config.json"

def run_robot_touch(bot):
    bot.gripper.set_pressure(0.6)
    bot.gripper.grasp(3)
    bot.gripper.release(0.5)

def get_current_pose_rpy(bot):
    T = bot.arm.get_ee_pose()
    x, y, z = T[0:3, 3]
    from tf_transformations import euler_from_matrix
    roll, pitch, yaw = euler_from_matrix(T[0:3, 0:3])
    return x, y, z, roll, pitch, yaw

def main():
    rclpy.init()
    cfg = GSConfig(GS_CONFIG).config
    static_tfm = np.eye(4)
    sensor_mount = np.eye(4)

    node = GelsightCloudPublisher(
        cfg,
        change_threshold=0.02,
        static_transform_matrix=static_tfm,
        sensor_mount_matrix=sensor_mount,
        min_points=50,
        min_depth=0.02,
        point_scale=0.0001,
        require_tf=False,
        std_threshold=0.03,
        contact_pixel_threshold=10,
        translation_move_threshold=1e-4,
        rotation_move_threshold=0.01,
    )

    bot = InterbotixManipulatorXS(
        robot_model='wx250',
        group_name='arm',
        gripper_name='gripper',
    )
    robot_startup()

    window_title = "Gelsight World Cloud"
    prev_depth = None

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0)
            frame = node.cam_stream.update(dt=0)
            if frame is None:
                continue
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

            prev_depth, pts_sensor, contact_mask, flat = UpdateView(
                image=frame,
                cam_stream=node.cam_stream,
                reconstruction=node.reconstruction,
                cmap=node.cmap,
                config=node.config,
                window_title=window_title,
                prev_depth_norm=prev_depth,
                change_threshold=node.change_threshold,
                std_threshold=node.std_threshold,
                contact_pixel_threshold=node.contact_pixel_threshold,
            )
            key = cv2.waitKey(1) & 0xFF

            if key == ord('t'):
                print("Touch sequence triggered!")
                run_robot_touch(bot)
                print("Current pose:", get_current_pose_rpy(bot))

            if key == ord('q') or cv2.getWindowProperty(window_title, cv2.WND_PROP_VISIBLE) < 1:
                break

    except KeyboardInterrupt:
        print("Exiting...")
    finally:
        robot_shutdown()
        node.destroy_node()
        rclpy.shutdown()
        if getattr(node.cam_stream, "camera", None):
            try:
                node.cam_stream.camera.release()
            except Exception:
                pass
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()