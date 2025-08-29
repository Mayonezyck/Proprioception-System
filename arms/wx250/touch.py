#!/usr/bin/env python3
import sys
import csv
import numpy as np
import time
from tf_transformations import euler_from_quaternion

from tf_transformations import euler_from_matrix

from interbotix_common_modules.common_robot.robot import robot_shutdown, robot_startup
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS


from interbotix_xs_msgs.srv import TorqueEnable

# --- CONFIG ---
TRIALS = 1
CSV_FILE = "repeatability_log.csv"


def run_bartender_demo(bot):
    """Run one cycle of the bartender motion sequence."""

    
    
    #RIT
    bot.gripper.set_pressure(0.9)
    bot.arm.set_ee_pose_components(x=0.35, y=0, z=0.074, pitch=1.57)
    time.sleep(1)
    bot.gripper.release(3)
    bot.gripper.grasp(3)
    bot.gripper.release(1)
    bot.arm.set_ee_pose_components(x=0.333, y=0, z=0.078, pitch=1.57)
    time.sleep(1)
    bot.gripper.release(3)
    bot.gripper.grasp(3)
    bot.gripper.release(1)
    bot.arm.set_ee_pose_components(x=0.317, y=0, z=0.082, pitch=1.57)
    time.sleep(1)
    bot.gripper.release(3)
    bot.gripper.grasp(3)
    bot.gripper.release(1)

    #TIR
    # bot.arm.set_ee_pose_components(x=0.315, y=0, z=0.07, pitch=1.57)
    # bot.gripper.release(3)
    # bot.gripper.grasp(3)
    # bot.gripper.release(3)
    # bot.arm.set_ee_pose_components(x=0.333, y=0, z=0.07, pitch=1.57)
    # bot.gripper.release(3)
    # bot.gripper.grasp(3)
    # bot.gripper.release(3)
    # bot.arm.set_ee_pose_components(x=0.35, y=0, z=0.07, pitch=1.57)
    # bot.gripper.release(3)
    # bot.gripper.grasp(3)
    # bot.gripper.release(3)
    #bot.gripper.grasp()
    #bot.arm.set_ee_pose_components(x=0.3, z=0.2)
    #bot.arm.set_ee_pose_components(x=0, y=0.3, z=0.2, pitch=1.57)
    #gripper_release(bot)

def get_current_pose_rpy(bot):
    """Get actual EE pose from 4x4 transform matrix as x,y,z,roll,pitch,yaw."""
    T = bot.arm.get_ee_pose()  # 4x4 matrix
    x, y, z = T[0:3, 3]
    roll, pitch, yaw = euler_from_matrix(T[0:3, 0:3])
    return x, y, z, roll, pitch, yaw



def main():
    bot = InterbotixManipulatorXS(
        robot_model='wx250',
        group_name='arm',
        gripper_name='gripper',
    )

    robot_startup()

    if bot.arm.group_info.num_joints < 5:
        bot.get_node().logfatal('This demo requires at least 5 joints!')
        robot_shutdown()
        sys.exit()

    with open(CSV_FILE, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["trial", "x", "y", "z", "roll", "pitch", "yaw"])

        for i in range(TRIALS):
            print(f"=== Trial {i+1}/{TRIALS} ===")
            run_bartender_demo(bot)
            print(get_current_pose_rpy(bot))
            x, y, z, roll, pitch, yaw = get_current_pose_rpy(bot)
            writer.writerow([i+1, x, y, z, roll, pitch, yaw])

    robot_shutdown()
    print(f"Repeatability data saved to {CSV_FILE}")


if __name__ == '__main__':
    main()

