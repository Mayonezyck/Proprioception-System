import xarm
# arm is the first xArm detected which is connected to USB
import matplotlib.pyplot as plt
from arm import Arm

if __name__ == '__main__':
    arm = Arm()
    print(arm.getBatteryVoltage())
    print(arm.joint_reading)
    #arm.setPositionClosedLoop(1, 300, tolerance=2.0, initial_duration=1000, fine_duration=200, poll_interval=0.1, timeout=5, degrees=True)
    arm.setThisPositionPID(1, 400, Kp=1.56, Ki=0.3, Kd=0.08, tolerance=2.0)
    arm.setThisPositionPID(3, 850, Kp=1.56, Ki=0.3, Kd=0.08, tolerance=2.0)
    arm.setThisPositionPID(4, 230, Kp=1.56, Ki=0.3, Kd=0.08, tolerance=2.0)
    arm.setThisPositionPID(1, 600, Kp=1.56, Ki=0.3, Kd=0.08, tolerance=2.0)

    #arm.setThisPositionPID(6, 300, Kp=1.56, Ki=0.3, Kd=0.08, tolerance=2.0)
    #arm.move(200, 200, 200)
    arm.close()