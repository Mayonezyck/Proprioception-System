import xarm
# arm is the first xArm detected which is connected to USB
import matplotlib.pyplot as plt
from armcontroller import ArmController
class Arm:
    def __init__(self):
        self.arm = ArmController('USB')
        self.servo1 = xarm.Servo(1)
        self.servo2 = xarm.Servo(2)
        self.servo3 = xarm.Servo(3)
        self.servo4 = xarm.Servo(4)
        self.servo5 = xarm.Servo(5)
        self.servo6 = xarm.Servo(6)
        self.joint_reading = [0,0,0,0,0,0]
        self.readPosition()
        print(self.joint_reading)
        self.homeall()
        print(self.joint_reading)

    def homeall(self):
        self.arm.setPosition([[1, 500], [2, 500], [3, 500], [4,500], [5,500], [6,500]],  2000, wait=True)
        self.readPosition()
        return 1
    
    def getBatteryVoltage(self):
        return self.arm.getBatteryVoltage() 

    def move(self, x, y, z):
        self.arm.move(x, y, z)

    def readPosition(self):
        for i in range(1, 7):
            self.joint_reading[i-1] = self.arm.getPosition(i)
        
    def setThisPositionPID(self, servo, target, duration=1000, Kp=0.5, Ki=0.1, Kd=0.05, tolerance=2.0, wait=True):
        self.arm.setPosition(servo, target, duration, wait)
        [th,ph] = self.arm.setPositionPID(servo, target, duration, wait =False)
        self._plot_position_curve(th, ph, target)

    def close(self):
        self.arm.servoOff([1, 2, 3, 4, 5, 6])

    
    def _plot_position_curve(self, time_data, position_data, target):
        """
        Plot the target position and actual position over time.
        
        Parameters:
        time_data: List of time points.
        position_data: List of dictionaries with actual positions corresponding to the time points.
        target: The target position.
        """
        # Extract actual positions from the dictionary
        actual_positions = [list(pos.values())[0] for pos in position_data]
        
        plt.figure()
        plt.plot(time_data, actual_positions, label='Actual Position')
        plt.axhline(y=target, color='r', linestyle='--', label='Target Position')
        plt.xlabel('Time (s)')
        plt.ylabel('Position')
        plt.title('Servo Position Over Time')
        plt.legend()
        plt.show()