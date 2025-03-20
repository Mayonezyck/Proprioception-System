from xarm.servo import Servo
from xarm.controller import Controller  # Assuming Controller is defined in controller.py
import time

class ArmController(Controller):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._initPID()

    def _initPID(self):
        self.pid_params = {
            1: {'Kp': 2, 'Ki': 0.6, 'Kd': 0.05},
            2: {'Kp': 1, 'Ki': 0.3, 'Kd': 0.05},
            3: {'Kp': 1.5, 'Ki': 0.6, 'Kd': 0.05},
            4: {'Kp': 1.3, 'Ki': .3, 'Kd': 0.05},
            5: {'Kp': 1, 'Ki': 0.3, 'Kd': 0.05},
            6: {'Kp': 1, 'Ki': 0.3, 'Kd': 0.05}
        }
    def setPositionPID(self, servos, position=None, duration=2000, wait=False):
        """
        Moves servo(s) to a target position using a PID control loop.
        
        This method accepts the same parameters as the regular setPosition method.
        All positions are assumed to be raw values (0 to 1000).
        Instead of a single open-loop command, a PID loop is run by repeatedly:
          - Requesting the current position via CMD_GET_SERVO_POSITION.
          - Computing the error versus the target.
          - Sending a new move command (CMD_SERVO_MOVE) with a short duration.
        
        Parameters:
            servos: int, float, Servo, or a list of Servo or (servo_id, target) pairs.
            position: Target position (required if servos is given as an int/float).
            duration: Maximum total time (in ms) to run the PID loop.
            wait: If True, an extra short wait is performed after completion.
        
        Note:
            - This implementation does not call the parent's setPosition or getPosition.
            - PID gains (Kp, Ki, Kd) and tolerance are fixed here; tune as needed.
        """
        # Convert input into a dictionary mapping servo id -> target raw position,
        # and a list that preserves the order of servo ids.
        servo_targets = {}
        servo_order = []  # Order in which servo ids are processed
        if isinstance(servos, (int, float)):
            if position is None:
                raise ValueError("Parameter 'position' missing.")
            sid = int(servos)
            servo_targets[sid] = int(position)
            servo_order.append(sid)
        elif isinstance(servos, Servo):
            servo_targets[servos.servo_id] = int(servos.position)
            servo_order.append(servos.servo_id)
        elif isinstance(servos, list):
            for item in servos:
                if isinstance(item, Servo):
                    servo_targets[item.servo_id] = int(item.position)
                    servo_order.append(item.servo_id)
                elif isinstance(item, (list, tuple)) and len(item) == 2 and isinstance(item[0], int):
                    servo_targets[item[0]] = int(item[1])
                    servo_order.append(item[0])
                else:
                    raise ValueError("Parameter list 'servos' is not valid.")
        else:
            raise ValueError("Parameter 'servos' is not valid.")
        
        # PID control parameters (tune these as needed)
        Kp = {}
        Ki = {}
        Kd = {}
        for sid in servo_order:
            Kp[sid] = self.pid_params[sid]['Kp']
            Ki[sid] = self.pid_params[sid]['Ki']
            Kd[sid] = self.pid_params[sid]['Kd']
        tolerance = 2  # Acceptable error in raw units
        dt = .1       # Sampling time in seconds
        
        # Initialize PID state for each servo
        integral = {sid: 0 for sid in servo_targets}
        previous_error = {sid: 0 for sid in servo_targets}
        
        max_time = duration / 1000.0  # Convert max duration from ms to seconds
        start_time = time.time()
        
        # Arrays to store time and position readings
        time_readings = []
        position_readings = []

        while True:
            elapsed = time.time() - start_time
            if elapsed > max_time:
                if self.debug:
                    print("PID control timed out after {:.2f} seconds.".format(elapsed))
                break

            # Request current positions for all servos.
            # Construct request data following parent's pattern:
            #   First byte: number of servos, then one byte per servo id.
            req_data = bytearray([len(servo_order)])
            for sid in servo_order:
                req_data.append(sid)
            self._send(self.CMD_GET_SERVO_POSITION, req_data)
            resp = self._recv(self.CMD_GET_SERVO_POSITION)
            if resp is None:
                if self.debug:
                    print("Failed to receive position data; retrying...")
                time.sleep(dt)
                continue

            # Parse response into a dictionary mapping servo id -> current position.
            current_positions = {}
            if len(servo_order) == 1:
                # Single servo: parent's getPosition uses resp[2] and resp[3]
                if len(resp) >= 4:
                    pos = resp[3] * 256 + resp[2]
                    current_positions[servo_order[0]] = pos
                else:
                    if self.debug:
                        print("Invalid response length for single servo.")
                    time.sleep(dt)
                    continue
            else:
                # Multiple servos: assume first byte is count and each servo
                # occupies 3 bytes in the response (with low and high bytes at offsets 2 and 3, etc.)
                count = resp[0]
                for i in range(count):
                    index_low = i * 3 + 2
                    index_high = i * 3 + 3
                    if index_high < len(resp):
                        pos = resp[index_high] * 256 + resp[index_low]
                        current_positions[servo_order[i]] = pos
                    else:
                        if self.debug:
                            print("Invalid response length for servo index", i)
                        continue

            # Record the current time and positions
            time_readings.append(elapsed)
            position_readings.append(current_positions.copy())

            # Compute PID outputs for each servo and check if all errors are within tolerance.
            all_within_tolerance = True
            pid_outputs = {}
            for sid in servo_order:
                target = servo_targets[sid]
                current = current_positions.get(sid)
                if current is None:
                    continue
                error = target - current
                if abs(error) > tolerance:
                    all_within_tolerance = False
                integral[sid] += error * dt
                derivative = (error - previous_error[sid]) / dt
                output = Kp[sid] * error + Ki[sid] * integral[sid] + Kd[sid] * derivative
                # New commanded position is computed as current plus PID output.
                new_command = current + output
                # Clamp the new command within valid raw range.
                new_command = max(0, min(1000, int(round(new_command))))
                pid_outputs[sid] = new_command
                previous_error[sid] = error

            # If all servo errors are within tolerance, exit the loop.
            if all_within_tolerance:
                if self.debug:
                    print("All servos reached target within tolerance.")
                break

            # Send move command with the new positions.
            # Use a short move duration (dt in ms) for incremental adjustments.
            move_duration = int(dt * 1000)
            move_data = bytearray([len(servo_order), move_duration & 0xff, (move_duration >> 8) & 0xff])
            for sid in servo_order:
                pos = pid_outputs.get(sid, 0)
                move_data.extend([sid, pos & 0xff, (pos >> 8) & 0xff])
            self._send(self.CMD_SERVO_MOVE, move_data)

            # Wait for dt seconds before the next iteration.
            time.sleep(dt)
        
        # Issue a final move command to set all servos exactly to their target positions.
        final_data = bytearray([len(servo_order), 0, 0])
        for sid in servo_order:
            target = servo_targets[sid]
            final_data.extend([sid, target & 0xff, (target >> 8) & 0xff])
        self._send(self.CMD_SERVO_MOVE, final_data)
        
        if wait:
            # Allow a brief wait after the final command.
            time.sleep(0.1)

        return time_readings, position_readings
