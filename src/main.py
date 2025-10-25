# EXAMPLE CODE USING PID TO TURN AND DRIVE STRAIGHT (INERTIAL SENSOR REQUIRED)
#
# Note this example builds off of the V5GyroExample project
#  https://github.com/NixRobotics/V5GyroExample

# Library imports
from vex import *

brain = Brain()

# DEVICE DECLARATIONS

# declare motors
l1 = Motor(Ports.PORT1, GearSetting.RATIO_18_1, True)
l2 = Motor(Ports.PORT3, GearSetting.RATIO_18_1, True)
left_drive = MotorGroup(l1, l2)
r1 = Motor(Ports.PORT2, GearSetting.RATIO_18_1, False)
r2 = Motor(Ports.PORT4, GearSetting.RATIO_18_1, False)
right_drive = MotorGroup(r1, r2)

inertial = Inertial(Ports.PORT5)

# NOTE: GYRO_SCALE is used to compensate for each inertial sensor's built in error. This will be different for each sensor
#  and must be determined experimentally before use.
# 
#  If GYRO_SCALE_UNKNOWN is set to True then the robot will turn 10 times to make the error more visible. Measure the error
#  and divide by 10 to get the amount the robot actually turns when instructed to turn 360 degrees. This value can then be
#  entered for ACTUAL_ROBOT_FULL_TURN, e.g. if the robot is pointing at 45degress to the right from where it started it says
#  that the robot will turn by 360 + 45/10 = 364.5 degrees when instructed to turn 360 degrees. So we set ACTUAL_ROBOT_FULL_TURN
#  to 364.5 and then set GYRO_SCALE_UNKNOWN to False to indicate we know what the error is.
#
#  The two values GYRO_SCALE_FOR_TURNS and GYRO_SCALE_FOR_READOUT are then used in the code as follows:
#   - GYRO_SCALE_FOR_TURNS is used when we tell the robot to turn. In the example above we want the robot to turn less
#   - GURO_SCALE_FOR_READOUT is used when we read the inertial sensor to display the current heading. In the example above
#     the inertial sensor is returning a value that is too small so we need to multiply by a factor > 1 to get the correct value
#
# IMPORTANT: If the robot does not turn cleanly meaning TURN_CONSTANT needs adjusting, do that first (see NOTE below). If this is
#  the case temporarily set GYRO_SCALE_UNKNOWN to False and ACTUAL_ROBOT_FULL_TURN to 360.0 and come back to this later

ACTUAL_ROBOT_FULL_TURN = 361.0 # (362 for CODE BOT) e.g. if robot actually turns 365 degrees for a 360 rotation enter 365 here
GYRO_SCALE_FOR_TURNS = 360.0 / ACTUAL_ROBOT_FULL_TURN
GYRO_SCALE_FOR_READOUT = ACTUAL_ROBOT_FULL_TURN / 360.0

# NOTE: TIME_FOR_FULL_TURN is how fast the robot can complete one full revolution. It is used to calculate a timeout value to stop
#  the robot in case turn command does not complete, e.g. if blocked against something
TIME_FOR_FULL_TURN = 2.0 # seconds. Set to 2 seconds by default - adjust accordingly based on your robot and turn_velocity() setting

# NOTE: Use ROBOT_INITIALIZED to allow movement. Calibration time is hidden when connected to field, but we need to prevent robot
#  from moving if we just do Program->Run on the controller
ROBOT_INITIALIZED = False

def pre_autonomous():
    # actions to do when the program starts
    global ROBOT_INITIALIZED

    # IMPORTANT: wait for sensors to initialize fully. Always include a small delay when using any sensors. This includes the 3-wire ports
    wait(0.1, SECONDS)

    # calibrate inertial and wait for completion - takes around 2 seconds
    # IMPORTANT: Robot must be stationary on a flat surface while this runs. Do not touch robot during calibration
    if inertial.installed():
        inertial.calibrate()
        while inertial.is_calibrating():
            wait(50, MSEC)

    ROBOT_INITIALIZED = True

# AUTONOMOUS HELPER FUNCTIONS

class GyroHelper:
    # returns the inertial sensor's corrected direction as continuous ROTATION [-inf, +inf]
    # this is the only version of the direction routines that queries the inertial sensor directly
    @staticmethod
    def gyro_rotation():
        return inertial.rotation(DEGREES) * GYRO_SCALE_FOR_READOUT

    # performs modulus operation on the input so that output is in range [0, 360) degrees
    # note that this will lose history on total full revolutions, but useful if we want current HEADING of the robot
    # @param rotation as ROTATION value (either corrected or not - function is agnostic)
    @staticmethod
    def to_heading(rotation):
        return rotation % 360.0

    # performs modulus operation and offset on the input so that output is in range (-180, + 180] degrees
    # note that this will lose history on total full revolutions, but useful if we want current ANGLE of the robot
    # @param rotation as ROTATION value (either corrected or not - function is agnostic)
    @staticmethod
    def to_angle(rotation):
        angle = rotation % 360.0
        if (angle > 180.0): angle -= 360.0
        return angle

    # returns the inertial sensor's corrected direction as HEADING [0, 360) degrees
    @staticmethod
    def gyro_heading():
        return GyroHelper.to_heading(GyroHelper.gyro_rotation())

    # returns the inertial sensor's corrected direction as ANGLE (-180, +180] degrees
    @staticmethod
    def gyro_angle():
        return GyroHelper.to_angle(GyroHelper.gyro_rotation())

    # Calculate a "raw" turn angle to get the robot facing towards a "real" HEADING based on current gyro reading
    #
    # This will return the smallest amount either left or right, ie no turns greater than 180deg. Provide own function if you want to turn
    # longer way around for some reason  e.g. 270degrees left instead of 90degrees right
    #
    # @param Input heading reflects the true HEADING we want the robot to finish at
    # Returns the scaled turn ANGLE with negative value up to -180deg * gyro_scale for left turn and positive value up to +180deg * scale_scale for right turn
    #
    # NOTE: The scaled return value in this case will *not* represent true motion of the robot, but rather the value we want from the gyro to get this motion
    # Therefore, returned value may exceed -180 to +180 degree range necessarily to compensate for a robot that underturns, so we apply the scale factor last,
    # meaning do not apply any additional limit code or bounds checking on the return value
    @staticmethod
    def calc_angle_to_heading(heading):
        # read corrected sensor as HEADING - this should reflect the robot's true HEADING, assuming scale factor is set correctly and sensor has not
        # drifted too much
        current_heading = GyroHelper.gyro_heading()
        # delta_heading will be the difference between the desired (real) heading and current (real) heading
        delta_heading = heading - current_heading
        # ensure result is in range -180deg (left turns) to +180deg (right turns) and finally multiply by scale factor
        delta_angle = GyroHelper.to_angle(delta_heading) * GYRO_SCALE_FOR_TURNS

        # returned value can be fed direcltly to drivetrain.turn_for(), but not drivetrain.turn_to_heading()
        return delta_angle

    # Computes the "raw" rotation value we want the gyro to read for a "real" HEADING
    # @param Input heading reflects the true HEADING we want the robot to finish at
    # Returns a scaled rotation value that can be used with drivetrain.turn_to_rotation()
    @staticmethod
    def calc_rotation_at_heading(heading):
        # First get the robot's total "real" rotation and heading - be careful not to read the inertial sensor twice in the same routine
        # in case it gets updated.
        current_rotation = GyroHelper.gyro_rotation()
        current_heading = GyroHelper.to_heading(current_rotation)

        # Calculate the real heading and angle delta to get to the desired heading
        delta_heading = heading - current_heading
        delta_angle = GyroHelper.to_angle(delta_heading)

        # The new rotation value will be the current + the angle delta * scale factor
        new_rotation = current_rotation + delta_angle
        new_rotation *= GYRO_SCALE_FOR_TURNS

        # Return value can be used with drivetrain.turn_to_rotation() - will not work with drivetrain.turn_to_heading()
        return new_rotation

class SimplePID:
    # Simple PID controller class for demonstration purposes only
    def __init__(self, Kp, Ki, Kd):
        # Constants
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

        # State variables
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_output = 0.0

        # Time parameters
        self.settle_timer_limit = 0.1 # default settle time in seconds
        self.settle_error_threshold = 1.0 # unit agnostic settle threshold
        self.timeout_timer_limit = 10.0 # default timeout in seconds
        self.timestep = 0.01 # approximate timestep in seconds - used to process timeouts. Changing this will scale the K values

        # Timers
        self.settle_timer = self.settle_timer_limit
        self.timeout_timer = self.timeout_timer_limit

        # Output limits and integral windup controls
        self.output_limit = 1.0 # default output limit (actual output will be clamped to -output_limit, +output_limit)
        self.output_ramp_limit = 0.0 # default output ramp limit (max change in output per compute() call). 0.0 = no ramp limit
        self.integral_limit = self.output_limit / self.Kp # default integral limit (integral term will be clamped to -integral_limit, +integral_limit)

        # Output flags
        self.is_timed_out = False
        self.is_settled = False

    # Setter functions
    def set_settle_time(self, time_sec):
        self.settle_timer_limit = time_sec

    def set_timeout(self, time_sec):
        self.timeout_timer_limit = time_sec

    # Our settle threshold will be in our measurement units. If degrees, we want this to be about 0.5degrees or less
    def set_settle_threshold(self, threshold):
        self.settle_error_threshold = threshold

    def set_timestep(self, timestep_sec):
        self.Ki *= timestep_sec / self.timestep
        self.Kd /= timestep_sec / self.timestep
        self.Kp /= timestep_sec / self.timestep
        self.timestep = timestep_sec

    # Output should be normalized to range [-1.0, +1.0]
    def set_output_limit(self, limit):
        self.output_limit = limit
        self.set_integral_limit(self.output_limit / self.Kp)

    # Ramp limit will be based on our normaled output range and timestep
    # So for a timestep of 0.01sec and a ramp limit of 0.1, the output can change by a maximum of 0.1 every 0.01sec
    # If output_limit is set to 1.0 this means it will take at least 0.2sec to go from -1.0 to +1.0 output
    def set_output_ramp_limit(self, ramp_limit):
        self.output_ramp_limit = ramp_limit

    # Integral limit will be based on our units of measurement and take the output_limit into consideration
    # E.g. for a heading controller we will want to limit the output to the range [-output_limit, +output_limit] which will
    # represent how fast we want each side to turn in the default range of [-1.0, +1.0] (1.0 = full power or 100%)
    # If the heading error is large enough we just want the maximum output and do not want the integral term to accumlate
    # The point at which we fall under the output limit will be when we transition from the saturation region to the controlable
    # region. This will typically be around an error value of around 10 degrees but will depend on the Kp value needed, so we
    # can only set this once we have some idea of what Kp will be.
    # Ideally our resulting error should represent some physically meaningful value so error in this case would be in degrees
    # E.g. if we set output_limit to 0.5 (50% power) and Kp to 0.01 we would saturate the output down to an error of 50deg
    #  saturation point = output_limit = Kp * error  => error = output_limit / Kp or 0.5 % / 0.01 Kp = 50 degrees
    # Therefore our integral limit would be set to 50 (degrees) in this case
    def set_integral_limit(self, limit):
        self.integral_limit = limit

    # Getter functions
    def get_is_timed_out(self):
        return self.is_timed_out
    
    def get_is_settled(self):
        return self.is_settled
    
    # Helper functions
    def limit(self, value, limit):
        limited_output = value
        is_limited = False
        if value > limit:
            limited_output = limit
        elif value < -limit:
            limited_output = -limit
        if (limited_output != value): is_limited = True

        return is_limited, limited_output
    
    def ramp_limit(self, value, prev_value, limit):
        ramp_limited_output = value
        is_ramp_limited = False
        if (abs(value - prev_value) > limit):
            if (value > prev_value): ramp_limited_output = prev_value + limit
            else: ramp_limited_output = prev_value - limit
        if (value != ramp_limited_output): is_ramp_limited = True

        return is_ramp_limited, ramp_limited_output

    # Main compute function
    def compute(self, setpoint, measurement):
        if self.is_done(): return 0.0

        error = setpoint - measurement

        # Integral windup control
        # Case 1: only accumulate integral if error is less than saturation limit
        if abs(error) < self.integral_limit:
            self.integral += error
        # Case 2: reset integral if error crosses zero
        if (error > 0.0 and self.prev_error < 0.0) or (error < 0.0 and self.prev_error > 0.0):
            self.integral = 0.0

        derivative = error - self.prev_error

        output = (self.Kp * error) + (self.Ki * self.integral) + (self.Kd * derivative)

        self.prev_error = error

        # Output limiting
        is_output_limited, output = self.limit(output, self.output_limit)
        
        # Output ramp limiting
        is_ramp_limited = False
        if (self.output_ramp_limit > 0.0):
            is_ramp_limited, output = self.ramp_limit(output, self.prev_output, self.output_ramp_limit)

        self.prev_output = output

        # TODO: minimum output for small errors

        # Timeouts and settling
        if abs(error) < self.settle_error_threshold and not is_ramp_limited:
            self.settle_timer -= self.timestep
        else:
            self.settle_time = self.settle_timer_limit
        self.timeout_timer -= self.timestep

        if (self.is_done()): return 0.0

        # print(error, output)

        return output
    
    # Timeout and settle check function
    def is_done(self):
        if self.settle_timer <= 0.0:
            self.is_settled = True
            return True
        
        if self.timeout_timer <= 0.0:
            self.is_timed_out = True
            return True
        
        return False
    
class SimpleDrive:

    class PIDParameters:
        def __init__(self):
            self.Kp = 1.0
            self.Ki = 0.0
            self.Kd = 0.0
            self.max_output = 1.0
            self.max_ramp = 1.0
            self.settle_error = 1.0

    # Simple drivetrain wrapper class for demonstration purposes only
    def __init__(self, left_motors, right_motors):
        self.turn_pid_constants = SimpleDrive.PIDParameters()

        self.left_motors = left_motors
        self.right_motors = right_motors

        self.stop_mode = BrakeType.COAST

    def set_drive_velocity(self, velocity, unit):
        pass

    def set_drive_acceleration(self, acceleration, unit):
        pass

    def set_turn_acceleration(self, acceleration, unit):
        self.turn_pid_constants.max_ramp = acceleration / 100.0

    def set_turn_velocity(self, velocity, unit):
        self.turn_pid_constants.max_output = velocity / 100.0

    def set_turn_constants(self, Kp, Ki, Kd, settle_error):
        self.turn_pid_constants.Kp = Kp
        self.turn_pid_constants.Ki = Ki
        self.turn_pid_constants.Kd = Kd
        self.turn_pid_constants.settle_error = settle_error

    def set_drive_constants(self, constant):
        # placeholder for setting turn constant if needed
        pass

    def set_timeout(self, time, unit):
        # placeholder for setting timeout if needed
        pass

    def set_stopping(self, mode):
        self.stop_mode = mode

    def turn_for(self, direction, angle, unit):
        turn_pid = SimplePID(self.turn_pid_constants.Kp, self.turn_pid_constants.Ki, self.turn_pid_constants.Kd)
        turn_pid.set_output_limit(self.turn_pid_constants.max_output) # limit output to 50% power
        turn_pid.set_settle_threshold(self.turn_pid_constants.settle_error) # settle threshold in degrees
        start_rotation = GyroHelper.gyro_rotation()
        target_rotation = start_rotation + (angle if direction == TurnType.RIGHT else -angle)
        while not turn_pid.is_done():
            current_rotation = GyroHelper.gyro_rotation()
            pid_output = turn_pid.compute(target_rotation, current_rotation)

            drive_voltage = pid_output * 11.5 # scale to voltage
            if (drive_voltage > 12.0): drive_voltage = 12.0
            if (drive_voltage < -12.0): drive_voltage = -12.0

            self.left_motors.spin(FORWARD, drive_voltage, VOLT)
            self.right_motors.spin(FORWARD, -drive_voltage, VOLT)
            wait(turn_pid.timestep, SECONDS)

        self.stop(self.stop_mode)
        print("Done Turn: ", turn_pid.get_is_settled(), turn_pid.get_is_timed_out())

    def stop(self, mode):
        self.left_motors.stop(mode)
        self.right_motors.stop(mode)
        
def autonomous():
    # wait for initialization to complete
    while not ROBOT_INITIALIZED:
        wait(10, MSEC)

    # place automonous code here
    pass

def user_control():
    brain.screen.clear_screen()
    brain.screen.print("Waiting for robot to initialize fully ... ")
    brain.screen.next_row()

    # wait for initialization to complete
    while not ROBOT_INITIALIZED:
        wait(10, MSEC)

    brain.screen.print("done")
    brain.screen.next_row()

    # Good idea to check if inertial sensor is present before using it as unexpected motion can occur
    if not inertial.installed():
        brain.screen.print("NO INERTAIL SENSOR")
        while True:
            wait(20, MSEC)

    # place user control code here
    drive_train = SimpleDrive(left_drive, right_drive)
    drive_train.set_turn_constants(Kp=0.02, Ki=0.0, Kd=0.0, settle_error=0.5)
    drive_train.set_turn_velocity(50, PERCENT)
    drive_train.turn_for(RIGHT, 90, DEGREES)

    # place driver control in this while loop
    while True:
        wait(20, MSEC)

# create competition instance
comp = Competition(user_control, autonomous)
pre_autonomous()
