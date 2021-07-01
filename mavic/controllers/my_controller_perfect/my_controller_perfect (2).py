# code by Purushotam kumar agrawal ---> { git - PURU2411 }

from controller import Robot
from controller import Camera
from controller import Compass
from controller import GPS
from controller import Gyro
from controller import InertialUnit
from controller import Motor
import numpy as np
# from controller import Keyboard
from simple_pid import PID
import keyboard


class drone():
    def __init__(self):
        self.current_angle = [0.0, 0.0, 0.0]  # [roll, pitch, yaw]
        self.current_location = [0.0, 0.0, 0.0] #[GPSx, GPSy, GPSz]
        self.acceleration = [0.0, 0.0] #[roll, pitch]
        
        # declaring and intialzing fianl setpoint [x, y, z] 
        self.setpoint_location = [0.0, 0.0, 0.0]

        # declaring and intialzing setpoint abgles [roll, pitch, yaw] 
        self.setpoint_angle = [0.0, 0.0, 0.0]


        # error cointainers of position [x, y, z]
        self.position_error = [0.0, 0.0, 0.0]
        self.position_sum_error = [0.0, 0.0, 0.0]
        self.position_changeIn_error = [0.0, 0.0, 0.0]
        self.position_prev_error = [0.0, 0.0, 0.0]

        # error cointainers of velocity [roll, pitch, yaw]
        self.angle_error = [0.0, 0.0, 0.0]
        self.angle_sum_error = [0.0, 0.0, 0.0]
        self.angle_changeIn_error = [0.0, 0.0, 0.0]
        self.angle_prev_error = [0.0, 0.0, 0.0]

        # PID values of angle [roll, pitch, yaw]
        self.kp_A = [0.8,.7,.1]
        self.ki_A = [0, 0.0, 0.0]
        self.kd_A = [0.1, .1, .1]
        
        # PID values of position[x, y, z]
        self.kp_P = [0.0, 0.0, 2]
        self.ki_P = [0.0, 0.0, 0.005]
        self.kd_P = [0.0, 0.0, 3]


    def set_drone_parameter(self, robot):
        
        # // Get propeller motors and set them to velocity mode.
        front_left_motor = robot.getDevice("front left propeller")
        front_right_motor = robot.getDevice("front right propeller")
        rear_left_motor = robot.getDevice("rear left propeller")
        rear_right_motor = robot.getDevice("rear right propeller")
        self.motors = [front_left_motor, front_right_motor, rear_left_motor, rear_right_motor]

        self.motors[0].setPosition(float("inf"))
        self.motors[1].setPosition(float("inf"))
        self.motors[2].setPosition(float("inf"))
        self.motors[3].setPosition(float("inf"))

        self.motors[0].setVelocity(0.0)
        self.motors[1].setVelocity(0.0)
        self.motors[2].setVelocity(0.0)
        self.motors[3].setVelocity(0.0)

        self.timestep = int(robot.getBasicTimeStep())
        self.time_step = 8
        self.PI = 3.1415926535897932384626433

        # declaring and intialzing IMU
        self.imu = InertialUnit("inertial unit")
        self.imu.enable(self.timestep)

        # declaring and intialzing GPS
        self.gps = GPS("gps")
        self.gps.enable(self.timestep)

        # declaring and intialzing compass
        self.compass = Compass("compass")
        self.compass.enable(self.timestep)

        # declaring and intialzing gyro
        self.gyro = Gyro("gyro")
        self.gyro.enable(self.timestep)

        # declaring and intialzing camera
        self.camera = robot.getDevice("camera")
        self.camera.enable(self.timestep)
        self.camera_roll_motor = robot.getDevice("camera roll")
        self.camera_pitch_motor = robot.getDevice("camera pitch")

        # LED on/off
        self.front_left_led = robot.getDevice("front left led")
        self.front_right_led = robot.getDevice("front right led")


    def read_sensors(self):
        # Read the sensors:
        self.current_angle[0] = self.imu.getRollPitchYaw()[0] + np.pi / 2.0
        self.current_angle[1] = self.imu.getRollPitchYaw()[1]
        self.current_angle[2] = self.compass.getValues()[0]
        
        self.current_location[0] = self.gps.getValues()[2]
        self.current_location[1] = self.gps.getValues()[0]
        self.current_location[2] = self.gps.getValues()[1]

        self.acceleration[0] = self.gyro.getValues()[0]
        self.acceleration[1] = self.gyro.getValues()[1]
        

    def PID_loop(self):
        self.read_sensors()

        for i in range(3):
            self.position_error[i] =   self.setpoint_location[i] - self.current_location[i]
            self.position_sum_error[i] = self.position_sum_error[i] + self.position_error[i]
            self.position_changeIn_error[i] = self.position_error[i] - self.position_prev_error[i]
            self.position_prev_error[i] = self.position_error[i]

        # assuming pitch -> x direction
        output1 = self.kp_P[0]*self.position_error[0] + self.ki_P[0]*self.position_sum_error[0] + self.kd_P[0]*self.position_changeIn_error[0]
        # assuming roll -> y direction
        output2 = self.kp_P[1]*self.position_error[1] + self.ki_P[1]*self.position_sum_error[1] + self.kd_P[1]*self.position_changeIn_error[1]
        # throttle -> z direction
        throttle = self.kp_P[2]*self.position_error[2] + self.ki_P[2]*self.position_sum_error[2] + self.kd_P[2]*self.position_changeIn_error[2]

        self.setpoint_angle = [output2, output1, -1]


        for i in range(3):
            self.angle_error[i] = self.current_angle[i] - self.setpoint_angle[i]
            self.angle_sum_error[i] = self.angle_sum_error[i] + self.angle_error[i]
            self.angle_changeIn_error[i] = self.angle_error[i] - self.angle_prev_error[i]
            self.angle_prev_error[i] = self.angle_error[i]

        # roll
        roll = self.kp_A[0]*self.angle_error[0] + self.ki_A[0]*self.angle_sum_error[0] + self.kd_A[0]*self.angle_changeIn_error[0]
        # pitch
        pitch = self.kp_A[1]*self.angle_error[1] + self.ki_A[1]*self.angle_sum_error[1] + self.kd_A[1]*self.angle_changeIn_error[1]
        # yaw
        yaw = self.kp_A[2]*self.angle_error[2] + self.ki_A[2]*self.angle_sum_error[2] + self.kd_A[2]*self.angle_changeIn_error[2]

        M0 = 68.5 + throttle - roll - pitch + yaw
        M1 = 68.5 + throttle + roll - pitch - yaw
        M2 = 68.5 + throttle - roll + pitch - yaw
        M3 = 68.5 + throttle + roll + pitch + yaw

        print(throttle)
        print(M0)
        print(M1)
        print(M2)
        print(M3)
        self.motors[0].setVelocity(M0)
        self.motors[1].setVelocity(-M1)
        self.motors[2].setVelocity(-M2)
        self.motors[3].setVelocity(M3)




if __name__ == '__main__':

    # create the Robot instance.
    robot = Robot()

    # thrust= 68.5

    we_drone = drone()
    we_drone.set_drone_parameter(robot)

    we_drone.setpoint_location = [0.0, 0.0, 1.0]

    while robot.step(we_drone.timestep) != -1:
        we_drone.PID_loop()
        # we_drone.motors[0].setVelocity(70.0)
        # we_drone.motors[1].setVelocity(-70.0)
        # we_drone.motors[2].setVelocity(-70.0)
        # we_drone.motors[3].setVelocity(70.0)

