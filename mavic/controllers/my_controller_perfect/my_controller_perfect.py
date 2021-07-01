from controller import Robot
from controller import Camera
from controller import Compass
from controller import GPS
from controller import Gyro
from controller import InertialUnit
from controller import Motor
# from controller import Keyboard
from simple_pid import PID
import keyboard
# create the Robot instance.
robot = Robot()

timestep = int(robot.getBasicTimeStep())
time_step = 8
PI = 3.1415926535897932384626433




camera = robot.getDevice("camera")
camera.enable(timestep)

imu = InertialUnit("inertial unit")
imu.enable(timestep)
gps = GPS("gps")
gps.enable(timestep)
compass = Compass("compass")
compass.enable(timestep)
gyro = Gyro("gyro")
gyro.enable(timestep)
camera_roll_motor = robot.getDevice("camera roll")
camera_pitch_motor = robot.getDevice("camera pitch")

# // Get propeller motors and set them to velocity mode.
front_left_motor = robot.getDevice("front left propeller")
front_right_motor = robot.getDevice("front right propeller")
rear_left_motor = robot.getDevice("rear left propeller")
rear_right_motor = robot.getDevice("rear right propeller")
 

front_left_motor.setPosition(float("inf"))
front_right_motor.setPosition(float("inf"))
rear_left_motor.setPosition(float("inf"))
rear_right_motor.setPosition(float("inf"))

front_left_motor.setVelocity(1.0)
front_right_motor.setVelocity(1.0)
rear_left_motor.setVelocity(1.0)
rear_right_motor.setVelocity(1.0)


thrust= 68.5

# X_Kp = 3
# X_Ki = -0.9
# X_Kd = 15

# Y_Kp = 10
# Y_Ki = 0.03
# Y_Kd = 18

Z_Kp = 15
Z_Ki = 0.11
Z_Kd = 7

pitch_Kp = 4
pitch_Ki = 0
pitch_Kd = 2
 
roll_Kp = 2
roll_Ki = 0.03
roll_Kd = 2
 
yaw_Kp = 1
yaw_Ki = 0
yaw_Kd = .6

pitch_PID = PID( pitch_Kp , pitch_Ki , pitch_Kd , setpoint=0.0)
roll_PID = PID( roll_Kp, roll_Ki, roll_Kd, setpoint=0.0)
yaw_PID = PID(yaw_Kp, yaw_Ki , yaw_Kd , setpoint=-1)

# X_PID = PID(X_Kp, X_Ki, X_Kd, setpoint=0)
# Y_PID = PID(Y_Kp, Y_Ki, Y_Kd, setpoint=0)
Z_PID = PID(Z_Kp, Z_Ki, Z_Kd, setpoint=1)

target_X, target_Y, target_Z = 0.0, 0.0, 1.0
# X,Y,Z=2.0,4.0,2.0

# target_Z = 1.0
# Main loop:
# print("enter the cordinate")
# X = int(input("\nenter the x cordinate:"))
# y = int(input("\n enter the y cordinate:"))
# Z = int(input("\n enter the z cordinate:"))
# print("enter the ff cordinate")
while (robot.step(timestep) != -1):
    # Read the sensors:
    roll = imu.getRollPitchYaw()[0] + PI / 2.0
    pitch = imu.getRollPitchYaw()[1]
    yaw = compass.getValues()[0]
    roll_acceleration = gyro.getValues()[0]
    pitch_acceleration = gyro.getValues()[1]
    
    xGPS = gps.getValues()[2]
    yGPS = gps.getValues()[0]
    zGPS = gps.getValues()[1]
    
    
    roll_PID.setpoint = target_X
    pitch_PID.setpoint = target_Y
    # yaw_PID.setpoint = yaw
    
    # X_PID.setpoint = target_X
    # Y_PID.setpoint = target_Y
    Z_PID.setpoint = target_Z
    
    
    print("xGPS:{}\t\tyGPS:{}\t\tzGPS:{}".format(xGPS,yGPS,zGPS))
    target_X = X
    if zGPS >target_Z:
        target_X = X
    if zGPS >target_Z:
        target_Y = Y
    if xGPS >= X:
        target_Y = Y
        
    while True:
        if keyboard.is_pressed('shift+up'):
            Z_dist = 1.0/70
            target_Z+=Z_dist
            print(zGPS)
            break
        elif keyboard.is_pressed('shift+down'):
            Z_dist = -1.0/70
            target_Z+=Z_dist
            print(zGPS)
            break
        elif keyboard.is_pressed('up'):
            pitch_dist = 1.0/60
            target_Y+=pitch_dist
            print(yGPS)
            break
        elif keyboard.is_pressed('left'):
            roll_dist = 1.0/60
            target_X+=roll_dist
            print(xGPS)
            break
        elif keyboard.is_pressed('right'):
            roll_dist = -1.0/60
            target_X+=roll_dist
            break
        elif keyboard.is_pressed('down'):
            pitch_dist = -1.0/50
            target_Y+=pitch_dist
            break
        else:
            roll_dist = 0
            pitch_dist = 0
            Z_dist = 0
            break

    Z_input = Z_PID(zGPS)+Z_dist
    yaw_input = yaw_PID(yaw)
    # roll_input = 10 * roll + roll_acceleration + roll_PID(xGPS+roll_dist)+roll_dist
    # pitch_input = 10 * pitch - pitch_acceleration + pitch_PID(-yGPS+pitch_dist)+pitch_dist
    
    roll_input = 11 * roll + roll_acceleration + roll_PID(xGPS)+roll_dist
    pitch_input = 10 * pitch - pitch_acceleration + pitch_PID(-yGPS)+pitch_dist
  
    front_left_motor_input = thrust + Z_input - roll_input - pitch_input + yaw_input
    front_right_motor_input = thrust + Z_input + roll_input - pitch_input - yaw_input
    rear_left_motor_input = thrust + Z_input - roll_input + pitch_input - yaw_input
    rear_right_motor_input = thrust + Z_input + roll_input + pitch_input + yaw_input
    
    
    # print(front_left_motor_input)
    front_left_motor.setVelocity(front_left_motor_input)
    front_right_motor.setVelocity(-front_right_motor_input)
    rear_left_motor.setVelocity(-rear_left_motor_input)
    rear_right_motor.setVelocity(rear_right_motor_input)
    
    
   
