#######################################
### code written by Ankit Kushwaha ####
#######################################

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
# front_left_led = wb_robot_get_device("front left led")
# front_right_led = wb_robot_get_device("front right led")
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
# motors[4] = {front_left_motor, front_right_motor, rear_left_motor, rear_right_motor}
 

front_left_motor.setPosition(float("inf"))
front_right_motor.setPosition(float("inf"))
rear_left_motor.setPosition(float("inf"))
rear_right_motor.setPosition(float("inf"))

front_left_motor.setVelocity(1.0)
front_right_motor.setVelocity(1.0)
rear_left_motor.setVelocity(1.0)
rear_right_motor.setVelocity(1.0)

# thresold_velocity = 360
# motors_speed = (thresold_velocity,thresold_velocity,thresold_velocity,thresold_velocity)

thrust= 68.5

vertical_Kp = 15
vertical_Ki = 0.08
vertical_Kd = 17

x_Kp = 12
x_Ki = 0.05
x_Kd = 14

y_Kp = 2
y_Ki = 0.05
y_Kd = 2

pitch_Kp = 2
pitch_Ki = 0.1
pitch_Kd = 2
 
roll_Kp = 2
roll_Ki = 0.1
roll_Kd = 2
 
yaw_Kp = 2
yaw_Ki = 0
yaw_Kd = 1

pitch_PID = PID( pitch_Kp , pitch_Ki , pitch_Kd , setpoint=0.0)
roll_PID = PID( roll_Kp, roll_Ki, roll_Kd, setpoint=0.0)
yaw_PID = PID(yaw_Kp, yaw_Ki , yaw_Kd , setpoint=-1)
x_PID = PID(x_Kp, x_Ki, x_Kd, setpoint=0)
y_PID = PID(y_Kp, y_Ki, y_Kd, setpoint=0)
vertical_PID = PID(vertical_Kp, vertical_Ki, vertical_Kd, setpoint=1)

target_X, target_Y, target_Z = 0.0, 0.0, 1.0
X,Y,Z=4.0,13.0,2.0
cnt=0
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
    
    # print(roll)
    # print(pitch)
    # print(yaw)
    # print(roll_acceleration)
    # print(pitch_acceleration)
    xGPS = -gps.getValues()[2]
    yGPS = gps.getValues()[0]
    zGPS = gps.getValues()[1]
    
    x_PID.setpoint = target_X
    y_PID.setpoint = target_Y
    vertical_PID.setpoint = target_Z
    
    # roll_PID.setpoint = target_X
    # pitch_PID.setpoint = target_Y
    yaw_PID.setpoint = yaw
    print("xGPS:{}\t\tyGPS:{}\t\tzGPS:{}".format(-xGPS,-yGPS,zGPS))
    # target_X = X
    # if zGPS >target_Z:
        # target_X = X
    # if xGPS >= X:
        # target_Y = Y
        
    while True:
        if keyboard.is_pressed('shift+up'):
            vertical_dist = 1.0/100
            target_Z+=vertical_dist
            print(zGPS)
            break
        elif keyboard.is_pressed('shift+down'):
            vertical_dist = -1.0/100
            target_Z+=vertical_dist
            print(zGPS)
            break
        elif keyboard.is_pressed('up'):
            pitch_dist = 1.0/100
            target_Y+=pitch_dist
            print(yGPS)
            break
        elif keyboard.is_pressed('left'):
            roll_dist = 1.0/100
            target_X+=roll_dist
            print(xGPS)
            break
        elif keyboard.is_pressed('right'):
            roll_dist = -1.0/100
            target_X+=roll_dist
            break
        elif keyboard.is_pressed('down'):
            pitch_dist = -1.0/100
            target_Y+=pitch_dist
            break
        else:
            roll_dist = 0
            pitch_dist = 0
            vertical_dist = 0
            break
    
    if zGPS >= target_Z:
        cnt=1
        
    
    if cnt==1:
        if target_X < X:
            if xGPS <= X:
                roll_dist = 1.0/100
                target_X+=roll_dist
        elif xGPS >= target_X:
            cnt=2 
    if cnt==2:
        if target_Y < Y:
            if yGPS <= Y:
                pitch_dist = 1.0/100
                target_Y+=pitch_dist
    print(cnt)
        # else:
            # target_X+=X
        
    vertical_input = vertical_PID(zGPS)*0.6+vertical_dist
    print(vertical_input)
    yaw_input = yaw_PID(yaw)
    # roll_input = 10 * roll + roll_acceleration + roll_PID(xGPS+roll_dist)+roll_dist
    roll_input = 10 * roll + roll_acceleration+roll_dist-x_PID(xGPS)
    pitch_input = 10 * pitch - pitch_acceleration + y_PID(-yGPS)+pitch_dist
    # pitch_input = 10 * pitch - pitch_acceleration + pitch_PID(-yGPS+pitch_dist)+pitch_dist
    
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()
    front_left_motor_input = thrust + vertical_input - roll_input - pitch_input + yaw_input
    front_right_motor_input = thrust + vertical_input + roll_input - pitch_input - yaw_input
    rear_left_motor_input = thrust + vertical_input - roll_input + pitch_input - yaw_input
    rear_right_motor_input = thrust + vertical_input + roll_input + pitch_input + yaw_input
    
    
    # print(front_left_motor_input)
    front_left_motor.setVelocity(front_left_motor_input)
    front_right_motor.setVelocity(-front_right_motor_input)
    rear_right_motor.setVelocity(rear_right_motor_input)
    rear_left_motor.setVelocity(-rear_left_motor_input)
    
   
# motors_speed = (front_left_motor_input,front_right_motor_input,rear_left_motor_input,rear_right_motor_input)
# wb_motor_set_velocity(front_left_motor, front_left_motor_input)
# wb_motor_set_velocity(front_right_motor, -front_right_motor_input)
# wb_motor_set_velocity(rear_left_motor, -rear_left_motor_input)
# wb_motor_set_velocity(rear_right_motor, rear_right_motor_input)
# Process sensor data here.
# if __name__ == "__main__":
# robot = Robot()
# run_robot(robot)
