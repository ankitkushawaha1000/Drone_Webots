#######################################
### code written by Ankit Kushwaha ####
#######################################
# https://github.com/ankitkushawaha1000
#######################################
### importing required modules     ####
#######################################
from controller import Robot
from controller import Camera
from controller import Compass
from controller import GPS
from controller import Gyro
from controller import InertialUnit
from controller import Motor
from simple_pid import PID
import keyboard
import numpy as np
import imutils
import cv2
import cv2.aruco as aruco
  
# create the Robot instance.
robot = Robot()
 
timestep = int(robot.getBasicTimeStep())
time_step = 8
PI = 3.1415926535897932384626433

imu = InertialUnit("inertial unit")
imu.enable(timestep)
gps = GPS("gps")
gps.enable(timestep)
compass = Compass("compass")
compass.enable(timestep)
gyro = Gyro("gyro")
gyro.enable(timestep)
#camera input
camera = robot.getDevice("camera")
camera.enable(timestep)
camera.recognitionEnable(timestep)
camera_roll_motor = robot.getDevice("camera roll")
camera_pitch_motor = robot.getDevice("camera pitch")

# get propeller motors and set them to velocity mode.
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


#######################################
###            pid values          ####
#######################################
thrust= 68.5

vertical_Kp = 15
vertical_Ki = 0.2
vertical_Kd = 11

x_Kp = 9
x_Ki = 0
x_Kd = 7

y_Kp = 7
y_Ki = 0
y_Kd = 4

yaw_Kp = 2
yaw_Ki = 0
yaw_Kd = 1

yaw_PID = PID(yaw_Kp, yaw_Ki , yaw_Kd , setpoint=-1)
x_PID = PID(x_Kp, x_Ki, x_Kd, setpoint=0)
y_PID = PID(y_Kp, y_Ki, y_Kd, setpoint=0)
vertical_PID = PID(vertical_Kp, vertical_Ki, vertical_Kd, setpoint=0)

#######################################
###          target values         ####
#######################################
target_X, target_Y, target_Z = 0.0, 0.0, 1.0
X,Y,Z=0.0,0.0,0.0
cnt=0
Xaruco = 99
Yaruco = 99

#######################################
###            main loop           ####
#######################################

while (robot.step(timestep) != -1):
    # Read the sensors:
    roll = imu.getRollPitchYaw()[0] + PI / 2.0
    pitch = imu.getRollPitchYaw()[1]
    yaw = compass.getValues()[0]
    roll_acceleration = gyro.getValues()[0]
    pitch_acceleration = gyro.getValues()[1]
    
    xGPS = -gps.getValues()[2]
    yGPS = -gps.getValues()[0]
    zGPS = gps.getValues()[1]
    
###########################################################################################################
###   press s for image processing, it updates the target position with the position of aruco's center ####
###########################################################################################################
    if keyboard.is_pressed('s'):
        image = np.array(camera.getImageArray())
        image = image.astype(float)
        cv2.imwrite('camera_image.jpg',image)
        cap = cv2.imread('camera_image.jpg')
        gray = cv2.cvtColor(cap, cv2.COLOR_BGR2GRAY)
        aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_1000)
        arucoParameters = aruco.DetectorParameters_create()
        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=arucoParameters)
        frame = aruco.drawDetectedMarkers(image, corners)
        if(len(corners)>0):
            for i in range(4):
                cornerx = corners[0][0][i][0]
                corners[0][0][i][0] = corners[0][0][i][1] - 300
                corners[0][0][i][1] = 300 - cornerx
            print("id = ", ids, " , corners : ", corners)
        
            x_sum = corners[0][0][0][0]+ corners[0][0][1][0]+ corners[0][0][2][0]+ corners[0][0][3][0]
            y_sum = corners[0][0][0][1]+ corners[0][0][1][1]+ corners[0][0][2][1]+ corners[0][0][3][1]
                
            x_center = (x_sum*.25)
            y_center = (y_sum*.25)
            
            fov = camera.getFov()
            image_length = 2*zGPS*np.tan(fov/2)/np.sqrt(1-np.tan(fov//2)*np.tan(fov/2))
            print(image_length)
            Xaruco = xGPS + x_center*image_length/600
            Yaruco = yGPS + y_center*image_length/600
            X = Xaruco
            Y = Yaruco
        # cv2.imshow('image',cap)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            pass
            # break
    else:
        pass
    
    
    #defining setpoint
    x_PID.setpoint = target_X
    y_PID.setpoint = target_Y
    vertical_PID.setpoint = target_Z
    yaw_PID.setpoint = yaw
    
#############################################################################
    print("xGPS:{}\t\tyGPS:{}\t\tzGPS:{}".format(xGPS,yGPS,zGPS))
#############################################################################
########## for manual control with keyboard #################################
#############################################################################
    while True:
        if keyboard.is_pressed('shift+up'):
            vertical_dist = 1.0/130
            target_Z+=vertical_dist
            print(zGPS)
            break
        if keyboard.is_pressed('shift+down'):
            vertical_dist = -1.0/100
            target_Z+=vertical_dist
            print(zGPS)
            break
        if keyboard.is_pressed('up'):
            pitch_dist = 1.0/100
            Y+=pitch_dist
            print(yGPS)
            break
        if keyboard.is_pressed('left'):
            roll_dist = -1.0/100
            target_X+=roll_dist
            print(target_X)
            print(xGPS)
            break
        if keyboard.is_pressed('right'):
            roll_dist = 1.0/100
            target_X+=roll_dist
            print(target_X)
            print(xGPS)
            break
        if keyboard.is_pressed('down'):
            pitch_dist = -1.0/100
            Y+=pitch_dist
            print(target_Y)
            print(yGPS)
            break
        else:
            roll_dist = 0
            pitch_dist = 0
            vertical_dist = 0
            break

#############################################################################
###########for automatic control #################################
#############################################################################
    if zGPS >= target_Z:
        if target_Z <= Z :
            vertical_dist = 1.0
            target_Z+=vertical_dist
        else:
            cnt=1 
    
    if cnt==1:
        if X>0:
            if target_X < X:
                roll_dist = 1.0/100
                target_X+=roll_dist
            else:
                cnt=2
        else:
            if target_X > X:
                roll_dist = -1.0/100
                target_X+=roll_dist
            else:
                cnt=2
    
    if cnt==2:
        if Y>0:
            if target_Y < Y:
                    pitch_dist = 1.0/100
                    target_Y+=pitch_dist
        else:
             if target_Y > Y:
                    pitch_dist -= 1.0/100
                    target_Y+=pitch_dist

    
                  
    if Xaruco!=99 and abs(Xaruco-xGPS)<= 10*image_length/600 and abs(Yaruco-yGPS)<= 10*image_length/600:
        Z = 0
        cnt = 4
        if zGPS >= target_Z:
            if target_Z >= 0.1:
                vertical_dist = -1.0/200
                target_Z+=vertical_dist
            else:
                target_Z = 0
                vertical_dist = 0
                
    if zGPS <= 0.3 and cnt==4:
            front_left_motor.setVelocity(0)
            front_right_motor.setVelocity(0)
            rear_right_motor.setVelocity(0)
            rear_left_motor.setVelocity(0)
            continue
            
#############################################################################
##########       velocity input for req condition                  ##########
#############################################################################
    vertical_input = vertical_PID(zGPS)+vertical_dist
    yaw_input = yaw_PID(yaw)
    roll_input = 10 * roll + roll_acceleration-x_PID(xGPS)*0.5+roll_dist
    pitch_input = 10 * pitch - pitch_acceleration + y_PID(yGPS)*0.5+pitch_dist
    # print("roll input: {}     pitch input : {}".format(roll_input,pitch_input))
#############################################################################
##########               velocity input                            ##########
#############################################################################
    front_left_motor_input = thrust + vertical_input - roll_input - pitch_input + yaw_input
    front_right_motor_input = thrust + vertical_input + roll_input - pitch_input - yaw_input
    rear_left_motor_input = thrust + vertical_input - roll_input + pitch_input - yaw_input
    rear_right_motor_input = thrust + vertical_input + roll_input + pitch_input + yaw_input
#############################################################################
##########       setting the propeller velocity                    ##########
#############################################################################
    front_left_motor.setVelocity(front_left_motor_input)
    front_right_motor.setVelocity(-front_right_motor_input)
    rear_right_motor.setVelocity(rear_right_motor_input)
    rear_left_motor.setVelocity(-rear_left_motor_input)

    
