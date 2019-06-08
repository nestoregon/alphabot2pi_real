#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge

from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from std_msgs.msg import String
from std_msgs.msg import Float32
from sensor_msgs.msg import CompressedImage
import time

class Control:
    """Control node class"""
    def __init__(self):
        # subscribers
        self.sub_ball_blue = rospy.Subscriber("/ball_blue_location", Point, self.callback_blue)
        self.sub_remote_key  = rospy.Subscriber("/remote_key", String, self.callback_remote_key)
        self.sub_remote_state  = rospy.Subscriber("/remote_state", String, self.callback_remote_state)
        # publishers
        self.pub_cmd_vel = rospy.Publisher("/cmd_vel", Twist, queue_size=5)
        self.pub_servo = rospy.Publisher("/servo_location", Point, queue_size=10)

        self.vel = Twist() # Twist object to transfer velocity to wheel
        self.servo = Point() # Point object to transfer location to servo
        self.servo.x, self.servo.y = 0, 0 # center for ball_following_drive
        self.w, self.h = 410, 308 # default image size
        self.state = "manual"
        self.remote_key = "stop"
        self.power_difference = 0.0

    # callback function that saves the blue ball location
    def callback_blue(self, data):
        self.blue_x = data.x
        self.blue_y = data.y

    # callback function that saves the key pressed
    def callback_remote_key(self, string):
        if self.state == "manual" or self.state == "ball_following_servo":
            self.remote_key = string.data

    # callback function that saves the state of the robot
    def callback_remote_state(self, string):
        self.state = string.data
        if self.state == "manual":
            self.servo.x, self.servo.y = 0 ,0
            self.vel.linear.x, self.vel.angular.z = 0.0, 0.0
        if self.state == "ball_following_drive":
            self.servo.x, self.servo.y = 1750, 900
        elif self.state == "ball_following_servo":
            self.servo.x, self.servo.y = 1750, 900
            self.vel.linear.x, self.vel.angular.z = 0.0, 0.0

    # get width and height of image
    def get_camera(self):
        bridge = CvBridge()
    	image_msg = rospy.wait_for_message("/raspicam_node/image/compressed",CompressedImage)
    	image = bridge.compressed_imgmsg_to_cv2(image_msg, "bgr8")
    	self.h , self.w= image.shape[:2]

    # manual mode, press numbers to move the robot
    def manual_drive(self):
        print(self.remote_key)
        if self.remote_key == "forward":
            self.vel.linear.x = 0.5
            self.vel.angular.z = 0.0
        if self.remote_key == "backward":
            self.vel.linear.x = -0.5
            self.vel.angular.z = 0.0
        if self.remote_key == "stop":
            self.vel.linear.x = 0.0
            self.vel.angular.z = 0.0
        if self.remote_key == "right":
            self.vel.linear.x = 0.0
            self.vel.angular.z = 0.3
        if self.remote_key == "left":
            self.vel.linear.x = 0.0
            self.vel.angular.z = -0.3


    # BALL FOLLOWING SERVO
    # Robot servo movement depending on ball postition within the image
    # Ball position is normalized (CENTER=0,0)
    # width=(-0.5,+0.5), height=(-0.5,+0.5)

    #  L+U |   UP   |  U+R
    #------|--------|------  -0.10
    # LEFT | CENTER | RIGHT
    #------|--------|------  +0.10
    #  L+D |  DOWN  |  D+R
    #
    #   -0.10     +0.10

    # ball following servo mode, the robot faces the ball using the servos
    def ball_following_servo(self):
        if self.blue_x < 0 and self.blue_y < 0:
            # ball out of sight
            print("no ball")
        else:
            mid_x  	= int(self.w/2)
            mid_y   = int(self.h/2)
            # x & y position relative to the center of the image
            delta_x	= self.blue_x - mid_x
            delta_y = self.blue_y - mid_y
            # normalize x & y relative postition
            norm_x 	= delta_x/self.w
            norm_y  = delta_y/self.h
            # x axis
            if norm_x > 0.1:
                print ("RIGHT {:.3f}".format(norm_x))
                self.servo.x -= 10
            elif norm_x < -0.1:
                print ("LEFT {:.3f}".format(norm_x))
                self.servo.x += 10
            if abs(norm_x) < 0.1:
                print ("CENTER {:.3f}".format(norm_x))
                self.servo.x += 0
            # y axis
            if norm_y > 0.15:
                print ("DOWN {:.3f}".format(norm_y))
                self.servo.y += 10
            elif norm_y < -0.15:
                print ("UP {:.3f}".format(norm_y))
                self.servo.y -= 10
            elif abs(norm_y) < 0.15:
                print ("CENTER {:.3f}".format(norm_y))
                self.servo.y +=  0

            # restrictions x
            if self.servo.x > 2300:
                self.servo.x = 2300
            elif self.servo.x < 600:
                self.servo.x = 600
            # restrictions y
            if self.servo.y > 1100:
                self.servo.y = 1100
            elif self.servo.y < 500:
                self.servo.y = 500

            print(self.servo.x)
            print(self.servo.y)

    # BALL FOLLOWING DRIVE
    # Robot movement depending on ball postition within the image
    # Ball position is normalized (CENTER=0,0)
    # width=(-0.5,+0.5), height=(-0.5,+0.5)

    #  L+F |  FRWD  |  F+R
    #------|--------|------  +0.25
    # LEFT | CENTER | RIGHT
    #------|--------|------  +0.30
    #  L+B |  BACK  |  B+R
    #
    #   -0.25     +0.25

    def ball_following_drive(self):

        if self.blue_x < 0 and self.blue_y < 0:
            # There is no ball, spin until you find it
            self.vel.linear.x = 0
            self.vel.angular.z = 0.20
        else:
            mid_x  	= int(self.w/2)
            mid_y   = int(self.h/2)
            # x & y position relative to the center of the image
            delta_x	= self.blue_x - mid_x
            delta_y = self.blue_y - mid_y
            # normalize x & y relative postition
            norm_x 	= delta_x/self.w
            norm_y  = delta_y/self.h
            # angular
            if norm_x > 0.25:
                print ("RIGHT {:.3f}".format(norm_x))
                # self.vel.angular.z =  0.03 # Fast mode
                self.vel.angular.z =  0.07 # Slow mode
            elif norm_x < -0.25:
                print ("LEFT {:.3f}".format(norm_x))
                # self.vel.angular.z =  -0.03 # Fast mode
                self.vel.angular.z =  - 0.07 # Slow mode
            if abs(norm_x) < 0.25:
                print ("CENTER {:.3f}".format(norm_x))
                self.vel.angular.z = 0
            # linear
            if norm_y < 0.25:
                print ("FORWARD {:.3f}".format(norm_y))
                self.vel.linear.x =  0.25
            elif norm_y < 0.30:
                print ("STOP {:.3f}".format(norm_y))
                self.vel.linear.x  =  0
            else:
                print ("BACKWARDS {:.3f}".format(norm_y))
                self.vel.linear.x  = -0.35

    def run(self):
        self.get_camera()
        rate = rospy.Rate(30) #0.0333 seconds
        while not rospy.is_shutdown():

            if self.state == "manual":
                self.manual_drive()

            elif self.state == "ball_following_drive":
                self.ball_following_drive()

            elif self.state == "ball_following_servo":
                self.ball_following_servo()

            self.pub_cmd_vel.publish(self.vel) # publish velocity on /cmv_vel topic
            self.pub_servo.publish(self.servo) # publish servo location on /servo_location topic
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node("control_node", anonymous=False)
    my_control = Control()
    try:
        my_control.run()
    except rospy.ROSInterruptException:
        pass
