#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
import time
import RPi.GPIO as GPIO

IN1 = 13
IN2 = 12
IN3 = 21
IN4 = 20
ENA = 6
ENB = 26
PA  = 50
PB  = 50

"""
Autonomous Robotic Platforms
Driver Node
Reference: https://github.com/azazdeaz/alphabot2-ros/
"""

class driver:
  def __init__(self):
    self.IN1 = 13
    self.IN2 = 12
    self.IN3 = 21
    self.IN4 = 20
    self.ENA = 6
    self.ENB = 26
    self.PA  = 50
    self.PB  = 50

    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    GPIO.setup(self.IN1, GPIO.OUT)
    GPIO.setup(self.IN2, GPIO.OUT)
    GPIO.setup(self.IN3, GPIO.OUT)
    GPIO.setup(self.IN4, GPIO.OUT)
    GPIO.setup(self.ENA, GPIO.OUT)
    GPIO.setup(self.ENB, GPIO.OUT)
    self.PWMA = GPIO.PWM(self.ENA,500)
    self.PWMB = GPIO.PWM(self.ENB,500)
    self.PWMA.start(self.PA)
    self.PWMB.start(self.PB)
    self.stop()

  def stop(self):
    self.PWMA.ChangeDutyCycle(0)
    self.PWMB.ChangeDutyCycle(0)
    GPIO.output(self.IN1, GPIO.LOW)
    GPIO.output(self.IN2, GPIO.LOW)
    GPIO.output(self.IN3, GPIO.LOW)
    GPIO.output(self.IN4, GPIO.LOW)


  def set_motor(self, left, right):
    print('left={} right={}'.format(left, right))

    if (right >= 0) and (right <= 100):
      GPIO.output(self.IN1, GPIO.HIGH)
      GPIO.output(self.IN2, GPIO.LOW)
      self.PWMA.ChangeDutyCycle(right)
    elif (right < 0) and (right >= -100):
      GPIO.output(self.IN1, GPIO.LOW)
      GPIO.output(self.IN2, GPIO.HIGH)
      self.PWMA.ChangeDutyCycle(0 - right)
    if (left >= 0) and (left <= 100):
      GPIO.output(self.IN3, GPIO.HIGH)
      GPIO.output(self.IN4, GPIO.LOW)
      self.PWMB.ChangeDutyCycle(left)
    elif (left < 0) and (left >= -100):
      GPIO.output(self.IN3, GPIO.LOW)
      GPIO.output(self.IN4, GPIO.HIGH)
      self.PWMB.ChangeDutyCycle(0 - left)

  # get cmd_vel message, and get linear velocity and angular velocity
  def drive(self, data):
    x = data.linear.x
    angular = data.angular.z
    # calculate right and left wheels' signal
    right = int((x + angular) * 50)
    left = int((x - angular) * 50)
    self.set_motor(left, right)



class drive_node:

  def __init__(self):
    """ Initialize drive node """
    rospy.init_node('drive', anonymous= True)

    """ Subscribe to alphabot_control topic of Twist type"""
    self.sub = rospy.Subscriber('/cmd_vel', Twist, self.callback)

    """ Initialize publisher to Gazebo"""
    #self.gazebo_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    """ Initialize drive of real robot """
    # self.real_robot = driver()


  def callback(self, message_received):
    """ Sending to Simulator """
    self.gazebo_pub.publish(message_received)

    """ Sending to Alphabot2 """
    # self.real_robot.drive(message_received)


def main():
  cn = drive_node()
  rospy.spin()

if __name__ == '__main__':
  try:
    main()
  except rospy.ROSInterruptException:
    print ("Shutting down drive wheel node.")
    pass

  
