#!/usr/bin/env python

import rospy
import time
from std_msgs.msg import String
import RPi.GPIO as GPIO

"""
Autonomous Robotic Platforms
Remote Node
Reference: https://github.com/nestoregon/alphabot2pi_real
"""

IR = 17
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(IR,GPIO.IN)

def getkey():
    if GPIO.input(IR) == 0:
		count = 0
		while GPIO.input(IR) == 0 and count < 200:  #9ms
			count += 1
			time.sleep(0.00006)
		if(count < 10):
			return;
		count = 0
		while GPIO.input(IR) == 1 and count < 80:  #4.5ms
			count += 1
			time.sleep(0.00006)

		idx = 0
		cnt = 0
		data = [0,0,0,0]
		for i in range(0,32):
			count = 0
			while GPIO.input(IR) == 0 and count < 15:    #0.56ms
				count += 1
				time.sleep(0.00006)

			count = 0
			while GPIO.input(IR) == 1 and count < 40:   #0: 0.56mx
				count += 1                               #1: 1.69ms
				time.sleep(0.00006)

			if count > 7:
				data[idx] |= 1<<cnt
			if cnt == 7:
				cnt = 0
				idx += 1
			else:
				cnt += 1
#		print data
		if data[0]+data[1] == 0xFF and data[2]+data[3] == 0xFF:  #check
			return data[2]
		else:
			print("repeat")
			return "repeat"

def main():

    n = 0
    rospy.init_node("remote_node", anonymous=False)
    pubDirection = rospy.Publisher("/remote_key", String, queue_size=5)
    pubState = rospy.Publisher("/remote_state", String, queue_size=5)
    rate = rospy.Rate(100) #0.1

    while not rospy.is_shutdown():
        key = getkey()
        if(key != None):
            n = 0
            if key == 0x45:
                pubState.publish("manual")
            if key == 0x46:
                pubState.publish("ball_following_drive")
            if key == 0x47:
                pubState.publish("ball_following_servo")
            if key == 0x18:
                pubDirection.publish("forward")
            if key == 0x08:
                pubDirection.publish("left")
            if key == 0x1c:
                pubDirection.publish("stop")
            if key == 0x5a:
                pubDirection.publish("right")
            if key == 0x52:
                pubDirection.publish("backward")

        else:
            n += 1
            if n > 20000:
                n = 0
                # pub.publish("stop")
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

