#!/usr/bin/env python
import rospy
import tf
from geometry_msgs.msg import Twist
from robotont_msgs.msg import LedModuleSegment, ColorRGB
from leap_motion.msg import Human, Hand, Finger

import time
import math

MAX_LIN_VEL = 0.5
MAX_ANG_VEL = 1.5
ANGLE_TOLERANCE = 0.15
VELOCITY_TOLERANCE = 0.028

# Choose robot type
ROBOT_TYPE = "OMNI"
# Not ready
# ROBOT_TYPE = "ORI"

last_heartbeat = 0


def callback(data):
    global last_heartbeat
    yaw = 0
    twist_msg = Twist()
    if data.left_hand.is_present:
        left_pos = data.left_hand.palm_center
        left_ori = data.left_hand.direction
        #yaw = tf.transformations.euler_from_quaternion([left_ori.x, left_ori.y, left_ori.z])[1]
	#yaw = tf.transformations.euler_from_quaternion([left_ori.x, left_ori.y, left_ori.z, left_ori.w])[1]
	yaw = -2*data.left_hand.yaw
	#rospy.loginfo("Have left hand Here")
        #rospy.loginfo("got left hand: %f", left_pos.z)
        #rospy.loginfo("  pos z: %f", left_pos.z)
        #rospy.loginfo("  yaw: %f", yaw)
        if left_pos.x > 0: #Hand: Right  Robot:Forward
            pass
        if left_pos.x < 0: #Left  Back
            pass
        if left_pos.z > 0: #Back Left
            pass
        if left_pos.z < 0: #Forward  Right
            pass
        #rospy.loginfo("  position x: %f", left_pos.x)
        #rospy.loginfo("  position y: %f", left_pos.y)
	#rospy.loginfo("  pinch_strength: %f", data.left_hand.pinch_strength)
        if data.left_hand.pinch_strength >= 0.5:
	    #rospy.loginfo(str(left_pos.x) +"   "+ str(left_pos.z))
            if ROBOT_TYPE == "OMNI":
	    	#left_pos = data.left_hand.pose.center
            	#rospy.loginfo("I am moving")
	    	if abs(left_pos.z) < VELOCITY_TOLERANCE:
    	        	left_pos.z = 0
	    	if abs(left_pos.x) < VELOCITY_TOLERANCE:
                	left_pos.x = 0

            	twist_msg.linear.x = min(max(-2*left_pos.z,-MAX_LIN_VEL),MAX_LIN_VEL)
            	twist_msg.linear.y = min(max(-2*left_pos.x,-MAX_LIN_VEL),MAX_LIN_VEL)
            
	    #elif ROBOT_TYPE == "ORI":
	    	#if abs(left_pos.x) >= 0.1:
		    #pass
	    #rospy.loginfo("  position: %f", twist_msg.linear.x)
            #rospy.loginfo("  position: %f", twist_msg.linear.y)
            
			

    if data.right_hand.is_present:
        right_pos = data.right_hand.palm_center
        #right_ori = data.right_hand.palm_pose.pose.orientation
        right_ori = data.right_hand.direction
        #yaw = tf.transformations.euler_from_quaternion([right_ori.x, right_ori.y, right_ori.z])[1]
	#yaw = tf.transformations.euler_from_quaternion([right_ori.x, right_ori.y, right_ori.z, right_ori.w])[1]
	yaw = -2*data.right_hand.yaw
	#rospy.loginfo("Have right hand Here")
        #rospy.loginfo("got right hand: %f", right_pos.z)
        #rospy.loginfo("  pos z: %f", right_pos.z)
        #rospy.loginfo("  yaw: %f", yaw)
        if right_pos.x > 0:
            pass
        if right_pos.x < 0:
            pass
        if right_pos.z > 0:
            pass
        if right_pos.z < 0:
            pass
        #rospy.loginfo("  position x: %f", right_pos.x)
        #rospy.loginfo("  position y: %f", right_pos.y)
        #rospy.loginfo("  pinch_strength: %f", data.right_hand.pinch_strength)
		
        if data.right_hand.pinch_strength >= 0.5:
            #right_pos = data.right_hand.pose.center
            #rospy.loginfo("Liigutan")
	    if abs(right_pos.z) < VELOCITY_TOLERANCE:
               	right_pos.z = 0
            if abs(right_pos.x) < VELOCITY_TOLERANCE:
            	right_pos.x = 0

            twist_msg.linear.x = min(max(-2*right_pos.z,-MAX_LIN_VEL),MAX_LIN_VEL)
            twist_msg.linear.y = min(max(-2*right_pos.x,-MAX_LIN_VEL),MAX_LIN_VEL)
            #rospy.loginfo("  position: %f", twist_msg.linear.x)
            #rospy.loginfo("  position: %f", twist_msg.linear.y)

    
    if abs(yaw) < ANGLE_TOLERANCE:
        yaw=0

    if data.left_hand.grab_strength != 1:
        twist_msg.angular.z = min(max(yaw,-MAX_ANG_VEL),MAX_ANG_VEL)
	#if yaw != 0:
	    #rospy.loginfo(" Speed: %f", twist_msg.angular.z)

    global cmd_vel_pub
    cmd_vel_pub.publish(twist_msg)
    last_heartbeat = rospy.get_time()


# Publish zero cmd_vel when no AR info has been received within given period
def timer_callback(event):
	global last_heartbeat
	if (rospy.get_time() - last_heartbeat) >= 0.5:
		cmd_vel_pub.publish(Twist())


def controller_main():
    # Starts a new node
    rospy.init_node('lmc_demo', anonymous=True)

    # Initialize publishers
    global led_pub
    led_pub = rospy.Publisher('/robotont/led_segment', LedModuleSegment, queue_size=1)
    global cmd_vel_pub
    cmd_vel_pub = rospy.Publisher('/robotont/cmd_vel', Twist, queue_size=1)
    led_msg = LedModuleSegment()

    # Initialize subscriber
    rospy.Subscriber("/leap_motion/leap_device", Human, callback)  #/robotont/leap_motion_output

    #Register heartbeat timer
    t = rospy.Timer(rospy.Duration(0.1), timer_callback)

    LED_COUNT=60
    led_msg.idx_start = 0
    led_msg.colors = []
    color = ColorRGB()

    #Init Led strip
    led_msg.colors = []
    color.r = 128
    color.g = 128
    color.b = 0
    for i in range(0, LED_COUNT):
        led_msg.colors.append(color)
    led_pub.publish(led_msg)

    rospy.spin()



if __name__ == '__main__':
    try:
        controller_main()
    except rospy.ROSInterruptException:
        print("Program interrupted, exiting ...")
        pass
