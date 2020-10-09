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
MULTIPLY = -2

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
        hand.ori = left_ori
        hand.pos = left_pos
        left_pinch = data.left_hand.pinch_strength
        hand_pinch = left_pinch
        left_grab = data.left_hand.grab_strength
        hand_grab = left_grab
        #yaw = tf.transformations.euler_from_quaternion([left_ori.x, left_ori.y, left_ori.z])[1]
	    #yaw = tf.transformations.euler_from_quaternion([left_ori.x, left_ori.y, left_ori.z, left_ori.w])[1]
	    yaw = MULTIPLY*data.left_hand.yaw

        if left_pos.x > 0: #Hand: Right  Robot:Forward
            pass
        if left_pos.x < 0: #Left  Back
            pass
        if left_pos.z > 0: #Back Left
            pass
        if left_pos.z < 0: #Forward  Right
            pass
        

    elif data.right_hand.is_present:
        if data.right_hand.is_present:
            right_pos = data.right_hand.palm_center
            right_ori = data.right_hand.direction
            hand_ori = right_ori
            hand_pos = right_pos
            right_pinch = data.right_hand.pinch_strength
            hand_pinch = right_pinch
            right_grab = right.right_hand.grab_strength
            hand_grab = right_grab
            #yaw = tf.transformations.euler_from_quaternion([right_ori.x, right_ori.y, right_ori.z])[1]
            #yaw = tf.transformations.euler_from_quaternion([right_ori.x, right_ori.y, right_ori.z, right_ori.w])[1]
            yaw = MULTIPLY*data.right_hand.yaw


    if hand_pinch >= 0.5:
        if ROBOT_TYPE == "OMNI":
        if abs(hand_pos.z) < VELOCITY_TOLERANCE:
                hand_pos.z = 0
        if abs(hand_pos.x) < VELOCITY_TOLERANCE:
                hand_pos.x = 0

        twist_msg.linear.x = min(max(MULTIPLY*hand_pos.z,-MAX_LIN_VEL),MAX_LIN_VEL)
        twist_msg.linear.y = min(max(MULTIPLY*hand_pos.x,-MAX_LIN_VEL),MAX_LIN_VEL)



    
    if abs(yaw) < ANGLE_TOLERANCE:
        yaw=0

    if hand_grab != 1:
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
