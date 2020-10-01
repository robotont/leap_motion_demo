#!/usr/bin/env python
import rospy
import tf
from geometry_msgs.msg import Twist
from robotont_msgs.msg import LedModuleSegment, ColorRGB
#from leap_motion_controller.msg import Set, Hand, Finger
from leap_motion.msg import Human, Hand, Finger #Set, 

import time
import math

MAX_LIN_VEL = 0.05
MAX_ANG_VEL = 0.6
ANGLE_TOLERANCE = 0.1

last_heartbeat = 0


def callback(data):
    global last_heartbeat
    yaw = 0

    if data.left_hand.is_present:
        #left_pos = data.left_hand.palm_pose.pose.position
        left_pos = data.left_hand.palm_center
        #left_ori = data.left_hand.palm_pose.pose.orientation
        left_ori = data.left_hand.direction
        #yaw = tf.transformations.euler_from_quaternion([left_ori.x, left_ori.y, left_ori.z])[1]
	#yaw = tf.transformations.euler_from_quaternion([left_ori.x, left_ori.y, left_ori.z, left_ori.w])[1]
	yaw = data.left_hand.yaw
	rospy.loginfo("Have left hand Here")
        rospy.loginfo("got left hand: %f", left_pos.z)
        rospy.loginfo("  pos z: %f", left_pos.z)
        rospy.loginfo("  yaw: %f", yaw)

    if data.right_hand.is_present:
	pass
        #right_pos = data.right_hand.palm_pose.pose.position
        #right_ori = data.right_hand.palm_pose.pose.orientation
        #yaw = tf.transformations.euler_from_quaternion([right_ori.x, right_ori.y, right_ori.z, right_ori.w])[1]
	
        #rospy.loginfo("got right hand: %f", right_pos.z)
        #rospy.loginfo("  pos z: %f", right_pos.z)
        #rospy.loginfo("  yaw: %f", yaw)

    if abs(yaw) < ANGLE_TOLERANCE:
        yaw=0

    twist_msg = Twist()
    twist_msg.angular.z = min(max(yaw,-MAX_ANG_VEL),MAX_ANG_VEL)


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
