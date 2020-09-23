#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from robotont_msgs.msg import LedModuleSegment, ColorRGB
from leap_motion_controller.msg import Set, Hand, Finger
import time
import math

MAX_LIN_VEL = 0.05
MAX_ANG_VEL = 0.6
ANGLE_TOLERANCE = 0.1

last_heartbeat = 0

yaw = 0

def callback(data):
    global last_heartbeat, yaw
    if data.left_hand.is_present:
        left_pos = data.left_hand.palm_pose.pose.position
        left_ori = data.left_hand.palm_pose.pose.orientation
        left_euler = tf.transformations.euler_from_quaternion(left_ori)
        yaw = euler[2]

        rospy.loginfo("got left hand:", left_pos.z)
        rospy.loginfo("  pos z:", left_pos.z)
        rospy.loginfo("  euler:", euler)

    if data.right_hand.is_present:
        right_pos = data.right_hand.palm_pose.pose.position
        right_ori = data.right_hand.palm_pose.pose.orientation
        right_euler = tf.transformations.euler_from_quaternion(left_ori)
        yaw = euler[2]

        rospy.loginfo("got right hand:", right_pos.z)
        rospy.loginfo("  pos z:", right_pos.z)
        rospy.loginfo("  euler:", euler)


    twist_msg = Twist()
    twist_msg.angular.z = min(max(yaw,-MAX_ANG_VEL),MAX_ANG_VEL)

    if abs(yaw) > ANGLE_TOLERANCE:
        global cmd_vel_pub
        last_heartbeat = rospy.get_time()
        cmd_vel_pub.publish(twist_msg)


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
    rospy.Subscriber("leap_motion_output", Set, callback)

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
