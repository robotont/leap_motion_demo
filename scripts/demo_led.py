#!/usr/bin/env python
import rospy
import tf
from robotont_msgs.msg import LedModuleSegment, ColorRGB
from leap_motion.msg import Set, Hand, Finger
import time
import math

last_heartbeat = 0
ANGLE_TOLERANCE = 0.1

LED_COUNT=60
led_msg = LedModuleSegment()
led_msg.idx_start = 0
led_msg.colors = []
color = ColorRGB()

def callback(data):
    global last_heartbeat, led_pub, led_msg
    r = 0
    g = 0
    b = 0

    if data.left_hand.is_present:
        left_pos = data.left_hand.palm_pose.pose.position
        left_ori = data.left_hand.palm_pose.pose.orientation
        yaw = tf.transformations.euler_from_quaternion([left_ori.x, left_ori.y, left_ori.z, left_ori.w])[1]

#        rospy.loginfo("got left hand: %f", left_pos.x)
#        rospy.loginfo("  pos z: %f", left_pos.y)
#        rospy.loginfo("  yaw: %f", yaw)

        r = min(max((left_pos.y-0.3)*1000+128,0),255)
        b = min(max(-yaw*100+64,0),127)

    if data.right_hand.is_present:
        right_pos = data.right_hand.palm_pose.pose.position
        right_ori = data.right_hand.palm_pose.pose.orientation
        yaw = tf.transformations.euler_from_quaternion([right_ori.x, right_ori.y, right_ori.z, right_ori.w])[1]

#        rospy.loginfo("got right hand: %f", right_pos.x)
#        rospy.loginfo("  pos z: %f", right_pos.y)
#        rospy.loginfo("  yaw: %f", yaw)
        g = min(max((right_pos.y-0.3)*1000+128,0),255)
        b += min(max(yaw*100+64,0),127)

    #Show Led strip
    led_msg.colors = []
    color.r = r
    color.g = g
    color.b = b
    for i in range(0, LED_COUNT):
        led_msg.colors.append(color)
    led_pub.publish(led_msg)
    time.sleep(0.1)


    last_heartbeat = rospy.get_time()


# Publish zero cmd_vel when no AR info has been received within given period
def timer_callback(event):
	global last_heartbeat
	if (rospy.get_time() - last_heartbeat) >= 0.5:
            rospy.logdebug("timeout")
            pass


def controller_main():
    # Starts a new node
    rospy.init_node('lmc_led_demo', anonymous=True)

    # Initialize publishers
    global led_pub
    led_pub = rospy.Publisher('/robotont/led_segment', LedModuleSegment, queue_size=1)

    # Initialize subscriber
    rospy.Subscriber("/robotont/leap_motion_output", Set, callback)

    #Register heartbeat timer
    t = rospy.Timer(rospy.Duration(0.1), timer_callback)



    rospy.spin()



if __name__ == '__main__':
    try:
        controller_main()
    except rospy.ROSInterruptException:
        print("Program interrupted, exiting ...")
        pass
