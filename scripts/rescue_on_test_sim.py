#!/usr/bin/env python


import rospy
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import Float32MultiArray, String, Bool



# callback for 
def rescue_on_callback(msg):
    rescue = msg.data
    if rescue:
        rate=rospy.Rate(1./2.)
        rate.sleep()
        
        rospy.loginfo('SIM_program: sending rescue-on command')
        rescue_pub.publish(True)




rospy.init_node('rescue_monitor', anonymous=True)


rospy.Subscriber('/ready_to_rescue', Bool, rescue_on_callback)
rescue_pub = rospy.Publisher('/rescue_on', Bool, queue_size=10)
rospy.spin()

