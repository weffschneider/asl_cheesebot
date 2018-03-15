<<<<<<< HEAD
#!/usr/bin/env python


import rospy
=======
import rospy
from gazebo_msgs.msg import ModelStates
>>>>>>> e67f58a17f5fbac0ebbdddfdac51e030c3cc2a0f
from std_msgs.msg import Float32MultiArray, String, Bool



# callback for 
def rescue_on_callback(msg):
    rescue = msg.data
    if rescue:
        rate=rospy.Rate(1./2.)
        rate.sleep()
        
        rospy.loginfo('SIM_program: sending rescue-on command')
        rescue_pub.publish(True)




<<<<<<< HEAD
rospy.init_node('rescue_monitor', anonymous=True)
=======
rospy.init_node('rescue_moniter', anonymous=True)
>>>>>>> e67f58a17f5fbac0ebbdddfdac51e030c3cc2a0f


rospy.Subscriber('/ready_to_rescue', Bool, rescue_on_callback)
rescue_pub = rospy.Publisher('/rescue_on', Bool, queue_size=10)
<<<<<<< HEAD
rospy.spin()
=======

>>>>>>> e67f58a17f5fbac0ebbdddfdac51e030c3cc2a0f

