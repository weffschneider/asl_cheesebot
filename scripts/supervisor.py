#!/usr/bin/env python

import rospy
# from gazebo_msgs.msg import ModelStates
from std_msgs.msg import Float32MultiArray, String, Bool
from geometry_msgs.msg import Twist, PoseArray, Pose2D, PoseStamped
from asl_turtlebot.msg import DetectedObject
from visualization_msgs.msg import Marker, MarkerArray
import tf
import math
from enum import Enum
import numpy as np

# threshold at which we consider the robot at a location
POS_EPS = .1
THETA_EPS = .3

# time to stop at a stop sign
STOP_TIME = 10

# minimum distance from a stop sign to obey it
# (bounding box height in pixels)
STOP_MIN_HEIGHT = 50

# time taken to cross an intersection
CROSSING_TIME = 15

# time to stop for animal rescue
RESCUE_TIME = 10

# state machine modes, not all implemented
class Mode(Enum):
    IDLE = 1
    POSE = 2 # unused
    STOP = 3
    CROSS = 4
    NAV = 5
    WAIT_FOR_INSTR = 6
    RESCUE = 7

class Supervisor:
    """ the state machine of the turtlebot """

    def __init__(self):
        rospy.init_node('turtlebot_supervisor', anonymous=True)

        # current pose
        self.x = 0
        self.y = 0
        self.theta = 0

        # pose goal
        self.x_g = 0
        self.y_g = 0
        self.theta_g = 0

        # TODO: hard coded for now
        self.firestation_x = 0
        self.firestation_y = 0
        self.firestation_theta = 0

        # current mode
        self.mode = Mode.IDLE
        self.last_mode_printed = None

        # flags for control flow
        self.exploring = True

        # rescuing animals
        self.animal_poses = []
        #self.animal_poses = [(1.4,0.3),(2.16,1.5)] # TEST
        self.initialized_rescue = False

        # create publishers
        self.nav_goal_publisher = rospy.Publisher('/cmd_nav', Pose2D, queue_size=10)
        self.pose_goal_publisher = rospy.Publisher('/cmd_pose', Pose2D, queue_size=10)
        self.cmd_vel_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.rescue_ready = rospy.Publisher('/ready_to_rescue', Bool, queue_size=1)
        self.pub_mark = rospy.Publisher('/visualization_marker_array', MarkerArray, queue_size = 10)
        self.markerlist = MarkerArray()
        self.markerid = 0
        
        # create subscribers
        rospy.Subscriber('/detector/stop_sign', DetectedObject, self.stop_sign_detected_callback)
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.rviz_goal_callback)
        rospy.Subscriber('/rescue_on', Bool, self.rescue_on_callback)
        # TODO: add Subscriber for animal detection: self.animal_detected_callback

        self.trans_listener = tf.TransformListener()

        # flag for detection
        self.cat_detected = False
        self.dog_detected = False
        self.elephant_detected = False

        # detector labels in tfmodels/coco_labels.txt
        # published by "camera_common" in "detector.py"
        rospy.Subscriber('/detector/cat', DetectedObject, self.animal_detected_callback)
        rospy.Subscriber('/detector/dog', DetectedObject, self.animal_detected_callback)
        rospy.Subscriber('/detector/elephant', DetectedObject, self.animal_detected_callback)

    def animal_detected_callback(self, msg):
        # call back for when the detector has found an animal - cat or dog -

        ymin, _, ymax, _ = msg.corners
        height = ymax-ymin
        print('-------------------Detecting-----------------')

        # record the animal's position w.r.t. the robot's pose
        if msg.name == 'cat' and not self.cat_detected:
            dist = 1.98*10**(-5)*height**2 - 7.88*10**(-3)*height + 0.935
            x_proj = self.x + dist*np.cos(self.theta)
            y_proj = self.y + dist*np.sin(self.theta)
            self.animal_poses.append( (x_proj, y_proj) )
            self.rviz_add_marker('cat', x_proj, y_proj)
            self.cat_detected = True
            print('append cat')

        elif msg.name == 'dog' and not self.dog_detected:
            dist = -6.37*10**(-6)*height**2 + 4.27*10**(-4)*height + 0.255
            x_proj = self.x + dist*np.cos(self.theta)
            y_proj = self.y + dist*np.sin(self.theta)
            self.animal_poses.append( (x_proj, y_proj) )
            self.rviz_add_marker('dog', x_proj, y_proj)
            self.dog_detected = True
            print('append dog')
            
        elif msg.name == 'elephant' and not self.elephant_detected:
            dist = 4.25*10**(-5)*height**2 - 1.36*10**(-2)*height + 1.25
            x_proj = self.x + dist*np.cos(self.theta)
            y_proj = self.y + dist*np.sin(self.theta)
            self.animal_poses.append( (x_proj, y_proj) )
            self.rviz_add_marker('elephant', x_proj, y_proj)
            self.elephant_detected = True
            print('append elephant')

        print(self.animal_poses)
        print('---------------------Detected-------------')


    # def record_animal_frame(self, msg):
    #     # OUT : [x, y, theta] of animal position in world frame

    #     # msg type : DetectedObject
    #     name = msg.name
    #     ymin, _, ymax, _ = msg.corners
    #     height = ymax-ymin

    #     # frame name depends on what animal we found
    #     frame_name = name + "_pose"

    #     return (self.x, self.y)
    
    def rviz_add_marker(self, name, x, y):
        '''add marker to rviz'''
        # ellipsoid marker
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.type = marker.SPHERE
        marker.action = marker.ADD
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.01
        
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = x
        marker.pose.position.y = y 
        marker.pose.position.z = 0 
        
        marker.id = self.markerid
        self.markerid += 1
        
        self.markerlist.markers.append(marker)
        
        # text marker
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.type = marker.TEXT_VIEW_FACING
        marker.action = marker.ADD
        
        marker.scale.z = 0.07
        
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = x
        marker.pose.position.y = y 
        marker.pose.position.z = .1 
        
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        
        marker.text = name
        
        marker.id = self.markerid
        self.markerid += 1
        
        self.markerlist.markers.append(marker)
        self.pub_mark.publish(self.markerlist)

    def rviz_goal_callback(self, msg):
        """ callback for a pose goal sent through rviz """

        self.x_g = msg.pose.position.x
        self.y_g = msg.pose.position.y
        rotation = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
        euler = tf.transformations.euler_from_quaternion(rotation)
        self.theta_g = euler[2]

        self.mode = Mode.NAV

    def stop_sign_detected_callback(self, msg):
        """ callback for when the detector has found a stop sign. Note that
        a distance of 0 can mean that the lidar did not pickup the stop sign at all """

        # distance of the stop sign
        # dist = msg.distance
        ymin, _, ymax, _ = msg.corners
        height = ymax-ymin

        # if close enough and in nav mode, stop
        if height > STOP_MIN_HEIGHT and self.mode == Mode.NAV:
            self.init_stop_sign()


    def rescue_on_callback(self, msg):
        """ callback for when we receive a rescue_on message from the fire station """

        rescue_on = msg.data

        if rescue_on and self.mode == Mode.WAIT_FOR_INSTR:
            self.update_waypoint()
            self.mode = Mode.NAV
            # NOTE: update_waypoint needs to happen BEFORE setting exploring to false,
            # otherwise we'll think we're done
            self.exploring = False


    def go_to_pose(self):
        # unused
        """ sends the current desired pose to the pose controller """

        pose_g_msg = Pose2D()
        pose_g_msg.x = self.x_g
        pose_g_msg.y = self.y_g
        pose_g_msg.theta = self.theta_g

        self.pose_goal_publisher.publish(pose_g_msg)

    def nav_to_pose(self):
        """ sends the current desired pose to the naviagtor """

        nav_g_msg = Pose2D()
        nav_g_msg.x = self.x_g
        nav_g_msg.y = self.y_g
        nav_g_msg.theta = self.theta_g

        self.nav_goal_publisher.publish(nav_g_msg)

    def stay_idle(self):
        """ sends zero velocity to stay put """

        vel_g_msg = Twist()
        vel_g_msg.linear.x = 0
        vel_g_msg.linear.y = 0
        vel_g_msg.angular.z = 0

        self.cmd_vel_publisher.publish(vel_g_msg)

    def close_to(self,x,y,theta,eps):
        """ checks if the robot is at a pose within some threshold """

        #return (abs(x-self.x)<eps and abs(y-self.y)<eps and abs(theta-self.theta)<THETA_EPS)
        return (abs(x-self.x)<eps and abs(y-self.y)<eps) # for now we don't care about orientation

    def set_goal_pose(self,x,y,theta):
        """ sets the goal pose """

        self.x_g = x
        self.y_g = y
        self.theta_g = theta

    def init_stop_sign(self):
        """ initiates a stop sign maneuver """

        self.stop_sign_start = rospy.get_rostime()
        self.stay_idle()
        self.mode = Mode.STOP

    def has_stopped(self):
        """ checks if stop sign maneuver is over """

        return (self.mode == Mode.STOP and (rospy.get_rostime()-self.stop_sign_start)>rospy.Duration.from_sec(STOP_TIME))

    def init_crossing(self):
        """ initiates an intersection crossing maneuver """

        self.cross_start = rospy.get_rostime()
        self.mode = Mode.CROSS

    def has_crossed(self):
        """ checks if crossing maneuver is over """

        return (self.mode == Mode.CROSS and (rospy.get_rostime()-self.cross_start)>rospy.Duration.from_sec(CROSSING_TIME))

    def init_rescue(self):
        """ initiates an animal rescue """

        self.rescue_start = rospy.get_rostime()
        self.stay_idle()
        self.mode = Mode.RESCUE

    def has_rescued(self):
        """ checks if rescue is over"""
        return (self.mode == Mode.RESCUE and (rospy.get_rostime()-self.rescue_start)>rospy.Duration.from_sec(RESCUE_TIME))

    def wait_for_instr(self):
        """ sends ready_to_rescue message, wait for response """
        self.stay_idle()
	print('Waiting for Instruction')
        self.rescue_ready.publish(True)
        self.mode = Mode.WAIT_FOR_INSTR


    def update_waypoint(self):
        # Update goal pose (x_g, y_g, theta_g), then switch to NAV mode to get there

        # update_waypoint should only be called when you are rescuing animals
        # otherwise this is going to start setting the waypoint to the nearest animal
        # to rescue and compete with 2D nav in RViz

        # 1. Find closest unrescued animal
        # 2. Go to animal
        # 3. Rescue
        # 4. Repeat 1-3 until no animals to rescue
        # 5. Return to firestation

        num_animals = len(self.animal_poses)

        if not self.initialized_rescue:
            # initialize boolean array to record which animals have been rescued
            self.to_rescue = np.ones(num_animals, dtype=bool)
            self.initialized_rescue = True

        if np.any(self.to_rescue):
            # Iterates through each of the animals and chooses the first one
            # in the list that has not been rescued yet.
            # Because we know that there are 3 animals max to be picked up
            # this for loop is fine. For any other circumstance this should be
            # done differently.
            animal_dist = np.zeros(num_animals)
            for i in range(num_animals):
                if self.to_rescue[i]:
                    pose_rescue = self.animal_poses[i]
                    self.to_rescue[i] = False
                    break
                else:
                    pass

           # Set the goal pose
            self.set_goal_pose(pose_rescue[0],pose_rescue[1],0.0)
        else:
            # Go to the firestation
            self.set_goal_pose(self.firestation_x,self.firestation_y,self.firestation_theta)
        print("Waypoint Updated")

        # Change the mode
        self.mode = Mode.NAV

    def loop(self):
        """ the main loop of the robot. At each iteration, depending on its
        mode (i.e. the finite state machine's state), if takes appropriate
        actions. This function shouldn't return anything """

        try:
            (translation,rotation) = self.trans_listener.lookupTransform('/map', '/base_footprint', rospy.Time(0))
            self.x = translation[0]
            self.y = translation[1]
            euler = tf.transformations.euler_from_quaternion(rotation)
            self.theta = euler[2]
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass

        # logs the current mode
        if not(self.last_mode_printed == self.mode):
            print('-------------Current Mode---------------')
            rospy.loginfo("Current Mode: %s", self.mode)
            print('----------------------------------------')
            self.last_mode_printed = self.mode

        # checks which mode it is in and acts accordingly
        if self.mode == Mode.IDLE:
            # send zero velocity
            self.stay_idle()
            if not self.exploring:
                print('YAY WE ARE DONE!')

        elif self.mode == Mode.POSE:
            # moving towards a desired pose
            if self.close_to(self.x_g,self.y_g,self.theta_g,POS_EPS):
                self.mode = Mode.IDLE
            else:
                self.go_to_pose()

        elif self.mode == Mode.STOP:
            # at a stop sign
            if self.has_stopped():
                self.init_crossing()
            else:
                self.stay_idle()

        elif self.mode == Mode.CROSS:
            # crossing an intersection
            if self.has_crossed():
                self.mode = Mode.NAV
            else:
                self.nav_to_pose()

        elif self.mode == Mode.NAV:

            if self.close_to(self.x_g,self.y_g,self.theta_g,POS_EPS):
                # waypoint reached

                if self.close_to(self.firestation_x,self.firestation_y,self.firestation_theta,3*POS_EPS):
                    # at firestation
                    if (self.exploring):
                        self.wait_for_instr()
                    else:
                        if np.any(self.to_rescue):
                            self.init_rescue()
                        else:
                            self.mode = Mode.IDLE

                else:
                    # non-firestation waypoint reached
                    if (self.exploring):
                        pass
                    else:
                        self.init_rescue()

            else:
                # waypoint not yet reached
                self.nav_to_pose()
                pass

        elif self.mode == Mode.WAIT_FOR_INSTR:
            pass

        elif self.mode == Mode.RESCUE:
            # save the animal!
            if self.has_rescued():
                self.update_waypoint()
            else:
                pass

        else:
            raise Exception('This mode is not supported: %s'
                % str(self.mode))

    def run(self):
        rate = rospy.Rate(10) # 10 Hz
        while not rospy.is_shutdown():
            self.loop()
            rate.sleep()

if __name__ == '__main__':
    sup = Supervisor()
    sup.run()
