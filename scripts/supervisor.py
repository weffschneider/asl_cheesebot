#!/usr/bin/env python

import rospy
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import Float32MultiArray, String, Bool
from geometry_msgs.msg import Twist, PoseArray, Pose2D, PoseStamped
from asl_turtlebot.msg import DetectedObject
import tf
import math
from enum import Enum

# threshold at which we consider the robot at a location
POS_EPS = .1
THETA_EPS = .3

# time to stop at a stop sign
STOP_TIME = 3

# minimum distance from a stop sign to obey it
STOP_MIN_DIST = .5

# time taken to cross an intersection
CROSSING_TIME = 3

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

        # assume we start at the fire station
        # TODO: is this right?
        self.firestation_x = self.x
        self.firestation_y = self.y
        self.firestation_theta = self.theta

        # current mode
        self.mode = Mode.IDLE
        self.last_mode_printed = None

        # flags for control flow
        self.exploring = True

        # create publishers
        self.nav_goal_publisher = rospy.Publisher('/cmd_nav', Pose2D, queue_size=10)
        self.pose_goal_publisher = rospy.Publisher('/cmd_pose', Pose2D, queue_size=10)
        self.cmd_vel_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.rescue_ready = rospy.Publisher('/ready_to_rescue', Bool, queue_size=1)
        
        # TODO: add Publisher for ready_to_rescue message

        # create subscribers
        rospy.Subscriber('/detector/stop_sign', DetectedObject, self.stop_sign_detected_callback)
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.rviz_goal_callback)
        rospy.Subscriber('/rescue_on', Bool, self.rescue_on_callback)
        
        # TODO: add Subscriber for animal detection: self.animal_detected_callback
        # TODO: add Subscriber for rescue_on message: self.rescue_on_callback

        self.trans_listener = tf.TransformListener()
        # TODO: publish animal locations to transform tree

        ##### For animal detection
        # num. of animals (get updated)
        self.num_animals = 0
        self.animal_poses = []

        # flag for detection
        self.cat_detected = False
        self.dog_detected = False

        # detector labels in tfmodels/coco_labels.txt
        # published by "camera_common" in "detector.py"
        rospy.Subscriber('/detector/cat', DetectedObject, self.animal_detected_callback)
        rospy.Subscriber('/detector/dog', DetectedObject, self.animal_detected_callback)

    def animal_detected_callback(self, msg):
        # call back for when the detector has found an animal - cat or dog -

        # record the animal's position w.r.t. the robot's pose
        if msg.name == 'cat' and not self.cat_detected:
            self.animal_poses.append( self.record_animal_frame(msg) )
            self.cat_detected = True
            self.num_animals += 1

        elif msg.name == 'dog' and not self.dog_detected:
            self.animal_poses.append( self.record_animal_frame(msg) )
            self.dog_detected = True
            self.num_animals += 1

    def record_animal_frame(self, msg):
        # OUT : [x, y, theta] of animal position in world frame

        # msg type : DetectedObject
        name = msg.name
        distance   = msg.distance
        thetaleft  = msg.thetaleft
        thetaright = msg.thetaright

        # animal location in robot's frame
        theta = (thetaleft + thetaright)/2.0

        # frame name depends on what animal we found
        frame_name = name + "_pose"

        # add frame to "animal_pose" from "base_footprint"
        br = tf.TransformBroadcaster()
        br.sendTransform( (distance*np.cos(theta), distance*np.sin(theta), theta),
                          (0., 0., 0., 1.),
                          rospy.Time.now(),
                          frame_name,
                          "base_footprint")

        # get position in world frame
        try:
            trans_ = self.trans_listener.lookupTransform('/map', frame_name, rospy.Time(0))
            animal_pose = (trans[0], trans_[1])
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass

            return animal_pose

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
        dist = msg.distance

        # if close enough and in nav mode, stop
        if dist > 0 and dist < STOP_MIN_DIST and self.mode == Mode.NAV:
            self.init_stop_sign()

    def animal_detected_callback(self, msg):
        """ callback for when the detector has found animal. """

        # distance to the animal
        dist = msg.distance

        # TODO: record animal position in some waypoint datatype
        # NOTE: we don't need to change modes here, right?

        # TODO: send message that we found it, for debugging

    def rescue_on_callback(self, msg):
        """ callback for when we receive a rescue_on message from the fire station """

        # TODO: not sure about the message type here
        rescue_on = msg.data

        if rescue_on and self.mode == Mode.WAIT_FOR_INSTR:
            self.update_waypoint()
            self.mode = NAV
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
        self.cmd_vel_publisher.publish(vel_g_msg)

    def close_to(self,x,y,theta):
        """ checks if the robot is at a pose within some threshold """

        return (abs(x-self.x)<POS_EPS and abs(y-self.y)<POS_EPS and abs(theta-self.theta)<THETA_EPS)

    def init_stop_sign(self):
        """ initiates a stop sign maneuver """

        self.stop_sign_start = rospy.get_rostime()
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
        self.mode = Mode.RESCUE

    def has_rescued(self):
        """ checks if rescue is over"""
        return (self.mode == Mode.RESCUE and (rospy.get_rostime()-self.rescue_start)>rospy.Duration.from_sec(RESCUE_TIME))

    def wait_for_instr(self):
        """ sends ready_to_rescue message, wait for response """
        # TODO: publish message
        self.rescue_ready.publish(True)

    def update_waypoint(self):
        # Update goal pose (x_g, y_g, theta_g), then switch to NAV mode to get there

        # TODO: fill me in with useful stuff

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
            rospy.loginfo("Current Mode: %s", self.mode)
            self.last_mode_printed = self.mode

        # checks wich mode it is in and acts accordingly
        if self.mode == Mode.IDLE:
            # send zero velocity
            self.stay_idle()

        elif self.mode == Mode.POSE:
            # moving towards a desired pose
            if self.close_to(self.x_g,self.y_g,self.theta_g):
                self.mode = Mode.IDLE
            else:
                self.go_to_pose()

        elif self.mode == Mode.STOP:
            # at a stop sign
            if self.has_stopped():
                self.init_crossing()
            else:
                pass

        elif self.mode == Mode.CROSS:
            # crossing an intersection
            if self.has_crossed():
                self.mode = Mode.NAV
            else:
                self.nav_to_pose()

        elif self.mode == Mode.NAV:

            if self.close_to(self.x_g,self.y_g,self.theta_g):
                # waypoint reached

                if self.close_to(self.firestation_x,self.firestation_y,self.firestation_theta):
                    # at firestation
                    if (self.exploring):
                        self.mode = Mode.WAIT_FOR_INSTR
                        self.wait_for_instr()
                    else:
                        self.mode = IDLE

                else:
                    # non-firestation waypoint reached
                    if (self.exploring):
                        self.update_waypoint()
                    else:
                        self.init_rescue()

            else:
                # waypoint not yet reached
                self.nav_to_pose()

        elif self.mode == Mode.WAIT_FOR_INSTR:
            pass

        elif self.mode == Mode.RESCUE:
            # save the animal!
            if self.has_rescued():
                self.update_waypoint()  # TODO: maybe want separate update_waypoint function for different places?like
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
like