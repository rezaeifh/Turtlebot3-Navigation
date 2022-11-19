#! /usr/bin/env python

import math
import rospy
import actionlib
from follow_wall.msg import OdomRecordAction, OdomRecordResult, OdomRecordFeedback
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point


class RecordOdomClass():

    def __init__(self):
        # create the action server
        self._as = actionlib.SimpleActionServer(
            "record_odom", OdomRecordAction, self.goal_callback, False)

        # create subscriber
        self.sub_odom = rospy.Subscriber(
            "/odom", Odometry, self.odometry_callback)
        self._feedback = OdomRecordFeedback()
        self._result = OdomRecordResult()
        self._as.start()

        # helper variables
        self._result.list_of_odoms = []
        self.pose_list = Point()
        self.pose_list.x = []
        self.pose_list.y = []
        self.pose_list.z = []
        self.x_pose = 0
        self.y_pose = 0
        self.theta = 0

    # subscriber callback function
    def odometry_callback(self, request):
        self.x_pose = request.pose.pose.position.x
        self.y_pose = request.pose.pose.position.y
        self.theta = request.pose.pose.orientation.z

    # action server callback function
    def goal_callback(self, goal):

        # this callback is called when the action server is called.
        # helper variables
        r = rospy.Rate(1)
        success = True
        init_x = self.x_pose
        init_y = self.y_pose
        total_distance = 0

        while total_distance < 5.4:

            if self._as.is_preempt_requested():
                rospy.loginfo("Pre-emption called ...")
                success = False
                self._as.set_preempted()
                self._as.set_aborted()
                break

            total_distance = math.sqrt(
                (self.x_pose-init_x)**2 + (self.y_pose-init_y)**2) + total_distance
            self._feedback.current_total = total_distance
            init_x = self.x_pose
            init_y = self.y_pose
            self._as.publish_feedback(self._feedback)
            self.pose_list.x.append(self.x_pose)
            self.pose_list.y.append(self.y_pose)
            self.pose_list.z.append(self.theta)
            self._result.list_of_odoms.append(self.pose_list)

            r.sleep()

        if success:
            self._result.list_of_odoms = self.pose_list
            rospy.loginfo('successful')
            rospy.loginfo(self._result.list_of_odoms)
            self._as.set_succeeded()


if __name__ == '__main__':
    rospy.init_node('record_odom_node')
    RecordOdomClass()
    rospy.spin()
