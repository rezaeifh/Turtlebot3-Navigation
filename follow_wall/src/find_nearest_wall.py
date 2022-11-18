#! /usr/bin/env python

# import usefull packages
import rospy
import numpy as np
from follow_wall.srv import FindWall, FindWallResponse
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class NearWallClass():

    def __init__(self):

        # create subscriber object
        self.sub = rospy.Subscriber('/scan', LaserScan, self.pose_callback)

        # create publisher object
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.pose_object = Twist()  # Twist Object

        # create service server
        self.wall_service = rospy.Service(
            '/find_wall', FindWall, self.my_callback)

        self.front = 0
        self.second_dis = 0
        self.minimum_range = 0

    def pose_callback(self, dis: LaserScan):

        #range_min = 3.5
        #min_ind = 360

        # find the min distance from the wall to right hand side of the robot
        range_min = min(dis.ranges)
        min_ind = dis.ranges.index(range_min)

        self.minimum_range = range_min
        self.min_index = min_ind
        self.front = dis.ranges[390]
        self.second_dis = dis.ranges[150]
        #rospy.loginfo("minimum range is " + str(range_min) +  " and min index is " + str(min_ind))

    def my_callback(self, request):

        srv_response = FindWallResponse()

        # tolerance
        Tol = 0.04

        # turning toward wall
        while self.front > self.minimum_range + Tol:
            rospy.loginfo_once("rotating toward wall")
            if self.min_index > 360:

                self.pose_object.linear.x = 0
                self.pose_object.angular.z = 0.15
            else:
                self.pose_object.linear.x = 0
                self.pose_object.angular.z = -0.15

            self.pub.publish(self.pose_object)

        # approaching the wall
        while self.front > 0.28:
            rospy.loginfo_once("approaching the wall")
            self.pose_object.linear.x = 0.02
            self.pose_object.angular.z = 0
            self.pub.publish(self.pose_object)

        # turning toward initial position
        while self.second_dis > self.minimum_range + Tol:
            rospy.loginfo_once("rotating to iniial position")
            self.pose_object.linear.x = 0
            self.pose_object.angular.z = 0.15
            self.pub.publish(self.pose_object)

        # stop after service is complete
        self.pose_object.linear.x = 0
        self.pose_object.angular.z = 0
        self.pub.publish(self.pose_object)
        srv_response.wallfound = True
        return srv_response  # the service Response class


if __name__ == '__main__':
    rospy.init_node('find_nearest_wall_node')
    near_object = NearWallClass()
    rospy.spin()
