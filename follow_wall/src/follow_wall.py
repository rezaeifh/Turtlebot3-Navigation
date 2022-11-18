#! /usr/bin/env python

# import usefull packages
import rospy
import actionlib
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from find_wall.srv import FindWall, FindWallRequest
from find_wall.msg import OdomRecordAction, OdomRecordGoal


class TrackingClass():

    def __init__(self):

        ##############################
        ##  Call find_wall service  ##
        ##############################

        # wait for the find_wall service
        rospy.wait_for_service('/find_wall')

        # create find_wall service
        self.find_wall_service_call = rospy.ServiceProxy(
            '/find_wall', FindWall)
        self.find_wall_object = FindWallRequest()

        # call find_wall service
        rospy.loginfo("The Service find_wall has been called")

        self.service_client_result = self.find_wall_service_call(
            self.find_wall_object)

        rospy.loginfo(self.service_client_result)
        rospy.loginfo("Successfully found the wall")

        #######################################
        ##  Call record_odom action service  ##
        #######################################

        # create action client
        self.action_client = actionlib.SimpleActionClient(
            '/record_odom', OdomRecordAction)
        self.action_client.wait_for_server()
        self.goal = OdomRecordGoal()
        self.action_client.send_goal(self.goal, feedback_cb=self.odom_callback)

        #############################
        ##  Start the subscribers  ##
        #############################

        self.sub = rospy.Subscriber(
            '/scan', LaserScan, self.pose_subs_callback)

        ###########################
        ##  Start the publisher  ##
        ###########################

        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.pose_object = Twist()  # Twist Object

        self.ctrl_c = False
        rospy.on_shutdown(self.shutdownhook)

        self.front = 0
        self.right = 0

    # shutdown condition (the robot will stop)
    def shutdownhook(self):
        self.ctrl_c = True
        self.pose_object.linear.x = 0
        self.pose_object.angular.z = 0
        self.pub.publish(self.pose_object)
        rospy.loginfo("program is shutting down")

    # action callback function
    def odom_callback(self, req):
        rospy.loginfo("Total distance is: " + str(req.current_total))

    # subscriber callback function
    def pose_subs_callback(self, dis: LaserScan):

        rospy.loginfo_once("The robot is following the wall")

        Tol = 0.04
        straight = dis.ranges[150]

        # find the min distance from the wall to right hand side of the robot
        range_min = min(dis.ranges[0:180])
        # min_index = dis.ranges.index(min(dis.ranges[0:360]))
        self.right = range_min

        # the distance of the front wall
        self.front = dis.ranges[360]

        # rospy.loginfo("Min index from right hand side is " + str(min_index) + "real:" + str(min_min))

        # find the state of the action
        state_result = self.action_client.get_state()

        # main part of the code (wall follower)
        if self.ctrl_c is False:  # check if the the user did not cancel the process)

            if state_result < 2:
                # check if wall is in front of the robot
                if self.front < 0.5:
                    rospy.loginfo("wall is in front of the robot")
                    self.pose_object.linear.x = 0.05
                    self.pose_object.angular.z = 1
                    self.pub.publish(self.pose_object)

                # check the robot is too close to the wall
                elif range_min < 0.22:
                    rospy.loginfo("robot is too close to the wall")
                    self.pose_object.linear.x = 0.009/(0.25 - range_min)
                    self.pose_object.angular.z = 0.6*(0.25 - range_min)
                    self.pub.publish(self.pose_object)

                # check the robot is too far from the wall
                elif range_min > 0.3:
                    rospy.loginfo("robot is far from the wall")
                    self.pose_object.linear.x = 0.7*(range_min - 0.25)
                    self.pose_object.angular.z = 1.5*(0.25 - range_min)
                    self.pub.publish(self.pose_object)

                else:
                    rospy.loginfo("perfect distance from the wall")
                    if range_min < 0.25:
                        self.pose_object.linear.x = 0.05
                        if straight > range_min + Tol:
                            self.pose_object.angular.z = -0.09
                        else:
                            self.pose_object.angular.z = 0

                    elif range_min > 0.26:
                        self.pose_object.linear.x = 0.05

                        if straight > range_min + Tol:
                            self.pose_object.angular.z = 0.01
                        else:
                            self.pose_object.angular.z = 0

                    else:
                        self.pose_object.linear.x = 0.06
                        self.pose_object.angular.z = 0

                    self.pub.publish(self.pose_object)

            # robot will stop after action is complete
            elif state_result >= 2:
                self.pose_object.linear.x = 0
                self.pose_object.angular.z = 0
                self.pub.publish(self.pose_object)
                self.action_client.wait_for_result()


if __name__ == '__main__':
    rospy.init_node('follow_wall_node')
    track_object = TrackingClass()
    rospy.spin()
