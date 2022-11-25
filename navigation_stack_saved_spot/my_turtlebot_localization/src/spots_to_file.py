#!/usr/bin/env python3

import rospkg
import rospy
from my_turtlebot_localization.srv import MyServiceMessage, MyServiceMessageResponse
from geometry_msgs.msg import PoseWithCovarianceStamped

# path of the file contains spots
rp = rospkg.RosPack()
package_path = rp.get_path('my_turtlebot_localization')
file_path = package_path + "/config/spots.yaml"

class SpotSaver():

    def __init__(self):
    
        # Start a subscriber to get the position and orientation and covariance
        rospy.Subscriber(
            '/amcl_pose', PoseWithCovarianceStamped, self.sub_callback)
        # Start a service to save the spot
        rospy.Service('/save_spot', MyServiceMessage, self.service_callback)

    # Subscriber callback function: get the position, orientation and covariance
    def sub_callback(self, msg):
        rospy.loginfo("updating position and orientation")
        self.cov_x = msg.pose.covariance[0]
        self.cov_y = msg.pose.covariance[7]
        self.pose = msg.pose.pose.position
        self.orient = msg.pose.pose.orientation

    def service_callback(self, request):

        # Calculating total covariance as a metric
        cov = (self.cov_x + self.cov_y)/2

        my_response = MyServiceMessageResponse()

        if cov < 0.65:

            with open(file_path, "a") as yamlfile:
                yamlfile.write(" %s : %.2f, %.2f, %.4f, %.4f, %.4f, %.4f \n" % (request.label, self.pose.x,
                               self.pose.y, self.orient.x, self.orient.y, self.orient.z, self.orient.w))

            my_response.navigation_successfull = True
            my_response.message = (
                "pose " + (request.label) + " is added to yaml file")
            rospy.loginfo(my_response.message)

        else:
            my_response.navigation_successfull = False
            my_response.message = (
                "Unsuccessful saving position : " + (request.label))

        return my_response


if __name__ == "__main__":
    rospy.init_node("spot_recorder")
    SpotSaver()
    rospy.spin()
