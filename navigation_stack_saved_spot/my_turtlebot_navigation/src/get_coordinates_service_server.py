#!/usr/bin/env python3
import yaml
import rospy
from my_turtlebot_navigation.srv import MyCustomServiceMessage, MyCustomServiceMessageResponse
from geometry_msgs.msg import Pose

file_path = "/home/rezaeifh/catkin_ws/src/Turtlebot3-Navigation/navigation_stack_saved_spot/my_turtlebot_localization/config/spots.yaml"


class Spotfind():

    def __init__(self):

        # Start a service to find the goal pose
        rospy.Service('/get_coordinates', MyCustomServiceMessage,
                      self.service_callback)

    # Service callback function
    def service_callback(self, request):

        my_response = MyCustomServiceMessageResponse()

        # It finds the label and separates corresponding position
        with open(file_path, "r") as f:
            docs = yaml.load_all(f, Loader=yaml.FullLoader)
            for x in docs:
                for k, v in x.items():
                    if k == request.label:
                        rospy.loginfo("getting position")
                        my_response.goalpose = v

        my_response.message = ("OK")
        rospy.loginfo(my_response.message)

        return my_response


if __name__ == "__main__":
    rospy.init_node("spot_finder")
    Spotfind()
    rospy.spin()
