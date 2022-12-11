#! /usr/bin/env python
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseResult, MoveBaseFeedback
from my_turtlebot_navigation.srv import MyCustomServiceMessage, MyCustomServiceMessageRequest

# list and sequence of positions
pose_list = ["corner1", "corner2", "pedestrian"]


class TurtleBot():
    def __init__(self):

        self.ctrl_c = False

        # Call service
        rospy.wait_for_service('/get_coordinates')
        get_coordinates_service = rospy.ServiceProxy(
            '/get_coordinates', MyCustomServiceMessage)

        self.get_coordinates_object = MyCustomServiceMessageRequest()

        # Action client
        client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)

        while not self.ctrl_c:

            # Construct a list of coordinates
            self.get_coordinates_object.label = pose_list[0]
            pose_list.remove(self.get_coordinates_object.label)
            result = get_coordinates_service(self.get_coordinates_object)
            self.result_list = list(map(float, result.goalpose.split(',')))

            client.wait_for_server()
            # Send goal
            self.get_goal()
            client.send_goal(self.goal, feedback_cb=self.feedback_callback)
            client.wait_for_result()

            rospy.sleep(2)

            if len(pose_list) == 0:
                print("Mission completed")
                self.ctrl_c = True

    def get_goal(self):
        # creates a goal to send to the action server
        self.goal = MoveBaseGoal()
        self.goal.target_pose.header.frame_id = 'map'
        self.goal.target_pose.pose.position.x = self.result_list[0]
        self.goal.target_pose.pose.position.y = self.result_list[1]
        self.goal.target_pose.pose.position.z = 0.0
        self.goal.target_pose.pose.orientation.x = self.result_list[2]
        self.goal.target_pose.pose.orientation.y = self.result_list[3]
        self.goal.target_pose.pose.orientation.z = self.result_list[4]
        self.goal.target_pose.pose.orientation.w = self.result_list[5]

    def feedback_callback(self, feedback):
        print('[Feedback] Going to ' + self.get_coordinates_object.label)


if __name__ == "__main__":
    rospy.init_node("path_planning_project")
    TurtleBot()
    rospy.spin()
