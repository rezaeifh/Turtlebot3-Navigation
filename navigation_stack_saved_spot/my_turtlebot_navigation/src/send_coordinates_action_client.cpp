#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <my_turtlebot_navigation/MyCustomServiceMessage.h>
#include <nav_msgs/Odometry.h>
#include <deque>
#include <iostream>
#include <string>
#include <vector>

class TurtleBot
{
protected:

	// ROS Objects
        ros::NodeHandle nh_;
        
        //ROS Service
        ros::ServiceClient get_coordinates_service;
        my_turtlebot_navigation::MyCustomServiceMessage get_coordinates_object; 
        std::string srv_name_;
        
        //ROS subscriber
        ros::Subscriber sub;
        
        //ROS action
        actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> client;
        move_base_msgs::MoveBaseGoal goal;
        
	std::vector<std::string> result_list;
	std::deque<std::string> pose_list = {"corner1", "corner2", "pedestrian"};
	float velo;
	std::string result;

public:
      TurtleBot(std::string name): client("move_base",true), srv_name_(name){  
        
	
        // wait for the service
	ros::service::waitForService(name);
        get_coordinates_service = nh_.serviceClient<my_turtlebot_navigation::MyCustomServiceMessage>(name);
        ros::Duration(0.5).sleep();
        
        //subscriber
        sub = nh_.subscribe<nav_msgs::Odometry>("/odom", 1000, boost::bind(&TurtleBot::vel_subs_callback, this, _1));

        // Action client
        client.waitForServer();
        
    };
    
   
   ~TurtleBot(void)
   {
   }
   
   void vel_subs_callback(const nav_msgs::Odometry::ConstPtr& vel)
   {
       velo = vel->twist.twist.linear.x;
   }
    void send_coordinate()
    {

        // Construct a list of coordinates
        if (velo < 0.001)
        {
            ros::service::waitForService("/get_coordinates");
            get_coordinates_object.request.label = pose_list.at(0);
            get_coordinates_service.call(get_coordinates_object);
            pose_list.pop_front();
            result = get_coordinates_object.response.goalpose;
            this->tokenize(result, ',', result_list);
            
        }
        
        // Send goal
        this->get_goal();
        sleep(2);
        client.sendGoal(goal, boost::bind(&TurtleBot::doneCb, this, _1, _2), boost::bind(&TurtleBot::activeCb, this), boost::bind(&TurtleBot::feedbackCb, this, _1));
        
        bool finished_before_timeout = client.waitForResult(ros::Duration(30.0));

        if (pose_list.size() == 0)
        {
            ROS_INFO("Mission completed");
            ros::shutdown();
        }
     }           
                
    void tokenize(std::string const &str, const char delim,
            std::vector<std::string> &out)
    {
        out.clear();
        size_t start;
        size_t end = 0;
 
        while ((start = str.find_first_not_of(delim, end)) != std::string::npos)
        {
            end = str.find(delim, start);
            out.push_back(str.substr(start, end - start));
        }
    }          

    void get_goal()
    {
    	client.waitForServer();
        // creates a goal to send to the action server
        goal.target_pose.header.frame_id = "map";
        goal.target_pose.pose.position.x = std::stof(result_list[0]);
        goal.target_pose.pose.position.y = std::stof(result_list[1]);
        goal.target_pose.pose.position.z = 0.0;
        goal.target_pose.pose.orientation.x = std::stof(result_list[2]);
        goal.target_pose.pose.orientation.y = std::stof(result_list[3]);
        goal.target_pose.pose.orientation.z = std::stof(result_list[4]);
        goal.target_pose.pose.orientation.w = std::stof(result_list[5]);
    }
    
    // Definition of the done callback.
    void doneCb(const actionlib::SimpleClientGoalState &state,
                const move_base_msgs::MoveBaseResultConstPtr &result) {
       
  	ROS_INFO("The Action has been completed");
    }
    
    // Definition of the active callback.
    void activeCb() { ROS_INFO("Goal just went active"); }
    
    // action callback function
    void feedbackCb(const move_base_msgs::MoveBaseFeedbackConstPtr& req)
    {
        ROS_INFO_STREAM("[Feedback] Going to "<<get_coordinates_object.request.label);
    }
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "path_planning_project");
  TurtleBot pointfollow("/get_coordinates");
  
  while(ros::ok())
  {
  pointfollow.send_coordinate();
  ros::spinOnce();
  }
 
  return 0;
}
