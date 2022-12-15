#include <ros/ros.h>
#include <cmath>
#include <actionlib/server/simple_action_server.h>
#include <follow_wall_cpp/OdomRecordAction.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Point.h>

class OdomRecordas
{
protected:
  
  // NodeHandle instance
  ros::NodeHandle nh_;
  
  // Action server
  actionlib::SimpleActionServer<follow_wall_cpp::OdomRecordAction> as_;
  std::string action_name_;
  follow_wall_cpp::OdomRecordFeedback feedback_;
  follow_wall_cpp::OdomRecordResult result_;
  
  // Subscriber
  ros::Subscriber sub_odom;
  
  
  //ros::Rate rate_;
  geometry_msgs::Point pose_list;
  float x_pose;
  float y_pose;
  float theta;
  
  

public:
  OdomRecordas(std::string name):
  
  //create action server
    as_(nh_, name, boost::bind(&OdomRecordas::goal_callback, this, _1), false),
    action_name_(name)
  {
    as_.start();
    
    //rate_ = ros::Rate(1);
    // create subscriber
    sub_odom = nh_.subscribe<nav_msgs::Odometry>("/odom", 1000, boost::bind(&OdomRecordas::odometry_callback, this, _1));
  };
  
  
  // helper variables
  //ros::Rate rate_ = ros::Rate(1);
    ~OdomRecordas(void)
  {
  }
  //subscriber callback function
  void odometry_callback(const nav_msgs::Odometry::ConstPtr &request) {
    x_pose = request->pose.pose.position.x;
    y_pose = request->pose.pose.position.y;
    theta = request->pose.pose.orientation.z;
    }



  // action server callback function
  void goal_callback(const follow_wall_cpp::OdomRecordGoal::ConstPtr &goal)
  {
    // helper variables
    bool success = true;
    float init_x = x_pose;
    float init_y = y_pose;
    float total_distance = 0.00;
    
    while (total_distance < 5.40)
    {
      if (as_.isPreemptRequested() || !ros::ok())
      {
        ROS_INFO("%s: Preempted", action_name_.c_str());
        // set the action state to preempted
        as_.setAborted();
        success = false;
        break;
      }
      
      total_distance = sqrt(pow((x_pose-init_x),2) + pow((y_pose-init_y),2)) + total_distance;
      
      feedback_.current_total = total_distance;
      
      init_x = x_pose;
      init_y = y_pose;
      
      as_.publishFeedback(feedback_);
      
      pose_list.x = x_pose;
      pose_list.y = y_pose;
      pose_list.z = theta;
      result_.list_of_odoms.push_back(pose_list);
      
      ros::spinOnce();
      //rate_.sleep();
      sleep(1);
      
      }
    
    
    if(success)
    {
      ROS_INFO("%s: Succeeded", action_name_.c_str());
      // set the action state to succeeded
      ROS_INFO("successful");
      as_.setSucceeded(result_);
    }
  }
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "record_odom_node");
  OdomRecordas record_odom("record_odom");
  while (ros::ok())
  {
  ros::spinOnce();
  }
  
  return 0;
}
