#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <my_turtlebot_navigation/MyCustomServiceMessage.h>
#include <sensor_msgs/LaserScan.h>
#include <string>


class Spotfind
{
protected:
    
    std::string file_path = "/home/rezaeifh/catkin_ws/src/Turtlebot3-Navigation/navigation_stack_saved_spot/my_turtlebot_localization/config/spots.yaml";
    
    // ROS Objects
    ros::NodeHandle nh_;

    std::string line;

public:    
    //Start a service to find the goal pose            
    ros::ServiceServer my_service = nh_.advertiseService("/get_coordinates", &Spotfind::service_callback, this);  
    
    // Service callback function
    bool service_callback(my_turtlebot_navigation::MyCustomServiceMessage::Request &request,
                         my_turtlebot_navigation::MyCustomServiceMessage::Response &my_response)
    {
        
        nh_.getParam(request.label, line);
        ROS_INFO("getting_position");
        my_response.goalpose = line;
        my_response.message = "OK";
        ROS_INFO("%s",my_response.message.c_str());

        return true;
    }
};
    
int main(int argc, char** argv)
{
  ros::init(argc, argv, "spot_finder");
  Spotfind Spotfind;
  ros::spin();
  
  return 0;
}
