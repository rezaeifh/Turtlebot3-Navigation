#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <follow_wall_cpp/FindWall.h>
#include <sensor_msgs/LaserScan.h>

class NearWallClass
{
    protected:
    
    // ROS Objects
    ros::NodeHandle nh_;
    
    public:    
    //create subscriber object
    ros::Subscriber sub = nh_.subscribe<sensor_msgs::LaserScan>("/scan", 1000, boost::bind(&NearWallClass::pose_callback, this, _1));
            
    //create publisher object
    ros::Publisher pub = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    // ROS Messages
    geometry_msgs::Twist pose_object;
           
    //create service server            
    ros::ServiceServer wall_service = nh_.advertiseService("/find_wall", &NearWallClass::my_callback, this);
            
    float front;
    float second_dis;
    float minimum_range;
    int min_index;
    
    void pose_callback(const sensor_msgs::LaserScan::ConstPtr &dis) {
    
        //find the min distance from the wall to right hand side of the robot
        
        auto range_min = std::min_element(dis->ranges.begin(), dis->ranges.end());
        int min_ind = std::distance(std::begin(dis->ranges), range_min);
        
        minimum_range = range_min[0];
        
        min_index = min_ind;
        front = dis->ranges[390];
        
        second_dis = dis->ranges[150];
	}
        
        bool my_callback(follow_wall_cpp::FindWall::Request &request,
                         follow_wall_cpp::FindWall::Response &srv_response)
        {
        
        srv_response.wallfound = false;
        double Tol = 0.04;
        
        // turning toward wall
        while (front > minimum_range + Tol) 
        {
        ROS_INFO_ONCE("rotating toward wall");
        	if (min_index > 360)
        	{
        	pose_object.linear.x = 0;
        	pose_object.angular.z = 0.15;
        	}
            	else
               {
               pose_object.linear.x = 0;
               pose_object.angular.z = -0.15;
               }
          pub.publish(pose_object);
          
          ros::spinOnce();
        }
               
        
        
        //approaching the wall
        while (front > 0.28)
        {
        
        ROS_INFO_ONCE("approaching the wall");
        pose_object.linear.x = 0.02;
        pose_object.angular.z = 0;
        pub.publish(pose_object);
        ros::spinOnce();
        }
        
        //turning toward initial position
        while (second_dis > minimum_range + Tol)
        {
        ROS_INFO_ONCE("rotating to iniial position");
        pose_object.linear.x = 0;
        pose_object.angular.z = 0.15;
        pub.publish(pose_object);
        ros::spinOnce();
        }

        //stop after service is complete
        pose_object.linear.x = 0;
        pose_object.angular.z = 0;
        pub.publish(pose_object);
        ROS_INFO("%s", std::to_string(front).c_str());
        srv_response.wallfound = true;
        return true;
     }
 };


int main(int argc, char** argv)
{
  ros::init(argc, argv, "find_nearest_wall_node");
  NearWallClass near_object;
  ros::spin();
  
  return 0;
}
