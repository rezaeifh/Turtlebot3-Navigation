#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <follow_wall_cpp/OdomRecordAction.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <follow_wall_cpp/FindWall.h>

class TrackingClass
{
protected:

	// ROS Objects
        ros::NodeHandle nh_;
        
        // ROS Publisher
        ros::Publisher pub;
        geometry_msgs::Twist pose_object;
        
        //ROS Subscriber
        ros::Subscriber sub;
        
        //ROS Service
        ros::ServiceClient find_wall_service_call; 
        std::string srv_name_;
        follow_wall_cpp::FindWall find_wall_object;
        
        
        //ROS action
        actionlib::SimpleActionClient<follow_wall_cpp::OdomRecordAction> action_client;
        
        actionlib::SimpleClientGoalState state_result = action_client.getState();
        
        
        float front;
        float right; 
        float straight;
        float range_min;
        bool done;

        

public:
  TrackingClass(std::string name): action_client("record_odom",true), srv_name_(name){
  
        //////////////////////////////
        //  Call find_wall service  //
        //////////////////////////////
        
        // wait for the service
	ros::service::waitForService(name);
        // create find_wall service
        find_wall_service_call = nh_.serviceClient<follow_wall_cpp::FindWall>(name);
        
        ros::Duration(0.5).sleep();
        if (find_wall_service_call.call(find_wall_object)){
        
        ROS_INFO("Successfully found the wall");
        }
        
        /////////////////////////////
        //  Start the subscribers  //
        /////////////////////////////
	
	sub = nh_.subscribe<sensor_msgs::LaserScan>("/scan", 1000, boost::bind(&TrackingClass::pose_subs_callback, this, _1));

        ///////////////////////////
        //  Start the publisher  //
        ///////////////////////////
	
	pub = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
	
        ///////////////////////////////////////
        //  Call record_odom action service  //
        ///////////////////////////////////////

        // create action client
        
        sleep(3);
        action_client.waitForServer(); 
        follow_wall_cpp::OdomRecordGoal goal;
        action_client.sendGoal(goal, boost::bind(&TrackingClass::doneCb, this, _1, _2), boost::bind(&TrackingClass::activeCb, this), boost::bind(&TrackingClass::feedbackCb, this, _1));
	
	
        
   }
   
   ~TrackingClass(void)
   {
   }
   
    // Definition of the done callback.
    void doneCb(const actionlib::SimpleClientGoalState &state,
                const follow_wall_cpp::OdomRecordResultConstPtr &result) {
  	ROS_INFO("The Action has been completed");
  	
  	done = true;
    }
    
    // Definition of the active callback.
    void activeCb() { ROS_INFO("Goal just went active"); }
    
    // action callback function
    void feedbackCb(const follow_wall_cpp::OdomRecordFeedbackConstPtr& req)
    {
        ROS_INFO_STREAM("Total distance is: "<< req->current_total);
    }
    
    // subscriber callback function
    void pose_subs_callback(const sensor_msgs::LaserScan::ConstPtr& dis)
    {
	
	float straight = dis->ranges[150];
        
        // find the min distance from the wall to right hand side of the robot
        float range_min = std::min_element(dis->ranges.begin(), dis->ranges.begin()+180)[0];
        right = range_min;

        // the distance of the front wall
        front = dis->ranges[360];
        
     }
     void move_robot()
     {
	 ROS_INFO_ONCE("The robot is following the wall");
	 
	 float Tol = 0.04;
	 
	 
        // main part of the code (wall follower)
        if (ros::ok())
        {
            if (done == true)
            {
        	// robot will stop after action is complete
        	pose_object.linear.x = 0;
        	pose_object.angular.z = 0;
        	pub.publish(pose_object);
        	action_client.waitForResult();
        	ros::shutdown();
            }
            else
            {
               // check if wall is in front of the robot
               if (front < 0.5)
               {
                    ROS_INFO("wall is in front of the robot");
                    pose_object.linear.x = 0.05;
                    pose_object.angular.z = 1;
                    pub.publish(pose_object);
                    
		}
               // check the robot is too close to the wall
               else if (right < 0.22)
               {
                    ROS_INFO("robot is too close to the wall");
                    pose_object.linear.x = 0.009/(0.25 - right);
                    pose_object.angular.z = 0.6*(0.25 - right);
                    pub.publish(pose_object);
		}
               // check the robot is too far from the wall
               else if (right > 0.3)
               {
                    ROS_INFO("robot is far from the wall");
                    pose_object.linear.x = 0.7*(right - 0.25);
                    pose_object.angular.z = 1.5*(0.25 - right);
                    pub.publish(pose_object);
		}
               else
               {
                    ROS_INFO("perfect distance from the wall");
                    if (right < 0.25)
                    {
                        pose_object.linear.x = 0.05;
                        if (straight > right + Tol)
                        {
                            pose_object.angular.z = -0.09;
                        }
                        else
                        {
                            pose_object.angular.z = 0;
			 }
		     }
                    else if (right > 0.26)
                    {
                        pose_object.linear.x = 0.05;

                        if (straight > right + Tol)
                        {
                            pose_object.angular.z = 0.01;
                        }
                        else
                        {
                            pose_object.angular.z = 0;
                        }
		     }
                    else
                    {
                         pose_object.linear.x = 0.06;
                         pose_object.angular.z = 0;
                    }
                    pub.publish(pose_object);
               }    
               
	    }

	}
        ros::spinOnce();
    }

 };

int main(int argc, char** argv)
{
  ros::init(argc, argv, "follow_wall_node");
  TrackingClass track_object("/find_wall");
  
  while(ros::ok())
  {
  track_object.move_robot();
  ros::spinOnce();
  }
  
  return 0;
}
