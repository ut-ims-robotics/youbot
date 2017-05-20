#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include <iostream>
#include <string>

/**
 * This node subscribes to the a topic of leap motion controller and publishes messages to move the base od the youbot
 */

ros::Publisher mv_pub;

geometry_msgs::Twist command_move_base;
std::string str;
int cnt = 0;
int lv = 0;

void leap_dirCallback(const std_msgs::String::ConstPtr& msg)
{
 //ROS_INFO("I heard: [%s]", msg->data.c_str());
 str = msg->data.c_str();
  
 // ROS_INFO("CNT: [%i]", cnt); 

  cnt = cnt + 1; 
  lv = cnt; 

 if (str == "left")
  {
   ROS_INFO("I heard: [%s]", msg->data.c_str());

   ros::Rate rate(10);

   command_move_base.linear.x = 0;
   command_move_base.linear.y = 0.1;
   command_move_base.linear.z = 0;
   command_move_base.angular.x = 0;
   command_move_base.angular.y = 0;
   command_move_base.angular.z = 0;

   mv_pub.publish(command_move_base);	
   rate.sleep();
  }
  else if (str == "right") 
  {
   ROS_INFO("I heard: [%s]", msg->data.c_str());

   ros::Rate rate(10);

   command_move_base.linear.x = 0;
   command_move_base.linear.y = -0.1;
   command_move_base.linear.z = 0;
   command_move_base.angular.x = 0;
   command_move_base.angular.y = 0;
   command_move_base.angular.z = 0;

   mv_pub.publish(command_move_base);	
   rate.sleep();
  }
  else if (str == "forward") 
  {
  ROS_INFO("I heard: [%s]", msg->data.c_str());

   ros::Rate rate(10);

   command_move_base.linear.x = 0.1;
   command_move_base.linear.y = 0;
   command_move_base.linear.z = 0;
   command_move_base.angular.x = 0;
   command_move_base.angular.y = 0;
   command_move_base.angular.z = 0;

   mv_pub.publish(command_move_base);	
   rate.sleep();
  }
  else if (str == "backward") 
  {
  ROS_INFO("I heard: [%s]", msg->data.c_str());

   ros::Rate rate(10);

   command_move_base.linear.x = -0.1;
   command_move_base.linear.y = 0;
   command_move_base.linear.z = 0;
   command_move_base.angular.x = 0;
   command_move_base.angular.y = 0;
   command_move_base.angular.z = 0;

   mv_pub.publish(command_move_base);	
   rate.sleep();
  }
  else
  {
   ROS_INFO("I heard: [%s]", msg->data.c_str());

   ros::Rate rate(10);

   command_move_base.linear.x = 0;
   command_move_base.linear.y = 0;
   command_move_base.linear.z = 0;
   command_move_base.angular.x = 0;
   command_move_base.angular.y = 0;
   command_move_base.angular.z = 0;

   mv_pub.publish(command_move_base);	
   rate.sleep();
  }
  

}

void stopfunc()
{
   ROS_INFO("Stopping");
   ros::Rate rate(10);

   command_move_base.linear.x = 0;
   command_move_base.linear.y = 0;
   command_move_base.linear.z = 0;
   command_move_base.angular.x = 0;
   command_move_base.angular.y = 0;
   command_move_base.angular.z = 0;

   mv_pub.publish(command_move_base);	
   rate.sleep();
}



int main(int argc, char **argv)
{
  //Initializes ROS, and sets up a node
  ros::init(argc, argv, "youbot_move");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("leap_dir", 1, leap_dirCallback);

  mv_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel",1);

  ros::Rate rate_loop(10);

  while(ros::ok()) 
  {
//    ROS_INFO("Level: [%i]", lv);
    if(cnt > 0)
    {
     lv = lv + 1;
     if(lv - cnt > 5)
     {
	stopfunc();
        cnt = 0;
        lv = 0;	
     } 
    }


    ros::spinOnce();
    rate_loop.sleep();
  }


  return 0;
}

