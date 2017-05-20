#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include <sstream>


//std::string str;


/**
void turtle_Callback(geometry_msgs::Twist::ConstPtr& vel)
{
 //geometry_msgs::Twist new_vel = *vel;
 //ROS_INFO("I heard: [%d]", new_vel.linear.x);
 ROS_INFO("AHA");

}
*/

int main(int argc, char **argv)
{

  ros::init(argc, argv, "test_talker");

  ros::NodeHandle n;

  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("leap_dir", 1000);

//  ros::Subscriber turtle_sub = n.subscribe("turtle1/cmd_vel", 10, turtle_Callback);


  ros::Rate loop_rate(10);


  int count = 0;
  while (ros::ok())
  {

    std_msgs::String msg;

    std::stringstream ss;
    ss << "forward";
    msg.data = ss.str();

    ROS_INFO("%s", msg.data.c_str());


    chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }


  return 0;
}
