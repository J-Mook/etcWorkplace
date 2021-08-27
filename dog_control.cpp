
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "geometry_msgs/Twist.h"

#include <sstream>


int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "control");

  ros::NodeHandle nh;

  ros::Publisher control_pub = nh.advertise<geometry_msgs::Twist>("/unitree/cmd_vel", 1000);
  ros::Publisher estop_pub = nh.advertise<std_msgs::Bool>("/unitree/stop", 1000);

  ros::Rate loop_rate(10);

  int count = 0;
  while (ros::ok())
  {
    geometry_msgs::Twist dog_control_msg;
    std_msgs::Bool estop_msg;

    estop_msg.data = false;

    dog_control_msg.linear.x = 0;
    dog_control_msg.linear.y = 0;
    dog_control_msg.linear.z = 0;
    dog_control_msg.angular.x = 0;
    dog_control_msg.angular.y = 0;
    dog_control_msg.angular.z = 0;
    
    control_pub.publish(dog_control_msg);
    estop_pub.publish(estop_msg);
    ros::spinOnce();
    loop_rate.sleep();
    ++count;
  }


  return 0;
}
