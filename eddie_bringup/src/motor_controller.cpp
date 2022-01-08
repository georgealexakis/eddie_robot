#include <cmath>
#include <iostream>
#include <ros/ros.h>
#include <std_msgs/Int16.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>
#include <nav_msgs/Odometry.h>
// Initialize variables
int gain = 600;
int leftout = 0;
int rightout = 0;
int leftoutb = 0;
int rightoutb = 0;
float left = 0;
float right = 0;
double encoderl = 0;
double encoderr = 0;
// Velocity callback
void velocityCallback(const geometry_msgs::Twist &msg)
{
  // Left wheel
  left = gain * (msg.linear.x - msg.angular.z);
  if (left > 700)
    left = 700;
  else if (left < -700)
    left = -700;
  leftoutb = (int)left;
  // Right wheel
  right = gain * (msg.linear.x + msg.angular.z);
  if (right > 700)
    right = 700;
  else if (right < -700)
    right = -700;
  rightoutb = (int)right;
}
// Velocity callback for move base
void velocityCallbackMb(const geometry_msgs::Twist &msg)
{
  // Left wheel
  left = gain * (msg.linear.x - msg.angular.z);
  if (left > 700)
    left = 700;
  else if (left < -700)
    left = -700;
  leftoutb = (int)left;
  // Right wheel
  right = gain * (msg.linear.x + msg.angular.z);
  if (right > 700)
    right = 700;
  else if (right < -700)
    right = -700;
  rightoutb = (int)right;
}
// Callback marker right
void markerrCallback(const std_msgs::Float64::ConstPtr &msg)
{
  encoderr = msg->data;
}
// Callback marker left
void markerlCallback(const std_msgs::Float64::ConstPtr &msg)
{
  encoderl = msg->data;
}
// Trigger main
int main(int argc, char **argv)
{
  // Initialize node
  ros::init(argc, argv, "motor_controller");
  ros::NodeHandle nh;
  // Publishers
  ros::Publisher pub_s1 = nh.advertise<std_msgs::Int16>("/servo1", 10);
  ros::Publisher pub_s2 = nh.advertise<std_msgs::Int16>("/servo2", 10);
  ros::Publisher pub_odom = nh.advertise<nav_msgs::Odometry>("/odom", 10);
  // Subscribers
  ros::Subscriber sub_vel = nh.subscribe("/robot/cmd_vel", 10, &velocityCallback);
  ros::Subscriber sub_mb = nh.subscribe("/cmd_vel", 10, &velocityCallbackMb);
  ros::Subscriber sub_e1 = nh.subscribe("/enc1", 10, markerrCallback);
  ros::Subscriber sub_e2 = nh.subscribe("/enc2", 10, markerlCallback);
  // Print message
  ROS_INFO("\n*********** Speed Controller Node Started ***********\n");
  // Set publish rate
  ros::Rate loop_rate(15);
  // Start publish loop
  while (ros::ok())
  {
    // Right wheel
    if (rightout + 100 < rightoutb)
    {
      rightout = rightout + 100;
    }
    else if (rightout - 100 > rightoutb)
    {
      rightout = rightout - 100;
    }
    else
    {
      rightout = rightoutb;
    }
    // Left wheel
    if (leftout + 100 < leftoutb)
    {
      leftout = leftout + 100;
    }
    else if (leftout - 100 > leftoutb)
    {
      leftout = leftout - 100;
    }
    else
    {
      leftout = leftoutb;
    }
    // Publish servo 1 messages
    std_msgs::Int16 msg;
    msg.data = leftout;
    pub_s1.publish(msg);
    // Publish servo 2 messages
    msg.data = rightout;
    pub_s2.publish(msg);
    // Lastly, spin and sleep for a while
    ros::spinOnce();
    loop_rate.sleep();
  }
  // Return 0 on exit
  return 0;
}
