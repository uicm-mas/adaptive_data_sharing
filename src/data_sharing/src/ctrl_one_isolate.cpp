#include <ros/ros.h>
#include <sstream>
#include <iostream>
#include "geometry_msgs/Twist.h"
using namespace std;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ctrl_one_isolate");
  ros::NodeHandle nh;
  ros::Publisher vel_pub;

  int robot_id;
  double x_direct, y_direct;
  ros::param::get("~robot_id", robot_id);
  ros::param::get("~x_direct", x_direct);
  ros::param::get("~y_direct", y_direct);

  stringstream ss;
  ss<<"/uav"<<robot_id<<"/cmd_vel";
  vel_pub = nh.advertise<geometry_msgs::Twist>(ss.str(), 1000);

  int frequence = 10;
  ros::Rate loop_rate(frequence);

  geometry_msgs:: Twist upmsg;
  upmsg.linear.z = 0.5;
  for(int i=0;i<5;i++)
  {
    upmsg.linear.z = 1;//0.5
    vel_pub.publish(upmsg);
    loop_rate.sleep();
  }

  geometry_msgs:: Twist msg;
  msg.linear.x = x_direct * 1.2;
  msg.linear.y = y_direct * 1.2;
  msg.linear.z = 0.0;

  geometry_msgs:: Twist stop_msg;
  stop_msg.linear.x = 0.0; //turn left
  stop_msg.linear.y = 0.0;
  stop_msg.linear.z = 0.0;

  int cnt = 1;
  while (ros::ok())
  {
    if(robot_id == 0)
      cout<<cnt<<endl;

    if(cnt < 90)
    {
      vel_pub.publish(msg);
    } 
    else 
    {
      if(robot_id == 0 && cnt < 120)
      {
        vel_pub.publish(msg);
      }  
      else {
        vel_pub.publish(stop_msg);
      }
    }

    cnt++;

    loop_rate.sleep();
  }
}
