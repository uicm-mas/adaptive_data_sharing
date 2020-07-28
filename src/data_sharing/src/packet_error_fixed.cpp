/*发送固定均匀丢包律*/

#include <ros/ros.h>
#include <cmath>
#include <vector>
#include <string>
#include "data_sharing/PacketError.h"
#include <fstream>
using namespace std;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "packet_error_fixed");

  ROS_INFO("Process fixed PER...");

  ros::NodeHandle nh;
  ros::V_Publisher per_pub;

  int total_robots;
  int rate;
  double per;

  ros::param::get("~total_robots", total_robots);
  ros::param::get("~rate", rate);
  ros::param::get("~per", per);

  for(int i=0; i<total_robots; i++)
  {
    stringstream ss;
    ss<<"/uav"<<i<<"/packet_loss_rate";
    per_pub.push_back(nh.advertise<data_sharing::PacketError>(ss.str(), 50));
  }

  ros::Rate loop_rate(rate);

  while(ros::ok())
  {
    for(int i=0; i<total_robots; i++)
    {
      data_sharing::PacketError msg;
      for(int j=0; j<total_robots; j++)
      {
        if(i == j)
          msg.data.push_back(0.0);
        else
          msg.data.push_back(per);
      }
      per_pub[i].publish(msg);
    }

    loop_rate.sleep();
  }
}
