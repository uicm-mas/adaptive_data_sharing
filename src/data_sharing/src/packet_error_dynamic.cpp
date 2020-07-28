/*根据机器人的动态位置计算并发送丢包律*/

#include <ros/ros.h>
#include <cmath>
#include <ctime>
#include <vector>
#include <string>
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include "data_sharing/PacketError.h"
#include "data_sharing/Connection.h"
#include <visualization_msgs/Marker.h>
#include <fstream>
using namespace std;

//信道模型参数
const double Ptx = 0.05;
const double N0 = 1e-12;
const double B = 10e6;
const double Betta = 2.68;

const double an = 90.2514;
const double gn = 3.4998;
const double ypn = 1.2865; //1e1.0942;

struct XYZ
{
  double x;
  double y;
  double z;
};

class Robot
{
public:
  ros::Subscriber sub;
  ros::Publisher pub;
  int robot_id;
  XYZ position;

  Robot(int id):robot_id(id)
  {
    ros::NodeHandle nh;
    stringstream ss;
    ss<<"/uav"<<robot_id<<"/ground_truth/state";
    sub = nh.subscribe<nav_msgs::Odometry>(ss.str(), 100, &Robot::getPosition, this);

    stringstream ss1;
    ss1<<"/uav"<<robot_id<<"/packet_loss_rate";
    pub = nh.advertise<data_sharing::PacketError>(ss1.str(), 100);

    position.x = position.y = position.z = 0.0;
  }

  void getPosition(const nav_msgs::Odometry::ConstPtr& lmsg)
  {
    position.x = lmsg->pose.pose.position.x;
    position.y = lmsg->pose.pose.position.y;
    position.z = lmsg->pose.pose.position.z;
  }

};

static vector<Robot*> robot_list;

bool equal(double a, double b)
{
  if((a - b > -0.0000001) && (a - b < 0.0000001))
  {
    return true;
  }
  else
  {
    return false;
  }
}

double getDistanceBetta(int i, int j)
{
  XYZ point1 = robot_list[i]->position;
  XYZ point2 = robot_list[j]->position;

  double distance = sqrt(pow((point1.x-point2.x),2) + pow((point1.y-point2.y),2) + pow((point1.z-point2.z),2));
  distance = pow(distance, Betta);
  return distance;
}

double getPackageRate(int i, int j)
{
  double package_error_rate = 0;
  double r = Ptx / (getDistanceBetta(i, j) * N0 * B);
  if(r>0 && r<ypn)
  {
    package_error_rate = 1;
  }
  else if(r>ypn || equal(r,ypn))
  {
    package_error_rate = an * exp(-1.0*gn*r);
  }
  return package_error_rate;
}

void calculating(int total_robots, vector<vector<double> >& packet_loss_rate)
{
  for(int i=0; i<total_robots; i++)
  {
    for(int j=0; j<total_robots; j++)
    {
      packet_loss_rate[i][j] = 0;
    }
  }

  for(int i=0; i<total_robots; i++)
  {
    for(int j=0; j<total_robots; j++)
    {
      if(j<=i)
      {
        packet_loss_rate[i][j] = packet_loss_rate[j][i];
      }
      else
      {
         packet_loss_rate[i][j] = getPackageRate(i, j);
      }
    }
  }
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "packet_error_dynamic");
  ROS_INFO("Process dynamic PER...");

  int rate;
  int total_robots;
  ros::NodeHandle nh;

  ros::param::get("~rate", rate);
  ros::param::get("~total_robots", total_robots);

  vector<vector<double> > packet_loss_rate(total_robots, vector<double>(total_robots)); //原始丢包率

  for(int i=0; i<total_robots; i++)
  {
    Robot *p = new Robot(i);
    robot_list.push_back(p);
  }

  ros::Rate loop_rate(rate);

  while(ros::ok())
  {
    ros::spinOnce();

    calculating(total_robots, packet_loss_rate);

    for(int i=0; i<total_robots; i++)
    {
      data_sharing::PacketError msg;
      for(int j=0; j<total_robots; j++)
      {
        msg.data.push_back(packet_loss_rate[i][j]);
      }
      robot_list[i]->pub.publish(msg);
    }

    loop_rate.sleep();
  }

}
