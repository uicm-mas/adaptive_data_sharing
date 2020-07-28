//计算平均收敛轮数和平均总消息总负载

#include <ros/ros.h>
#include <fstream>
using namespace std;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "average");

  int total_uavs; //机器人总数
  int times; //算法运行次数
  string address;

  ros::param::get("~total_uavs", total_uavs);
  ros::param::get("~times", times);
  ros::param::get("~address", address);

  double cnt_sum = 0.0;
  int data;

  ifstream fin;

  for(int i=0; i<total_uavs; i++)
  {
    stringstream ss;
    ss<<address<<"/con_uav"<<i<<".txt"; 
    fin.open(ss.str(), ios::in);

    for(int j=0; j<times; j++)
    {
      fin>>data;
      cnt_sum += data;
    }

    fin.close();
  }
  cnt_sum /= 1.0 * times * total_uavs;

  cout<<address<<":"<<endl;
  cout<<"average con is: "<<cnt_sum<<endl;


  int data_bytes = 4 + 4;
  int req_bytes = 2;
  double sum[total_uavs];
  double ave = 0.0;
  int data_num, req_num;

  for(int i=0; i<total_uavs; i++) //处理第一轮的负载
    sum[i] = 1 * data_bytes + (total_uavs-1) * req_bytes;

  for(int i=0; i<total_uavs; i++)
  {
    stringstream ss;
    ss<<address<<"/pb_uav"<<i<<".txt"; 
    fin.open(ss.str(), ios::in);

    while(!fin.eof())
    {
      fin>>data_num;
      fin>>req_num;
      sum[i] += data_num * data_bytes + req_num * req_bytes; //处理剩下每轮的负载
    }
    fin.close();
    sum[i] /= 1.0 * times;
    ave += sum[i];
  }
  ave = ave / total_uavs;
  cout<<"average pb is: "<<ave<<endl;

}




