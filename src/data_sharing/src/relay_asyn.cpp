#include <ros/ros.h>
#include "std_msgs/String.h"
#include <string>
#include <sstream>
#include <iostream>
#include "data_sharing/Request.h"
#include "data_sharing/Data.h"
#include "data_sharing/PacketError.h"
#include "data_sharing/DataId.h"
#include <ctime>
#include <boost/thread/thread.hpp>
#include "ros/callback_queue.h"
#include <unistd.h>
#include <fstream>
#include <random>
using namespace std;

static set<int> swarm; //存储群体机器人的id
static map<int, string> data_avail;
static vector<bool> is_lose; //default:true
static set<int> recv_req; //收到的请求
static set<int> recv_choose;
static bool get_choose = false;
static ros::CallbackQueue data_queue;

static default_random_engine random_engine(time(NULL));
static uniform_real_distribution<double> distribution(0.0, 1.0); // [0.0, 1.0)

void processReq(const data_sharing::Request::ConstPtr& msg, int come_id)
{
  if(!is_lose[come_id]) //来自编号为come_id的uav的数据没丢
  {
    int len = msg->request.size();
    for(int i=0; i<len; i++) //把接收到的请求加入recv_req中
      recv_req.insert(msg->request[i]);
  }
}

void processData(const data_sharing::Data::ConstPtr& msg, int come_id)
{
  if(!is_lose[come_id]) //来自编号为come_id的uav的数据没丢
  {
    int len = msg->identifier.size();
    for(int i=0; i<len; i++) //更新data_avail和swarm
    {
      if(!data_avail.count(msg->identifier[i])) //data_avail中没有编号为identifier[i]的数据
      {
        data_avail.insert(pair<int, string>(msg->identifier[i], msg->data[i])); //则data_avail中插入该数据
        swarm.erase(msg->identifier[i]); //则swarm中删除该编号
      }
    }
  }
}

void processPER(const data_sharing::PacketError::ConstPtr& msg, int total_uavs) //根据per判断是否丢包
{
  double random_num[total_uavs];

  int len = msg->data.size();
  for(int i=0; i<len; i++)
  {
    random_num[i] = distribution(random_engine);
    if(random_num[i] < msg->data[i])
      is_lose[i] = true; //丢包
    else
      is_lose[i] = false; //不丢包
  }
}

void chooseData(const data_sharing::DataId::ConstPtr& msg)
{
  recv_choose.clear();
  get_choose = true;
  int len = msg->id.size();
  for(int i=0; i<len; i++)
    recv_choose.insert(msg->id[i]);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "relay_asyn");

  ros::NodeHandle nh;
  ros::Publisher req_pub, data_pub, own_pub;
  ros::V_Subscriber req_sub, data_sub;
  ros::Subscriber per_sub, choose_sub;

  int robot_id;
  int total_uavs; //群体机器人群体数目
  int frequence; //消息发送频率Hz
  string address;

  ros::param::get("~robot_id", robot_id);
  ros::param::get("~total_uavs", total_uavs);
  ros::param::get("~frequence", frequence);
  ros::param::get("~address", address);

  stringstream ss, ss0;
  ss<<address<<"/con_uav"<<robot_id<<".txt"; //存储机器人达到收敛所需的轮数
  ss0<<address<<"/pb_uav"<<robot_id<<".txt"; //存储机器人传输数据数

  ofstream fout1, fout2;
  fout1.open(ss.str(), ios::out|ios::app);
  fout2.open(ss0.str(), ios::out|ios::app);

  for(int i=0; i<total_uavs; i++)
  {
    is_lose.push_back(true);
    if(i != robot_id) //初始化swarm，为群体中其他uav的id
    {
      swarm.insert(i);
    }
  }

  stringstream ss1;
  ss1<<"/uav"<<robot_id<<"/request";
  req_pub = nh.advertise<data_sharing::Request>(ss1.str(), 1000); //宣告请求

  stringstream ss2;
  ss2<<"/uav"<<robot_id<<"/data";
  data_pub = nh.advertise<data_sharing::Data>(ss2.str(), 1000); //宣告数据

  for(int i=0; i<total_uavs; i++)
  {
    if(i != robot_id)
    {
      stringstream ss3;
      ss3<<"/uav"<<i<<"/request"; //订阅请求
      req_sub.push_back(nh.subscribe<data_sharing::Request>(ss3.str(), 1000, boost::bind(processReq, _1, i)));
    }
  }

  for(int i=0; i<total_uavs; i++)
  {
    if(i != robot_id)
    {
      stringstream ss4;
      ss4<<"/uav"<<i<<"/data"; //订阅数据
      data_sub.push_back(nh.subscribe<data_sharing::Data>(ss4.str(), 1000, boost::bind(processData, _1, i)));
    }
  }

  stringstream ss5;
  ss5<<"/uav"<<robot_id<<"/packet_loss_rate";
  per_sub = nh.subscribe<data_sharing::PacketError>(ss5.str(), 100, boost::bind(processPER, _1, total_uavs)); //订阅PER

  stringstream ss6;
  ss6<<"/uav"<<robot_id<<"/own_data";
  own_pub = nh.advertise<data_sharing::DataId>(ss6.str(), 1000);

  stringstream ss7;
  ss7<<"/uav"<<robot_id<<"/choose_data";
  ros::SubscribeOptions ops = ros::SubscribeOptions::create<data_sharing::DataId>(ss7.str(), 1000, chooseData, ros::VoidPtr(), &data_queue);
  choose_sub = nh.subscribe(ops);

  stringstream send_data;
  send_data<<"This is uav "<<robot_id<<".";
  data_avail.insert(pair<int, string>(robot_id, send_data.str())); //初始化data_avail，初始值为本机的(id,data)

  bool is_conv = false; //记录是否收敛
  int cnt = 1; //记录当前通信轮数

  data_sharing::Request init_req;
  for(set<int>::iterator it=swarm.begin(); it!=swarm.end(); it++)
    init_req.request.push_back(*it);
  req_pub.publish(init_req); //发布最初请求

  data_sharing::Data init_data;
  init_data.identifier.push_back(robot_id);
  init_data.data.push_back(send_data.str());
  data_pub.publish(init_data); //发布最初数据

  ros::Rate loop_rate(frequence);

  sleep(1);
  while(ros::ok())
  {
    recv_req.clear();
    get_choose = false;

    cnt++;

    ros::spinOnce(); //处理请求，数据，per消息的回调函数

    data_sharing::DataId own_msg; //把当前拥有的数据发送给relay_control，以进行中继选择
    for(map<int, string>::iterator it=data_avail.begin(); it!=data_avail.end(); it++)
    {
      own_msg.id.push_back(it->first);
    }
    own_pub.publish(own_msg);

     while(true) //处理来自relay_control的消息
     {
       data_queue.callAvailable(ros::WallDuration());
       if(get_choose)
       {
         break;
       }
     }

    if(swarm.empty()) //如果本机已经收敛  (set为空，empty()返回true)
    {
      data_sharing::Request req_msg;
      req_pub.publish(req_msg); //发送空的request msg，实际上可以不发

      if(!is_conv) //如果是刚达到收敛状态
      {
        is_conv = true;
        cout<<"uav "<<robot_id<<" is over, and cnt is "<<cnt<<"."<<endl;

        if(fout1.is_open()) //记录收敛需要的通信轮数
        {
          fout1<<cnt<<" ";
        }
        fout1.close();
      }
    }
    else //如果本机未收敛，继续发送请求
    {
      data_sharing::Request req_msg;
      for(set<int>::iterator it=swarm.begin(); it!=swarm.end(); it++)
      {
        req_msg.request.push_back(*it);
      }
      req_pub.publish(req_msg);
    }

    //request和recv_choose的交集
    if(!recv_choose.count(-1) && recv_req.size()) //需要发送数据
    {
      data_sharing::Data data_msg;
      for(set<int>::iterator it=recv_choose.begin(); it!=recv_choose.end(); it++) //request和recv_choose的交集
      {
        if(recv_req.count(*it))
        {
          data_msg.identifier.push_back(*it);
          data_msg.data.push_back(data_avail[*it]);
        }
      }
      data_pub.publish(data_msg);
      fout2<<data_msg.identifier.size()<<" "; //记录一轮数据消息中的数据个数
    }
    else
    {
      data_sharing::Data data_msg;
      data_pub.publish(data_msg);
      fout2<<0<<" ";
    }

    fout2<<swarm.size()<<endl; //记录一轮请求消息中的请求个数

    loop_rate.sleep();
  }

  fout2<<endl;
  fout2.close();
}
