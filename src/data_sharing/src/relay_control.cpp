#include <ros/ros.h>
#include <cmath>
#include <vector>
#include <string>
#include <fstream>
#include <algorithm>
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include "data_sharing/DataId.h"
using namespace std;

const double Ptx = 0.05;
const double N0 = 1e-12;
const double B = 1e7;
const double Betta = 2.68;

const double an = 90.2514;
const double gn = 3.4998;
const double ypn = 1.2865; //1.0942

const double threshold = 1.35029; // transmission threshold for SNR.
//per=0.5,threshold = 1.48458;
//per=0.7,threshold = 1.38845;
//per=0.6,threshold = 1.43249;
//per=0.8,threshold = 1.35029;
//per=0.4,threshold = 1.54834.

const double PERth = 0.05; //for multiple relay

static vector<bool> receive;

struct XYZ
{
    double x;
    double y;
    double z;
};

struct Relay
{
    int relay_id;
    double channel_snr;
    Relay(int relay_id, double channel_snr)
    {
        this->relay_id = relay_id;
        this->channel_snr = channel_snr;
    }
};

class Robot
{
public:
    ros::NodeHandle n;
    ros::Subscriber pos_sub;
    ros::Subscriber data_sub;
    ros::Publisher data_pub;
    int robot_id;
    XYZ position;
    set<int> own_data, choose_data;

    Robot(int id):robot_id(id)
    {
        stringstream ss;
        ss<<"/uav"<<robot_id<<"/ground_truth/state";
        pos_sub = n.subscribe<nav_msgs::Odometry>(ss.str(), 5, &Robot::getPosition, this);

        stringstream ss1;
        ss1<<"/uav"<<robot_id<<"/own_data"; //每个uav已经拥有的数据
        data_sub = n.subscribe<data_sharing::DataId>(ss1.str(), 100, boost::bind(&Robot::getData, this, _1, robot_id));

        stringstream ss2;
        ss2<<"/uav"<<robot_id<<"/choose_data"; //每个uav中通过中继选择，被选择出要发送的数据
        data_pub = n.advertise<data_sharing::DataId>(ss2.str(), 100);

        position.x = position.y = position.z = 0.0;
    }

    void getPosition(const nav_msgs::Odometry::ConstPtr& lmsg)
    {
        position.x = lmsg->pose.pose.position.x;
        position.y = lmsg->pose.pose.position.y;
        position.z = lmsg->pose.pose.position.z;
    }

    void getData(const data_sharing::DataId::ConstPtr& msg, int rob_id)
    {
        if(!receive[rob_id])
            receive[rob_id] = true;

        own_data.clear();

        int len = msg->id.size();
        for(int i=0; i<len; i++)
            own_data.insert(msg->id[i]);

        cout<<"msg_len:"<<len<<endl;
    }

};

static vector<Robot*> robot_list;

bool equal(double a, double b)
{
    if((a - b > -0.0000001) && (a - b < 0.0000001))
        return true;
    else
        return false;
}

double getDistanceBetta(int i, int j)
{
    XYZ point1 = robot_list[i]->position;
    XYZ point2 = robot_list[j]->position;

    double distance = sqrt(pow((point1.x-point2.x),2) + pow((point1.y-point2.y),2) + pow((point1.z-point2.z),2));
    distance = pow(distance, Betta);
    return distance;
}

double getPackageRate(int i, int j, vector<vector<double> >& snr)
{
    double package_error_rate;
    double r = snr[i][j]; //Ptx / (getDistanceBetta(i, j) * N0 * B);
    if(r>0 && r<ypn)
        package_error_rate = 1;
    else if(r>ypn || equal(r,ypn))
        package_error_rate = an * exp(-1.0*gn*r);
    return package_error_rate;
}

void getPER(int total_robots, vector<vector<double> >& packet_loss_rate, vector<vector<double> >& snr)
{
    for(int i=0; i<total_robots; i++)
    {
      for(int j=0; j<total_robots; j++)
      {
        packet_loss_rate[i][j] = 0;
      }
    }

    for(int i=0; i<total_robots; i++)
        for(int j=0; j<total_robots; j++)
        {
            if(j<=i)
            {
                packet_loss_rate[i][j] = packet_loss_rate[j][i];
            }
            else
            {
                packet_loss_rate[i][j] = getPackageRate(i, j, snr);
            }
        }
}

double getSNR(int i, int j)
{
    XYZ point1 = robot_list[i]->position;
    XYZ point2 = robot_list[j]->position;

    double distance = sqrt(pow((point1.x-point2.x),2) + pow((point1.y-point2.y),2) + pow((point1.z-point2.z),2));
    distance = pow(distance, Betta);
    double signal_noise_ratio = 1.0 * Ptx / (distance * N0 * B);
    return signal_noise_ratio;
}

void getSNR(int total_robots, vector<vector<double> >& snr)
{
    for(int i=0; i<total_robots; i++)
    {
      for(int j=0; j<total_robots; j++)
      {
        snr[i][j] = 0;
      }
    }

    for(int i=0; i<total_robots; i++)
        for(int j=0; j<total_robots; j++)
        {
            if(j<=i)
                snr[i][j] = snr[j][i];
            else
                snr[i][j] = getSNR(i, j);
        }
}

void chooseData(int total_robots, vector<vector<double> >& snr) //single relay
{
    for(int i=0; i<total_robots; i++)
        robot_list[i]->choose_data.clear();

    for(int i=0; i<total_robots; i++) //choose a relay for source and destination, maybe none
    {
        for(int j=0; j<total_robots; j++) // i is source, j is destination
        {
            if(i == j)
                continue;
            if(robot_list[j]->own_data.count(i)) //already reached
            {
                continue;
            }
//      if(snr[i][j] > threshold) //去掉per阈值
//      {
//       robot_list[i]->choose_data.insert(i); //send from source, i.e. lazy
//      }
//      else
//      {
            set<int> relay;
            for(int k=0; k<total_robots; k++) //confirm choice for relay
            {
                if(k!=i && k!=j)
                {
                    if(robot_list[k]->own_data.count(i))
                        relay.insert(k);
                }
            }
            if(relay.size() == 0) // find no relay, must send by source
            {
                robot_list[i]->choose_data.insert(i);
            }
            else // find a relay
            {
                int relay_id = *relay.begin();
                double max_snr = snr[relay_id][j];
                set<int>::iterator it = relay.begin();
                it ++;
                for(; it!=relay.end(); it++)
                {
                    if(snr[*it][j] > max_snr)
                    {
                        relay_id = *it;
                        max_snr = snr[*it][j];
                    }
                }
                robot_list[relay_id]->choose_data.insert(i);
            }
//      }
        }
    }
}

bool cmp(const Relay& a, const Relay& b)
{
    return a.channel_snr > b.channel_snr;
}

void chooseData2(int total_robots, vector<vector<double> >& packet_loss_rate, vector<vector<double> >& snr) //multiple relay
{
    for(int i=0; i<total_robots; i++)
        robot_list[i]->choose_data.clear();

    for(int i=0; i<total_robots; i++) //choose relay for a source and destination, maybe none
    {
        for(int j=0; j<total_robots; j++) // i is source, j is destination
        {
            if(i == j)
                continue;
            if(robot_list[j]->own_data.count(i)) //already reached
            {
                continue;
            }
            if(packet_loss_rate[i][j] < 0.7)//snr[i][j] > threshold, 0.5, 0.7
            {
                robot_list[i]->choose_data.insert(i); //send from source, i.e. lazy
            }
            else
            {
                set<int> relay;
                for(int k=0; k<total_robots; k++) //confirm choice for relay
                {
                    if(k!=i && k!=j)
                    {
                        if(robot_list[k]->own_data.count(i))
                            relay.insert(k);
                    }
                }
                if(relay.size() == 0) // find no relay, must send by source
                {
                    robot_list[i]->choose_data.insert(i);
                }
                else // find a relay
                {
                    vector<Relay> relay_list;
                    for(set<int>::iterator it = relay.begin(); it!=relay.end(); it++)
                    {
                        int relay_id = *it;
                        double channel_snr = min(snr[i][relay_id], snr[relay_id][j]);
                        Relay tmp(relay_id, channel_snr);
                        relay_list.push_back(tmp);
                    }
                    sort(relay_list.begin(), relay_list.end(), cmp);
                    relay.clear();

                    double product = packet_loss_rate[i][j];
                    for(vector<Relay>::iterator it = relay_list.begin(); it!=relay_list.end(); it++)
                    {
                        int id = (*it).relay_id;
                        product *= (1 - (1-packet_loss_rate[i][id]) * (1-packet_loss_rate[id][j]));
                        relay.insert(id);
                        if(product < PERth)
                            break;
                    }
                    for(set<int>::iterator it = relay.begin(); it!=relay.end(); it++)
                    {
                        robot_list[*it]->choose_data.insert(i);
                    }
                }
            }
        }
    }
}

void pub_data(int total_robots)
{
    for(int i=0; i<total_robots; i++)
    {
        data_sharing::DataId msg;

        if(robot_list[i]->choose_data.size() == 0)
        {
            msg.id.push_back(-1);
        }

        for(set<int>::iterator it=robot_list[i]->choose_data.begin(); it!=robot_list[i]->choose_data.end(); it++)
        {
            msg.id.push_back(*it);
        }
        robot_list[i]->data_pub.publish(msg);
    }
}

int main(int argc, char **argv)
{

    ROS_INFO("Process...");
    ros::init(argc, argv, "relay_control");
    ros::NodeHandle nh;

    int total_robots;
    ros::param::get("~total_robots", total_robots);

    vector<vector<double> > snr(total_robots, vector<double>(total_robots));
    vector<vector<double> > packet_loss_rate(total_robots, vector<double>(total_robots));

    for(int i=0; i<total_robots; i++)
    {
        Robot *p = new Robot(i);
        robot_list.push_back(p);
        receive.push_back(false);
    }

    while(ros::ok())
    {
        ros::spinOnce();

        bool all_receive = true;
        for(int i=0; i<total_robots; i++)
        {
            all_receive = all_receive && receive[i];
        }

        if(all_receive)
        {
            getSNR(total_robots, snr);
            chooseData(total_robots, snr);
//            chooseData2(total_robots, packet_loss_rate, snr);
            pub_data(total_robots);

            for(int i=0; i<total_robots; i++)
            {
                receive[i] = false;
            }
        }
    }
}
