
//#include <gait_processing/gaitOut.h>
#include "ros/ros.h"
#include "ros/message_traits.h"
#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Imu.h"
#include <iostream>
#include <string>
#include <fstream>
#include <istream>
#include <sstream>
#include <cmath>
#include <vector>
#include <stdint.h>
#include <bits/basic_string.h>

//#include "MadgwickAHRS.h"
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/Dense>
#define sampleRate 100.0
using namespace std;
using namespace Eigen;

double global_count = 0.0;
double meanX,meanY,meanZ;
vector<Vector3d> ACC,GYR,vel,vel_drift;
vector<ros::Time> TIME;
vector<Quaterniond> quat,ORI;
vector<int> stationary,motionStart,motionEnd;
vector<int> motionStartTrue,motionEndTrue;
Quaterniond Q(1,0,0,0);
Vector3d P(0,0,0);
ros::Publisher process_pub;

template <class Type> 
Type stringToNum(const string& str)  
{  
    istringstream iss(str);  
    Type num;  
    iss >> num;  
    return num;      
} 

double getData(string line,string aim,int length_a,int length_b);
void AHRSupdate(double gx, double gy, double gz, double ax, double ay, double az, double Kp, double Ki);

void readIMUCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
  
  
    //string data_str =  msg->data.c_str();
    
    nav_msgs::Odometry odom;
    odom.header.frame_id = "odom";
    
  //gait_processing::gaitOut output;
  /* ROS_INFO("received acc: [%lf]", getData(data_str,"accX",7,16));
   ROS_INFO(" [%lf]",getData(data_str,"accY",7,16));
   ROS_INFO(" [%lf]",getData(data_str,"accZ",7,16));
   ROS_INFO("number: [%lf]",global_count/100);*/
  Vector3d acc,gyr;
  Quaterniond ori;
  double time;
  ros::Time timeFine;
  double InitPeriod = 2.0;
  //double InitPeriod = 1.0;//this is just for test!!
    vector<int> stationary_temp;
    
//    acc[0] = getData(data_str,"accX",7,16)/9.81;
//    acc[1] = getData(data_str,"accY",7,16)/9.81;
//    acc[2] = getData(data_str,"accZ",7,16)/9.81;
//    gyr[0] = getData(data_str,"gyrX",7,22)*57.3;
//    gyr[1] = getData(data_str,"gyrY",7,22)*57.3;
//    gyr[2] = getData(data_str,"gyrZ",7,22)*57.3;
   
     acc[0] = msg->linear_acceleration.x/9.81;
   acc[1] = msg->linear_acceleration.y/9.81;
   acc[2] = msg->linear_acceleration.z/9.81;
   gyr[0] = msg->angular_velocity.x*57.3;
   gyr[1] = msg->angular_velocity.y*57.3;
   gyr[2] = msg->angular_velocity.z*57.3;
   ori.w() = msg->orientation.w;
   ori.x() = msg->orientation.x;
   ori.y() = msg->orientation.y;
   ori.z() = msg->orientation.z;
   
   timeFine = msg->header.stamp;
   //TimeStamp timeFine(stringToNum<double>(data_str.substr(0,15))/1000000000);
   TIME.push_back(timeFine);
   ACC.push_back(acc);
   GYR.push_back(gyr);
   ORI.push_back(ori);
   time = global_count/sampleRate;
   //ROS_INFO("time: [%lf]",time);
   if(time<InitPeriod)
   { meanX = meanX + acc[0];
     meanY = meanY + acc[1];
     meanZ = meanZ + acc[2];
     ROS_INFO("Initializing;Please stand still.\n");
     
    }
   else if(time==InitPeriod)//should be ok to neglect the acc and gyr data at this time point
   {
    meanX = meanX/(InitPeriod*sampleRate);
    meanY = meanY/(InitPeriod*sampleRate);
    meanZ = meanZ/(InitPeriod*sampleRate);
    for(int i=0;i<2000;i++)
    {
      AHRSupdate(0,0,0,meanX,meanY,meanZ,1,0);  //initialization
      //cout<<"initializing: "<<Q.w()<<" "<<Q.x()<<" "<<Q.y()<<" "<<Q.z()<<endl;
    }
    ROS_INFO("Initialization complete,now Quaternion: [%lf]",Q.w());
    ROS_INFO(" [%lf]",Q.x());
    ROS_INFO(" [%lf]",Q.y());
    ROS_INFO(" [%lf]",Q.z());
     ACC.clear();
     GYR.clear();
     TIME.clear();
   }
    else 
    {
      double gyrMag;
      gyrMag = sqrt(pow(gyr[0],2)+pow(gyr[1],2)+pow(gyr[2],2));
      if (gyrMag<25)
	stationary.push_back(1);
      else
	stationary.push_back(0);
      //ROS_INFO("stationary: [%d]",stationary[stationary.size()-1]);
      int currentNumber = global_count-InitPeriod*sampleRate-1;
      //ROS_INFO("currentNumber: [%d]",currentNumber);//for debugging
      if ((stationary.size()>1)&&(stationary[currentNumber]-stationary[currentNumber-1] == -1))
      {motionStart.push_back(currentNumber);
      //ROS_INFO("motionStart at: [%d]", motionStart[motionStart.size()-1]);
	
      } 
      else if ((stationary.size()>1)&&(stationary[currentNumber]-stationary[currentNumber-1] == 1))
      {
	motionEnd.push_back(currentNumber-1);
	//ROS_INFO("motionEnd at: [%d]", motionEnd[motionEnd.size()-1]);
	if(motionEnd[motionEnd.size()-1]-motionStart[motionEnd.size()-1]>=30)
	{
	  motionEndTrue.push_back(motionEnd[motionEnd.size()-1]);
	  motionStartTrue.push_back(motionStart[motionEnd.size()-1]);
	 //ROS_INFO("motionStartTrue at: [%d]", motionStartTrue[motionStartTrue.size()-1]);
	  ROS_INFO("motionEndTrue at: [%d]", motionEndTrue[motionEndTrue.size()-1]);
	 //once step updated and confirmed,carry out the zero-velocity update
	 //first correct the stationary status
	 if(motionEndTrue.size() == 1) //this is for the first step
	 {
	   for(int i=0;i<motionStartTrue[0];i++)
	   {
	     stationary[i] = 1;
	     stationary_temp.push_back(1);
	  }
	  for(int i=motionStartTrue[0];i<=motionEndTrue[0];i++)
	  {
	    stationary[i] = 0;
	    stationary_temp.push_back(0);
	  }
	 }
	 else  //this is for the rest of steps
	 {
	   for(int i = motionEndTrue[motionEndTrue.size()-2]+1;i<motionStartTrue[motionStartTrue.size()-1];i++)
	   {
	   stationary[i] = 1;
	   stationary_temp.push_back(1);
	   //ROS_INFO("stationary: [%d]",stationary[i]);
	  }
	    for(int i = motionStartTrue[motionStartTrue.size()-1];i<=motionEndTrue[motionEndTrue.size()-1];i++)
	    {
	      stationary[i] = 0;
	      stationary_temp.push_back(0);
	      //ROS_INFO("stationary: [%d]",stationary[i]);
	    }
	 } 
	 ROS_INFO("gyr size: [%ld]",GYR.size());
	 ROS_INFO("acc size: [%ld]",ACC.size());
	 
	 //Transform<double,3,Isometry> t;
	  Quaterniond v_q,v0XYZ;
	  Vector3d acc;
	 Vector3d V(0,0,0);
	 for(int i=0;i<GYR.size()-1;i++)//calculate orientation
	 {
	   //ROS_INFO("stationary: [%d]",stationary[motionEndTrue[motionEndTrue.size()-2]+1+i]);
	   if(stationary_temp[i])//not stationary[i]!!!
	    AHRSupdate(GYR[i][0]/57.3,GYR[i][1]/57.3,GYR[i][2]/57.3,ACC[i][0],ACC[i][1],ACC[i][2],0.5,0);
	    else
	    AHRSupdate(GYR[i][0]/57.3,GYR[i][1]/57.3,GYR[i][2]/57.3,ACC[i][0],ACC[i][1],ACC[i][2],0,0);
	    quat.push_back(Q);
	   
	    /*t = Transform<double,3,Isometry>::Identity();
	    t.rotate(Q);
	  ACC[i] = t*ACC[i]; */ 
	    
	    v_q.w() = 0;
	    v_q.x() = ACC[i][0];
	    v_q.y() = ACC[i][1];
	    v_q.z() = ACC[i][2];
	    v0XYZ = (Q*v_q)*Q.conjugate();
	   // v0XYZ = (ORI[i]*v_q)*ORI[i].conjugate();
	    ACC[i][0] = v0XYZ.x();
	    ACC[i][1] = v0XYZ.y();
	    ACC[i][2] = v0XYZ.z();
      
	    
	    ACC[i] = ACC[i]*9.81;
	    ACC[i][2] = ACC[i][2] - 9.81;
	    //if(i>0)
	    V = V+ACC[i]/sampleRate;
	   
	    if(stationary_temp[i])  //zero-velocity update
	      {V<<0,0,0;}
//    	      ROS_INFO("vel: [%lf]",V[0]);
// 	      ROS_INFO("vel: [%lf]",V[1]);
//    	     ROS_INFO("vel: [%lf]",V[2]);
	    vel.push_back(V);
	    
	}
	//calculate swing phase drift
	Vector3d drift(0,0,0);
	for(int i=0;i<quat.size();i++)
	{
	  vel_drift.push_back(drift);
	}
	Vector3d driftRate = vel[vel.size()-1]/(1+motionEndTrue[motionEndTrue.size()-1]-motionStartTrue[motionStartTrue.size()-1]);
  /*	ROS_INFO("driftRate: [%lf]",driftRate[0]);
  	ROS_INFO("driftRate: [%lf]",driftRate[1]);
        ROS_INFO("driftRate: [%lf]",driftRate[2]);*/
	for(int j=motionStartTrue[motionStartTrue.size()-1];j<=motionEndTrue[motionEndTrue.size()-1];j++)
	{
	  vel_drift[j]<<driftRate[0]*(j-motionStartTrue[motionStartTrue.size()-1]+1),
			driftRate[1]*(j-motionStartTrue[motionStartTrue.size()-1]+1),
			driftRate[2]*(j-motionStartTrue[motionStartTrue.size()-1]+1);
 
	}
	vector<Vector3d> pos;
	for(int i=0;i<vel.size();i++)//remove swing phase drift
	{
// 	  ROS_INFO("vel_drift: [%lf]",vel_drift[motionEndTrue[motionEndTrue.size()-1]-vel.size()+1+i][0]);
//    	  ROS_INFO("vel_drift: [%lf]",vel_drift[motionEndTrue[motionEndTrue.size()-1]-vel.size()+1+i][1]);
//    	  ROS_INFO("vel_drift: [%lf]",vel_drift[motionEndTrue[motionEndTrue.size()-1]-vel.size()+1+i][2]);
	  vel[i] = vel[i] - vel_drift[motionEndTrue[motionEndTrue.size()-1]-vel.size()+1+i];
   	  
	   P = P+vel[i]/sampleRate;
	   pos.push_back(P);
	}
// 	for(int j=motionStartTrue[motionStartTrue.size()-1];j<=motionEndTrue[motionEndTrue.size()-1];j++)
// 	{
// 	 ROS_INFO("vel: [%lf]",vel[j][0]);
//    	  ROS_INFO("vel: [%lf]",vel[j][1]);
// 	  ROS_INFO("vel: [%lf]",vel[j][2]);
//  //sth wrong here
// 	}
	
    
 	for(int i=0;i<quat.size();i++)
 	{
//  	  output.quatW = quat[i].w();
//  	  output.quatX = quat[i].x();
//  	  output.quatY = quat[i].y();
//  	  output.quatZ = quat[i].z();
//  	  output.posX = pos[i][0];
//  	  output.posY = pos[i][1];
//  	  output.posZ = pos[i][2];
// 	  output.time = TIME[i];
	  
 	 // process_pub.publish(output);
	  
	  odom.header.stamp = TIME[i];
	  odom.pose.pose.position.x = pos[i][0]; 
	  odom.pose.pose.position.y = pos[i][1]; 
	  odom.pose.pose.position.z = pos[i][2]; 
 	  odom.pose.pose.orientation.w = quat[i].w();
 	  odom.pose.pose.orientation.x = quat[i].x();
 	  odom.pose.pose.orientation.y = quat[i].y();
 	  odom.pose.pose.orientation.z = quat[i].z();
// 	  odom.pose.pose.orientation.w = quat[i].conjugate().w();
//  	  odom.pose.pose.orientation.x = quat[i].conjugate().x();
// 	  odom.pose.pose.orientation.y = quat[i].conjugate().y();
// 	  odom.pose.pose.orientation.z = quat[i].conjugate().z();
//	  odom.child_frame_id = "base_link";
	  process_pub.publish(odom);
	  
 	}
	quat.clear();
	ACC.clear();
	ACC.push_back(acc);//or data at this point will be lost(no used yet, decided by the way motionEnds are recongnized)	
	GYR.clear();
	GYR.push_back(gyr);
	ORI.clear();
	ORI.push_back(ori);
	TIME.clear();
	//TIME.push_back(time);  //should be wrong?
	TIME.push_back(timeFine);
	vel.clear();
	//vel_drift.clear();
	}
      }
      
    }  
   
   
   
   global_count++; //packet number
   return;


}

int main(int argc, char **argv)
{
    vector<double> accX,accY,accZ,gyrX,gyrY,gyrZ,time,ret;
   
/**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line. For programmatic
   * remappings you can use a different version of init() which takes remappings
   * directly, but for most command-line programs, passing argc and argv is the easiest
   * way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "gaitprocess");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("/imu/data", 1000, readIMUCallback);
  //ros::Subscriber sub = n.subscribe("/imu_data_str", 1000, readIMUCallback);
  //process_pub = n.advertise<gait_processing::gaitOut>("/gaitOuts",1000);
  process_pub = n.advertise<nav_msgs::Odometry>("/odom",1000);


  ros::spin();


  return 0;
}



double getData(string line,string aim,int length_a,int length_b)
{
  double aimData;
  if(line.size())
   {
	string::size_type idx;
	
	idx = line.find(aim);
    
	string truncate(line.substr(idx+length_a,length_b));//find particular data from imu_data_str
	if(idx == string::npos )
	cout << "not found\n";
	else
	aimData = stringToNum<double>(truncate); 			
   }
  else
  ROS_INFO("error:no data received\n");
      /*for(int i=0;i<aimData.size();i++)
      {
	printf("%0.16lf\n",aimData[i]);
      }
      cout<<"size is "<<aimData.size()<<endl;*/
      return aimData;
      
}

void AHRSupdate(double gx,double gy,double gz,double ax,double ay,double az,double Kp,double Ki)
{
  //double sampleRate = 100;
  
  
  Quaterniond q;
  q = Q;
 // q = Q.conjugate();
  //q<<1,0,0,0;
  Vector3d IntError;
  IntError<<0,0,0;
  Vector3d gyr,acc;
  gyr<<gx,gy,gz;
  acc<<ax,ay,az;
  if (acc.norm()==0)
  {cerr<<"error:acc is 0 "<<endl;
  //  return -1;
  }
  else 
  {acc = acc/acc.norm();}
  Vector3d v;
  v<<2*(q.x()*q.z()-q.w()*q.y()),
     2*(q.w()*q.x()+q.y()*q.z()),
     pow(q.w(),2)-pow(q.x(),2)-pow(q.y(),2)+pow(q.z(),2);
  Vector3d error = v.cross(acc);
  IntError = IntError + error;
  Vector3d Ref;
  Ref = gyr - (Kp*error + Ki*IntError);
  Quaterniond mid(0,Ref[0],Ref[1],Ref[2]);
  Quaterniond pDot;
  //pDot = (q*mid)/2;
  pDot = q*mid;
  pDot.w() = pDot.w()/(2*sampleRate);
  pDot.x() = pDot.x()/(2*sampleRate);
  pDot.y() = pDot.y()/(2*sampleRate);
  pDot.z() = pDot.z()/(2*sampleRate);
  
 // q = q + pDot/sampleRate;
  q.w() = q.w()+pDot.w();
  q.x() = q.x()+pDot.x();
  q.y() = q.y()+pDot.y();
  q.z() = q.z()+pDot.z();
  double norm = q.norm();
  q.w() = q.w()/norm;
  q.x() = q.x()/norm;
  q.y() = q.y()/norm;
  q.z() = q.z()/norm;
 // Q = q.conjugate();
  Q = q;
  //return Q;
}
