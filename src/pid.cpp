#include "ros/ros.h"
#include "std_msgs/Empty.h"
#include "geometry_msgs/Twist.h"
#include "ardrone_autonomy/Navdata.h"
#include "sensor_msgs/Imu.h"
#include <sstream>
#include <iostream>
#include <math.h>
#include <string>

ardrone_autonomy::Navdata msg_in;
geometry_msgs::Twist msgpos_in;
sensor_msgs::Imu msgyaw_in;

void read_navdata(const ardrone_autonomy::Navdata::ConstPtr& msg)
{ 
  msg_in.altd = msg->altd;
 
}
void read_yaw(const sensor_msgs::Imu::ConstPtr& msgyaw)
{
  msgyaw_in.orientation.x =  msgyaw->orientation.x;
  msgyaw_in.orientation.y =  msgyaw->orientation.y;
  msgyaw_in.orientation.z =  msgyaw->orientation.z;
  msgyaw_in.orientation.w =  msgyaw->orientation.w;
}
void read_pos(const geometry_msgs::Twist::ConstPtr& msgpos)//subscriber for position
{
  msgpos_in.linear.x = msgpos->linear.x;
  msgpos_in.linear.y = msgpos->linear.y;
  msgpos_in.linear.z = msgpos->linear.z;
  msgpos_in.angular.z = msgpos->angular.z;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pid");
  ros::NodeHandle n;
  
  std_msgs::Empty my_msg;  
  geometry_msgs::Twist m;
  ros::Publisher takeoff = n.advertise<std_msgs::Empty>("/ardrone/takeoff", 10,true);
  takeoff.publish(my_msg);
  
  ros::Publisher twist;
  twist = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

  ros::Subscriber read = n.subscribe("/ardrone/navdata", 1000, read_navdata);
  ros::Subscriber readyaw = n.subscribe("/ardrone/Imu", 1000, read_yaw);
  ros::Subscriber readpos = n.subscribe("destination/pose",1000, read_pos);  
  int c;

      for(c=0;c<10;c++)
      c++;

       ros::Rate loop_rate(10);

       m.linear.z = 0;
       m.linear.y = 0;
       m.angular.x = 0;
       m.angular.y = 0;
       m.angular.z = 0;
       m.linear.x = 0;  
        
       float z1,y1,x1,yy1;
	//altitude + yaw    
       float z = msgpos_in.linear.z;
       float  y=msgpos_in.angular.z; 
       z1 = msg_in.altd;
       //don't use this conversion   use library for conversion
       y1 = asin(2*msgyaw_in.orientation.x*msgyaw_in.orientation.y + 2*msgyaw_in.orientation.z*msgyaw_in.orientation.w); //don't use this conversion  use library for conversion
       // x and yy
       x1 = msgpos_in.linear.x;
       yy1 = msgpos_in.linear.y;
       //altitude + yaw
       float error,pre_error,integral,derivative,errory,pre_errory,integraly,derivativey;
       integral =0;
       integraly = 0;
       pre_errory = 0;
       //x and yy
       float errorx,pre_errorx,integralx,derivativex,erroryy,pre_erroryy,integralyy,derivativeyy;
       integralx = 0;
       integralyy = 0;
       pre_errorx = 0;
       pre_erroryy = 0;
       
       float kp,ki,kd,kpy,kiy,kdy,kpx,kix,kdx,kpyy,kiyy,kdyy;
       
       if(argc > 1)
       {
          kp = std::atof(argv[1]);
          kd = std::atof(argv[2]);
          ki = std::atof(argv[3]);
          kpy = std::atof(argv[4]);
          kdy = std::atof(argv[5]);
          kiy = std::atof(argv[6]);
          kpx = std::atof(argv[7]);
          kdx = std::atof(argv[8]);
          kix = std::atof(argv[9]);
          kpyy = std::atof(argv[10]);
	  kdyy = std::atof(argv[11]);
	  kiyy = std::atof(argv[12]);
        }
       else
       {
          kp = 0.11;
          ki = 0.0000081;
          kd = 2.0; 
	  kpy = 0.03;
	  kdy = 0.1;
	  kiy = 0;
	  kpx = 0.11;
	  kix = 0.0000081;
	  kdx = 2;
	  kpyy = 0.11;
	  kiyy  = 0.0000081;
	  kdyy = 2;
        }

       std::cout<<"Kp: "<<kp<<" Kd: "<<kd<<" Ki: "<<ki<<std::endl;
       std::cout<<"Kpy: "<<kpy<<" Kdy: "<<kdy<<" Kiy: "<<kiy<<std::endl;
    
        int region = 1;
	int regiony=1;

       while (ros::ok())
	 {    //altitude + yaw
       
	  if(abs(error) >500)
          region = 1;
        else if ((abs(error) < 500) && (abs(error) > 100))
          region = 5;
        else
	region = 10;
	 	    if(abs(errory) >500)
          regiony = 1;
        else if ((abs(errory) < 500) && (abs(errory) > 100))
          regiony = 5;
        else
	regiony = 10;
		    if(errory>=0)
		      {
			if(errory>=180)
			  errory = 360 - errory;
		      }
		    else if(errory<0)
		      {
			errory = (360 + errory)*(-1);
		      }
		    // x and yy
		    /*  if(abs(error) >500)
            region = 1;
           else if ((abs(error) < 500) && (abs(error) > 100))
            region = 5;
           else
	   region = 10;*/
	   //altitude + yaw
       errory = (y - y1)/70;
       integraly = integraly + errory;
       derivativey = errory-pre_errory;


          error = (z - z1)/70;
       integral = integral + error;
       derivative = error-pre_error;
       //x and yy
       errorx =(msgpos_in.linear.x)/70;
       integralx = integralx + errorx;
       derivativex = errorx - pre_errorx;
       erroryy =(msgpos_in.linear.y)/70;
       integralyy = integralyy + erroryy;
       derivativeyy = erroryy - pre_erroryy;
       if(abs(error)>0.01)
	 {
       //altitude + yaw
       
         m.linear.z = kp*error +  kd*derivative/region + ki*integral*region;
	 }
       else 
	 m.linear.z = 0;
       if(abs(errory)>0.1)
	 {
	  m.angular.z = kpy*errory +  kdy*derivativey/regiony + kiy*integraly*regiony;
	 }
       else 
	 m.angular.z = 0;
       if(abs(errorx)>0.01)
	 {
	  //x and yy
	  m.linear.x = kpx*errorx + kdx*derivativex + kix*integralx;
	 }
       else
	 m.linear.x = 0;
       if(abs(erroryy)>0.01)
	 {
	  m.linear.y = kpyy*erroryy + kdyy*derivativeyy + kiyy*integralyy;
	 }
       else
	 {
	   m.linear.y = 0;
	 }
	     ROS_INFO("%.2f %.2f %.2f %.2f %.2f %.2f %.2f", msg_in.rotZ, errory, integraly, derivativey, m.angular.z,y,y1);    
	
       twist.publish(m);
       //altitude + yaw
          pre_error = error;
	  z1 = msg_in.altd;
          pre_errory = errory;
	  //don't use this conversion   use library
	  y1 = asin(2*msgyaw_in.orientation.x*msgyaw_in.orientation.y + 2*msgyaw_in.orientation.z*msgyaw_in.orientation.w);  //don't use this conversion   use library 
	   //x and yy
	   pre_errorx = errorx;
	   pre_erroryy = erroryy;
      
       ros::spinOnce(); 
       loop_rate.sleep();
     
     }
  
  
    return 0;
  }
