#include "ros/ros.h"
#include "std_msgs/Empty.h"
#include "geometry_msgs/Twist.h"
#include "ardrone_autonomy/Navdata.h"
#include "sensor_msgs/Imu.h"
#include <sstream>
#include <iostream>
#include <math.h>
#include <string>
#include <eigen3/Eigen/Geometry>

ardrone_autonomy::Navdata msg_in;
geometry_msgs::Twist msgpos_in;
sensor_msgs::Imu msgyaw_in;
Eigen::Quaterniond quat_orientation;

#define PI 3.14149265

double yaw,pitch,roll;


void read_navdata(const ardrone_autonomy::Navdata::ConstPtr& msg)
{ 
  msg_in.altd = msg->altd; 
}
void read_pos(const geometry_msgs::Twist::ConstPtr& msgpos)//subscriber for position
{
  msgpos_in.linear.x = msgpos->linear.x;
  msgpos_in.linear.y = msgpos->linear.y;
  msgpos_in.linear.z = msgpos->linear.z;
  msgpos_in.angular.z = msgpos->angular.z;
}

struct Quaternionm
{
    double w, x, y, z;
};
void GetEulerAngles(Quaternionm q, double& yaw, double& pitch, double& roll)
{
    const double w2 = q.w*q.w;
    const double x2 = q.x*q.x;
    const double y2 = q.y*q.y;
    const double z2 = q.z*q.z;
    const double unitLength = w2 + x2 + y2 + z2;    // Normalised == 1, otherwise correction divisor.
    const double abcd = q.w*q.x + q.y*q.z;
    const double eps = 1e-7;    // TODO: pick from your math lib instead of hardcoding.
    const double pi = 3.14159265358979323846;   // TODO: pick from your math lib instead of hardcoding.
    if (abcd > (0.5-eps)*unitLength)
    {
        yaw = 2 * atan2(q.y, q.w);
        pitch = pi;
        roll = 0;
    }
    else if (abcd < (-0.5+eps)*unitLength)
    {
        yaw = -2 * ::atan2(q.y, q.w);
        pitch = -pi;
        roll = 0;
    }
    else
    {
        const double adbc = q.w*q.z - q.x*q.y;
        const double acbd = q.w*q.y - q.x*q.z;
        yaw = ::atan2(2*adbc, 1 - 2*(z2+x2));
        pitch = ::asin(2*abcd/unitLength);
        roll = ::atan2(2*acbd, 1 - 2*(y2+x2));
    }
}


void read_yaw(const sensor_msgs::Imu::ConstPtr& msgyaw)
{
  msgyaw_in.orientation.x =  msgyaw->orientation.x;
  msgyaw_in.orientation.y =  msgyaw->orientation.y;
  msgyaw_in.orientation.z =  msgyaw->orientation.z;
  msgyaw_in.orientation.w =  msgyaw->orientation.w;
  Quaternionm myq;
  myq.x = msgyaw->orientation.x;
  myq.y = msgyaw->orientation.y;
  myq.z = msgyaw->orientation.z;
  myq.w = msgyaw->orientation.w;  
  GetEulerAngles(myq, yaw, pitch, roll);
 
  //ROS_INFO("Angles %lf %lf %lf",yaw,pitch,roll);
}
int main(int argc, char **argv)
{
  ros::init(argc, argv, "altipid");
  ros::NodeHandle n;
  
  std_msgs::Empty my_msg;  
  geometry_msgs::Twist m;
  //ros::Publisher takeoff = n.advertise<std_msgs::Empty>("/ardrone/takeoff", 10,true);
  //takeoff.publish(my_msg);
  
  ros::Publisher twist;
  twist = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

  ros::Subscriber read = n.subscribe("/ardrone/navdata", 1000, read_navdata);
  ros::Subscriber readyaw = n.subscribe("/ardrone/imu", 1000, read_yaw);
   ros::Subscriber readpos = n.subscribe("destination/pose",1000, read_pos);  

       ros::Rate loop_rate(10);
     
       m.linear.z = 0.0;
       m.linear.y = 0.0;
       m.angular.x = 0.0;
       m.angular.y = 0.0;
       m.angular.z = 0.0;
       m.linear.x = 0.0;  
        
       double z1,y1;
       double z = 1000.0;
       double  y=45.0; 
       z1 = msg_in.altd;
       
       //don't use this conversion   use library for conversion
       y1 = yaw*180/PI;
       double errorx = 0.0;
       double erroryy = 0.0;
       double error,pre_error,integral,derivative,errory,pre_errory,integraly,derivativey;
       integral =0;
       integraly = 0;
       pre_error = 0;
       pre_errory = 0;
       double pre_errorx,integralx,derivativex,pre_erroryy,integralyy,derivativeyy;
       integralx = 0.0;
       integralyy = 0.0;
       pre_errorx = 0.0;
       pre_erroryy = 0.0;
       
       double kpx,kix,kdx,kpyy,kiyy,kdyy;
       double kp,ki,kd,kpy,kiy,kdy;
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
          kiy = 0.0;
          kpx = 0.011;
          kix = 0.00000081;
          kdx = 0.2;
          kpyy = 0.011;
          kiyy  = 0.00000081;
          kdyy = 0.2;
        }

       std::cout<<"Kp: "<<kp<<" Kd: "<<kd<<" Ki: "<<ki<<std::endl;
       std::cout<<"Kpy: "<<kpy<<" Kdy: "<<kdy<<" Kiy: "<<kiy<<std::endl;
    
        int region = 1;
	      int regiony=1;
        int regionx = 1;
        int regionyy = 1;
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
        y = msgpos_in.angular.z;
		     errory = (y - y1);
       integraly = integraly + errory;
       derivativey = errory-pre_errory;

        if(errory>=0.0)
		      {
			if(errory>=180.0)
			  errory = 360 - errory;
		      }
		    else if(errory<0.0)
		      {
		      	if(errory<=-180)
			errory = 360 + errory;
		      }
          if(abs(errorx) >500)
            regionx = 1;
           else if ((abs(errorx) < 500) && (abs(error) > 100))
            regionx = 5;
           else
            regionx = 10;
       if(abs(erroryy) >500)
            region = 1;
           else if ((abs(erroryy) < 500) && (abs(error) > 100))
            region = 5;
           else
            region = 10;

          z = msgpos_in.linear.z;
          error = (z - z1)/70.0;
       integral = integral + error;
       derivative = error-pre_error;
       //x and yy
       errorx =(msgpos_in.linear.x)/70.0;
       integralx = integralx + errorx;
       derivativex = errorx - pre_errorx;

       erroryy =(msgpos_in.linear.y)/70.0;
       integralyy = integralyy + erroryy;
       derivativeyy = erroryy - pre_erroryy;

     if(abs(error)>0.01)
	 {
       //altitude + yaw
       
         m.linear.z = kp*error +  kd*derivative/region + ki*integral*region;
	 }
       else 
	 m.linear.z = 0;
       if(abs(errory)>0.00001)
	 {
	  m.angular.z = kpy*errory +  kdy*derivativey/regiony + kiy*integraly*regiony;
	 }
       else 
	 m.angular.z = 0;
   if(abs(errorx)>0.01)
   {
    //x and yy
    m.linear.x = kpx*errorx + kdx*derivativex/regionx + kix*integralx*regionx;
   }
       else
   m.linear.x = 0;
       if(abs(erroryy)>0.01)
   {
    m.linear.y = kpyy*erroryy + kdyy*derivativeyy/regionyy + kiyy*integralyy*regionyy;
   }
       else
   {
     m.linear.y = 0;
   }
   
       twist.publish(m);
       ROS_INFO("%lf %lf %lf %lf %lf %lf",z1,z,y1,y,m.linear.x,m.linear.y); 
	
   pre_error = error;
	  z1 = msg_in.altd;
          pre_errory = errory;
  	  //don't use this conversion   use library
	  y1 = yaw*180/PI; 
    //x and yy
     pre_errorx = errorx;
     pre_erroryy = erroryy;
 ros::spinOnce(); 
       loop_rate.sleep();
     
     }



}  

