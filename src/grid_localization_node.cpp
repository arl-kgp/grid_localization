#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include "std_msgs/String.h"
#include "std_msgs/Empty.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Point32.h"
#include "geometry_msgs/Vector3Stamped.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include "grid_localization/Navdata.h"
#include <math.h>

using namespace cv;
using namespace std;


// Global frame information
int fx,fy = 0;


//Color based filtering parameters
int h_th = 60;
int s_th = 60;
int v_th = 30;
int color_h = 29; 
int color_s = 71;

//int color_h = 142; 
//int color_s = 100;

int nearest_threshold = 300;

Point image_center(0,0);

//Hough parameters
int hough_rho = 1;
int hough_theta = 284;
int hough_threshold = 86;
vector<Vec2f> houghLines,lines;
int group_rho = 200;
int group_degrees = 45;
int group_distance = 94;

//Canny parameters
int lowThreshold=100,upperThreshold=100,max_lowThreshold=255,max_upperThreshold=255;

//Autopilot parametrs
float vz_max = 1.05;
float vx_max = 1.05;
float vy_max = 1.05;

float pxv = 0;
float pyv = 0;
float pzv = 0;

float pxa = 0;
float pya = 0;
float pza = 0;


int holding_altitude = 1080; // in cm
bool altitude_hold = false;
int alt_th = 5;
bool alt_send = false;

bool first_mag = true;

int ex_pid = 0;
int ey_pid = 0;
int z_pid = holding_altitude;
float m_pid = 0;

bool node_hold = false;
int point_th = 40;
Point target_point;
bool point_send = false;

bool grid_follow = false;
Point grid_point(0,0);
int intersection_line[2];
bool inner = false;
int outer_radi = 100;
bool on_node = false;


// Function to find intersection between two lines
void findInterscetion(float rho1,float theta1,float rho2,float theta2,int rows,int cols,Point *pt)
{
  double a1=cos(theta1);
  double a2=cos(theta2);
  double b1=sin(theta1);
  double b2=sin(theta2);
  pt->x=(rho1*b2-rho2*b1)/(b2*a1-b1*a2);
  pt->y=(rho2*a1-rho1*a2)/(b2*a1-a2*b1);
}

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_; 
  ros::Subscriber navdata_sub;  
  ros::Subscriber command_sub;  
  ros::Publisher takeoff_pub;
  ros::Publisher land_pub;
  ros::Publisher reset_pub;
  ros::Publisher controls_pub;
  ros::Publisher pid_pub;
  ros::Subscriber autopilot_cmd_vel_sub;
  ros::Subscriber coordinate_sub;
  ros::Subscriber magnetic_sub;
  
public:
  ImageConverter()
  : it_(nh_)
  {
    image_sub_ = it_.subscribe("/ardrone/bottom/image_raw", 1,&ImageConverter::imageCb, this);
    navdata_sub = nh_.subscribe("/ardrone/navdata", 1000, &ImageConverter::navdataCb,this);
    command_sub = nh_.subscribe("/autopilot/command", 1000, &ImageConverter::commandCb,this);
    autopilot_cmd_vel_sub = nh_.subscribe("/autopilot/cmd_vel", 1000, &ImageConverter::cmdvelCb,this);
    coordinate_sub = nh_.subscribe("/autopilot/coordinate", 1000, &ImageConverter::coordinateCb,this);
    magnetic_sub = nh_.subscribe("/ardrone/mag", 1000, &ImageConverter::magneticCb,this);
    takeoff_pub = nh_.advertise<std_msgs::Empty>("/ardrone/takeoff", 1000);
    reset_pub = nh_.advertise<std_msgs::Empty>("/ardrone/reset", 1000);
    land_pub = nh_.advertise<std_msgs::Empty>("/ardrone/land", 1000);
    controls_pub = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
    pid_pub = nh_.advertise<geometry_msgs::Twist>("destination/pose", 1000);
    cv::namedWindow("hough");   
    cv::namedWindow("image");
    cv::namedWindow("sliders");
  }

  ~ImageConverter()
  {
    cv::destroyWindow("image");
  }

  //Main function to change quad velocity
  void publish_velocities()
  {
  	geometry_msgs::Twist command;
  	command.linear.y = -pxv;
  	command.linear.x = pyv;
  	command.linear.z = pzv;
  	command.angular.x = pxa;
  	command.angular.y = pya;
  	command.angular.z = pza;   

  	controls_pub.publish(command);
  	//ROS_INFO("Publishing Velocity: %f %f %f",pxv,pyv,pzv);
  }

  //Main function to publish targets to pid
  void publish_targets()
  {
    geometry_msgs::Twist command;
    command.linear.y = -ex_pid;
    command.linear.x = ey_pid;
    command.linear.z = z_pid;
    command.angular.z = m_pid;
    command.angular.x = 0;
    command.angular.y = 0;   

    pid_pub.publish(command);
    //ROS_INFO("Publishing Velocity: %f %f %f",pxv,pyv,pzv);
  }

  //Arrange the points based on priority
  void arrange_points(Point *pt1, Point *pt2)
  {
  		int p1 = 0;
  		int p2 = 0;

  		if(pt1->x == 0) p1 = 1;
  		if(pt1->x == fx) p1 = 3;
  		if(pt2->x == 0) p2 = 1;
  		if(pt2->x == fx) p2 = 3;

  		if(pt1->y == 0) p1 = 2;
  		if(pt1->y == fy) p1 = 4;
  		if(pt2->y == 0) p2 = 2;
  		if(pt2->y == fy) p2 = 4;

  		if(p1>p2)
  		{
  			Point x = *pt2;
  			*pt2 = *pt1;
  			*pt1 = x;
  		}
  }

  //Function to find line points
  void line_points(Vec2f line, Point *pt1, Point *pt2)
  {
  		float rho = line[0], theta = line[1];
        double a = cos(theta), b = sin(theta);
        double x0 = a*rho, y0 = b*rho;
        pt1->x = cvRound(x0 + 1000*(-b));
        pt1->y = cvRound(y0 + 1000*(a));
        pt2->x = cvRound(x0 - 1000*(-b));
        pt2->y = cvRound(y0 - 1000*(a)); 
        float Slope = (float)(pt2->y - pt1->y)/(float)(pt2->x - pt1->x);

        
        if(pt1->x < 0) 
        {        	
        	pt1->x = 0;
        	pt1->y = pt2->y - ( Slope*(float)(pt2->x-pt1->x) );
        }

        if(pt1->x > fx) 
        {        	
        	pt1->x = fx;
        	pt1->y = pt2->y - ( Slope*(float)(pt2->x-pt1->x) );
        }

        if(pt2->x < 0) 
        {        	
        	pt2->x = 0;
        	pt2->y = pt1->y + ( Slope*(float)(pt2->x-pt1->x) );
        }

        if(pt2->x > fx) 
        {        	
        	pt2->x = fx;
        	pt2->y = pt1->y + ( Slope*(float)(pt2->x-pt1->x) );
        }

        if(pt2->y > fy) 
        {        	
        	pt2->y = fy;
        	pt2->x = pt1->x + ( (float)(pt2->y-pt1->y)/Slope );
        }

        if(pt2->y < 0) 
        {        	
        	pt2->y = 0;
        	pt2->x = pt1->x + ( (float)(pt2->y-pt1->y)/Slope );
        }

        if(pt1->y < 0) 
        {        	
        	pt1->y = 0;
        	pt1->x = pt2->x - ( (float)(pt2->y-pt1->y)/Slope );
        }

        if(pt1->y > fy) 
        {        	
        	pt1->y = fy;
        	pt1->x = pt2->x - ( (float)(pt2->y-pt1->y)/Slope );
        }

        arrange_points(pt1,pt2);
               
  }

  // Function to reset quad
  void reset_quad()
  {
  		std_msgs::Empty empty;
    	ROS_INFO("Reset");
    	reset_pub.publish(empty);
    	pxv = 0;
		pyv = 0;
		pzv = 0;

		pxa = 0;
		pya = 0;
		pza = 0;

		//publish_velocities();

    ex_pid = 0;
    ey_pid = 0;
    //publish_targets();

		altitude_hold = false;
		node_hold = false;
		point_send = false;
		alt_send = false;
		target_point = Point(0,0);		
		grid_point = Point(0,0);
  }

  //Function to turn off the autopilot
  void autopilot_off()
  {	
	ROS_INFO("autopilot_off");
	altitude_hold = false;
	node_hold = false;
	point_send = false;
	alt_send = false;
	//target_point = Point(0,0);
	grid_point = Point(0,0);
	pxv = 0;
	pyv = 0;
	pzv = 0;

	pxa = 0;
	pya = 0;
	pza = 0;

	//publish_velocities();
  ex_pid = 0;
  ey_pid = 0;
  //publish_targets();
  }

  // Fing slope angle
  float slope_angle( Point *pt1, Point *pt2)
  {	
	float theta;
  		if((float)(pt2->x - pt1->x) != 0)
  		{
  			float slope = (float)(pt2->y - pt1->y) / (float)(pt2->x - pt1->x);
  			theta = atan(slope) * 180 / CV_PI;
  		}

  		if(pt2->x > pt1->x && pt2->y < pt1->y) // 1st Quadrant
  		{
  			//ROS_INFO("1st Q");
  			theta = -theta;
  		}

  		else if(pt2->x < pt1->x && pt2->y < pt1->y) // 2nd Quadrant
  		{
  			//ROS_INFO("2nd Q");
  			theta = 180-theta;
  		}

  		else if(pt2->x < pt1->x && pt2->y > pt1->y) // 3rd Quadrant
  		{
  			//ROS_INFO("3rd Q");
  			theta = 180-theta;
  		}

  		else if(pt2->x > pt1->x && pt2->y > pt1->y) // 4th Quadrant
  		{
  			//ROS_INFO("4th Q");
  			theta = 360-theta;
  		}

  		else if((float)(pt2->x - pt1->x) == 0 && pt2->y < pt1->y) 
  		{
  			theta = 90;
  		}

  		else if((float)(pt2->x - pt1->x) == 0 && pt2->y > pt1->y) 
  		{
  			theta = 270;
  		}

  		return theta;
  }

  // Function to track a point
  /*bool point_tracker()
  {

  	if(target_point.x < image_center.x - point_th)
  	{
  		pyv = vy_max;
  		point_send = true;
  	}

  	else if(target_point.x > image_center.x + point_th)
  	{
  		pyv = -vy_max;
  		point_send = true;
  	}

  	if(target_point.y < image_center.y - point_th)
  	{
  		pxv = vx_max;
  		point_send = true;
  	}

  	else if(target_point.y > image_center.y + point_th)
  	{
  		pxv = -vx_max;
  		point_send = true;
  	}

  	if(abs(image_center.x - target_point.x) + abs(image_center.y - target_point.y) < 2*point_th && point_send == true)
  	{
  		point_send = false;
  		pxv = pyv = 0;
  		publish_velocities();

  		if(inner == false)
  		{
  			inner = true;
  			return true;
  		}

  		else return false;
  	}

  	if(abs(image_center.x - target_point.x) + abs(image_center.y - target_point.y) > 2*outer_radi && inner == true)
  	{
  		inner = false;
  	}

  	if(point_send == true)
  	{
  		publish_velocities();
  		return false;
  	}
  }*/

  	bool point_tracker()
  	{
  		/*float theta = slope_angle(&image_center,&target_point);
  		//ROS_INFO("%f",theta);

  		int error = norm(image_center - target_point);
  		//ROS_INFO("error: %d",error);

  		
  			pyv = sin((float)(theta*CV_PI)/180 )*vx_max;
  			pxv = cos((float)(theta*CV_PI)/180 )*vx_max;        
  			publish_velocities();*/

        ex_pid = target_point.x - image_center.x;
        ey_pid = image_center.y - target_point.y;
        //publish_targets();
  		
  		
  		
  		return false;  		
  	}

  	//Stop

  	void stop()
  	{
  		pxv = pyv = 0;
  		//publish_velocities();
      ex_pid = 0;
      ey_pid = 0;
      //publish_targets();

  		//publish_velocities();publish_velocities();publish_velocities();publish_velocities();publish_velocities();publish_velocities();publish_velocities();
  		//ROS_INFO("Stop");
  	}

  //Manual Velocity Override
  void cmdvelCb(const geometry_msgs::Twist::ConstPtr& msg)
  {  
  	autopilot_off();	
  	pxv = msg->linear.x;
  	pyv = msg->linear.y;
  	pzv = msg->linear.z;

  	pxa = msg->angular.x;
  	pya = msg->angular.y;
  	pza = msg->angular.z;  	

  	publish_velocities();
  }

  // Callback function for navdata
  void navdataCb(const grid_localization::Navdata::ConstPtr& msg)
  {  	
  	if(altitude_hold == true)
  	{
  		int present_altitude = msg->altd;
  		if(present_altitude < holding_altitude - alt_th)
  		{
  			pzv = vz_max;
  			alt_send = true;
  			publish_velocities();
  		}
  		else if(present_altitude > holding_altitude + alt_th)
  		{
  			pzv = -vz_max;
  			alt_send = true;
  			publish_velocities();
  		}
  		else
  		{
  			pzv = 0;
  			if(alt_send == true)
  			{
  				publish_velocities();
  				alt_send = false;
  			}
  			
  		}
  	}  	

  }

  // Callback function for coordinate
  void coordinateCb(const geometry_msgs::Point32::ConstPtr& msg)
  {
  	grid_point.x = msg->x;
  	grid_point.y = msg->y;
  	ROS_INFO("Received points: %f,%f",msg->x,msg->y);
  }

  // Callback function for magnetic
  void magneticCb(const geometry_msgs::Vector3Stamped::ConstPtr& msg)
  {
    if(first_mag == true && msg->vector.y != 0)
    {
      m_pid = msg->vector.y; 
      ROS_INFO("Will hold the magnetic: %f",m_pid);
      first_mag = false;
    }    
  }

  void commandCb(const std_msgs::String::ConstPtr& msg)
  {
    string received_command = msg->data;
    ROS_INFO("Received: %s",received_command.c_str());
    // Processing of the received commands

    //Takeoff
    if(received_command == "to")
    {
    	std_msgs::Empty empty;
    	ROS_INFO("Taking off");
    	takeoff_pub.publish(empty);
    }

    //Landing
    else if(received_command == "ld")
    {
    	std_msgs::Empty empty;
    	ROS_INFO("Landing");
    	land_pub.publish(empty);
    }

    //Reset
    else if(received_command == "reset")
    {
    	reset_quad();    	
    }

    //Holding Altitude
    else if(received_command == "holdalt")
    {
    	color_h = 142; 
        color_s = 100;
    	ROS_INFO("Will hold altitude of %d cm",holding_altitude);
    	altitude_hold = true;
    }

    //Stop Holding Altitude
    else if(received_command == "stopholdalt")
    {
    	ROS_INFO("Will stop alt_hold");
    	altitude_hold = false;
    }

    //Up Altitude
    else if(received_command == "up")
    {
    	holding_altitude += 50;
    	ROS_INFO("Will increase holding altitude by 50 cm. Final: %d",holding_altitude);
    }

    //Down Altitude
    else if(received_command == "down")
    {
    	holding_altitude -= 50;
    	ROS_INFO("Will reduce holding altitude by 50 cm. Final: %d",holding_altitude);
    }

    //Holding Node
    else if(received_command == "holdnode")
    {
    	ROS_INFO("Will hold nearest node");
    	node_hold = true;
    }

    //Stop Holding Node
    else if(received_command == "stopholdnode")
    {
    	ROS_INFO("Stopping Node hold");
    	node_hold = false;
    }
    
    // Grid Follow
    else if(received_command == "grid" && node_hold == true)
    {
    	node_hold = false;
    	ROS_INFO("Starting Grid follower to reach %d,%d",grid_point.x,grid_point.y);
    	grid_follow = true;    	
    }

    // Stop Grid Follow
    else if(received_command == "stopgrid" && grid_follow == true)
    {
    	ROS_INFO("Stopping Grid Follow");
    	grid_follow = false;    	
    }

    // Autopilot master off
    else if(received_command == "off")
    {
    	ROS_INFO("Autopilot off");
    	autopilot_off();	
    }
  }


  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }


  // Image Processing starts from here
    fx = cv_ptr->image.cols;
    fy = cv_ptr->image.rows;
  image_center.x = fx/2;
  image_center.y = fy/2;

  Mat orig_hsv,canny_output;
  cvtColor(cv_ptr->image, orig_hsv, CV_BGR2HSV);
  Mat choosen(fy, fx, CV_8UC1, Scalar(0));
  Mat features(fy, fx, CV_8UC3, Scalar(0,0,0));

  // Color based filter 
  createTrackbar( "H:", "sliders", &color_h, 255);
  createTrackbar( "S:", "sliders", &color_s, 255);

  for(int i = 0; i < choosen.rows;i++)
      {
        for(int j = 0; j < choosen.cols;j++)
        {
          if( abs(orig_hsv.at<Vec3b>(i,j)[0] - color_h) < h_th && abs(orig_hsv.at<Vec3b>(i,j)[1] - color_s) < s_th)
          {
            choosen.at<uchar>(i,j) = 255;
          }
        }
      }
      imshow("hsv",choosen);
      // Color based filter end

      // Canny 
      blur( choosen, choosen, Size(3,3) ); 
      createTrackbar( "Canny_lower:", "sliders", &lowThreshold, max_lowThreshold);
      createTrackbar( "Canny_upper:", "sliders", &upperThreshold, max_upperThreshold);     
      Canny(choosen,canny_output,lowThreshold,upperThreshold);
      // Canny end

      // Hough lines and filtering
      createTrackbar( "Rho:", "sliders", &hough_rho, 100);
      createTrackbar( "Theta:", "sliders", &hough_theta, 1000);   
      createTrackbar( "Threshold:", "sliders", &hough_threshold, 200);
      createTrackbar( "Group_rho:", "sliders", &group_rho, 700);   
      createTrackbar( "Group_degrees:", "sliders", &group_degrees, 360);
      createTrackbar( "Group_distance:", "sliders", &group_distance, 360);
      double hough_double_theta =  (double)hough_theta/10000; 
      double hough_double_rho = hough_rho;    
      HoughLines(canny_output, houghLines, hough_double_rho, hough_double_theta, hough_threshold, 0, 0 );

      int loop_no = houghLines.size();
      lines.clear();
      for( size_t i = 0; i < loop_no; i++ )
      {

         /*float rho1 = houghLines[i][0], theta1 = houghLines[i][1];
         float meanRho = rho1;
         float meanTheta = theta1;
         int counter = 1;
         int flag_write = 0; 

        for (int j = 0; j < lines.size();j++)
         {
           float rho2 = lines[j][0], theta2 = lines[j][1];
           if( abs(rho1-rho2) < group_rho && (abs(theta1 - theta2)*180)/CV_PI < group_degrees )
           {
            	flag_write = 1;
            	meanRho += rho2;
            	meanTheta += theta2;
            	counter++;
           }
         }           
        
         if(flag_write == 0)
         {
         	lines.push_back(Vec2f(meanRho/counter,meanTheta/counter));          
         }*/  

		Point pt1,pt2;
		line_points(houghLines[i],&pt1,&pt2);
		float rho1 = houghLines[i][0], theta1 = houghLines[i][1];
         float meanRho = rho1;
         float meanTheta = theta1;
		int counter = 1;
		int flag_write = 0; 

        for (int j = 0; j < lines.size();j++)
         {
           	Point pt3,pt4;
			line_points(lines[j],&pt3,&pt4);
			float rho2 = lines[j][0], theta2 = lines[j][1];
           if(  (norm(pt1 - pt3) + norm(pt2 - pt4)) < group_distance)
           {
            	flag_write = 1;
            	meanRho += rho2;
            	meanTheta += theta2;
            	counter++;
           }
         }           
	        
         if(flag_write == 0)
         {
         	lines.push_back(Vec2f(meanRho/counter,meanTheta/counter));          
         }
          
      }
      // Hough lines and filtering end

      // Display all lines
      for(int i = 0; i< houghLines.size();i++)
      {
        Point pt1,pt2;
        line_points(houghLines[i], &pt1, &pt2);
        line( features, pt1, pt2, Scalar(0,255,0), 4, CV_AA);
        //circle(features,pt1,20,Scalar(2,2,255)); 
        //circle(features,pt2,20,Scalar(2,255,2));
      }

      // Display choosen lines
      for(int i = 0; i< lines.size();i++)
      {
        Point pt1,pt2;
        line_points(lines[i], &pt1, &pt2);        
        line( features, pt1, pt2, Scalar(255,0,0), 4, CV_AA);
      }


      //Find intersection of lines
      Point r,l,t,b = Point(0,0);
      circle(features,image_center,40,Scalar(255,255,255)); 
      Point nearest_intersection(0,0);
      double distance = 99999;
      for( size_t i = 0; i < lines.size(); i++ )
        {
          float theta= lines[i][1] ,rho= lines[i][0];
          
          for(size_t j = i+1; j < lines.size(); j++)
          {
            if(fabs(theta-lines[j][1])>1)      // if two lines are not parallel we will find their intersection   
            {
                Point intersectionPoint;
                findInterscetion(rho,theta,lines[j][0],lines[j][1],cv_ptr->image.rows,cv_ptr->image.cols,&intersectionPoint);
                //circle(canny_output,intersectionPoint,30,Scalar(255)); 
                //circle(features,intersectionPoint,30,Scalar(255,255,0));
                double res =  norm(intersectionPoint - image_center);   
                if(res < distance)
                {
                	distance = res;
                	nearest_intersection = intersectionPoint;
                	if( theta == CV_PI/2 || theta == (3*CV_PI)/2 || abs(tan(theta))>1)
                	{
                		intersection_line[0] = j;
                		intersection_line[1] = i;
                	}

                	else
                	{
                		intersection_line[0] = i;
                		intersection_line[1] = j;
                	}
                	
                }
            }
          }
         }

      // Find nearest neighbour points
      double distancet = 99999; double distancel = 99999; double distanceb = 99999; double distancer = 99999;
      for( size_t i = 0; i < 2; i++ )
        {
          float theta= lines[intersection_line[i]][1] ,rho= lines[intersection_line[i]][0];
          
          for(size_t j = i+1; j < lines.size(); j++)
          {
            if(fabs(theta-lines[j][1])>1)      // if two lines are not parallel we will find their intersection   
            {
                Point intersectionPoint;
                findInterscetion(rho,theta,lines[j][0],lines[j][1],cv_ptr->image.rows,cv_ptr->image.cols,&intersectionPoint);
                double res =  norm(intersectionPoint - image_center);   

                if(i == 0)
                {
                	if(res > distance && res < distancet && intersectionPoint.y < image_center.y)
	                {
	                	distancet = res;
	                	t = intersectionPoint;
	                }  

	                if(res > distance && res < distanceb && intersectionPoint.y > image_center.y)
	                {
	                	distanceb = res;
	                	b = intersectionPoint;
	                }
                } 

                else
                {
                	if(res > distance && res < distancel && intersectionPoint.x < image_center.x)
	                {
	                	distancel = res;
	                	l = intersectionPoint;
	                }  
	                if(res > distance && res < distancer && intersectionPoint.x > image_center.x)
	                {
	                	distancer = res;
	                	r = intersectionPoint;
	                }  
                } 
            }
          }
         }


         circle(features,nearest_intersection,30,Scalar(0,255,255)); 
         circle(features,t,30,Scalar(255,0,0));
         circle(features,b,30,Scalar(255,0,0));
         circle(features,l,30,Scalar(255,255,255));
         circle(features,r,30,Scalar(255,255,255));
       //Finding intersection ends

       // Node Hold
       if(node_hold == true && nearest_intersection != Point(0,0))
       {
       		target_point = nearest_intersection;
       		int error = norm(image_center - target_point);

       		if(error > outer_radi)
       		{
       			//ROS_INFO("I am tracking");
				    point_tracker();
       		}

       		if(error > outer_radi ) {point_send = false; //ROS_INFO("I am falseing");
       		}

       		if (point_send == false && error < point_th)
       		{
       			ROS_INFO("I am stopping node_hold");
       			stop();
       			
       			point_send = true;
       		}

       		//ROS_INFO("%d",point_send);


       }

       // Grid Follow
       if(grid_follow == true)
       {	
       		Point pt1,pt2,pt3,pt4;    
       		line_points(lines[intersection_line[0]], &pt1, &pt2);
       		line_points(lines[intersection_line[1]], &pt3, &pt4);

	        line( features, pt1, pt2, Scalar(0,0,255), 4, CV_AA);
	        line( features, pt3, pt4, Scalar(0,255,255), 4, CV_AA);
	        
       		if(grid_point.x != 0)
       		{
       			if(grid_point.x > 0) // Move to the right
       			{
       				/*if(r != Point(0,0))
       				{
       					target_point = r;
       					//ROS_INFO("Targeting the node on the right");
       				}*/

       				if(norm(image_center - nearest_intersection) > nearest_threshold)
       				{
       					on_node = false;
       				}

       				if(  nearest_intersection.x > image_center.x && on_node == false)
       				{
       					target_point = nearest_intersection;
       				}

       				else
       				{
       					target_point = pt4;
       					//ROS_INFO("Targeting right line point");
       				}
       			}

       			else // Move to the left
       			{
       				/*if(l != Point(0,0))
       				{
       					target_point = l;
       					//ROS_INFO("Targeting the node on the left");
       				}*/

                if(norm(image_center - nearest_intersection) > nearest_threshold)
              {
                on_node = false;
              }

       				if(  nearest_intersection.x < image_center.x && on_node == false)
       				{
       					target_point = nearest_intersection;
       				}



       				else
       				{
       					target_point = pt3;
       					//ROS_INFO("Targeting left line point");
       				}
       			}
       			int error = norm(image_center - target_point);

	       		if(error > outer_radi)
	       		{
	       			//ROS_INFO("I am tracking");
					point_tracker();
	       		}

	       		if(error > outer_radi ) {point_send = false; //ROS_INFO("I am falseing");
	       		}

	       		if (point_send == false && error < point_th)
	       		{
	       			ROS_INFO("I am stopping***************");
	       			grid_point.x = grid_point.x-1;
	       			ROS_INFO("********* GRID POINT :: %d",grid_point.x);
	       			on_node = true;
	       			//stop();
	       			
	       			point_send = true;
	       		}

       			
       		}

       		else if(grid_point.y != 0)
       		{
       			if(grid_point.y < 0) // Move to the bottom
       			{
       				/*if(b != Point(0,0))
       				{
       					target_point = b;
       					//ROS_INFO("Targeting the node below");
       				}*/

       				if(norm(image_center - nearest_intersection) > nearest_threshold)
       				{
       					on_node = false;
       				}

       				if(  nearest_intersection.y > image_center.y && on_node == false)
       				{
       					target_point = nearest_intersection;
       				}

       				else
       				{
       					target_point = pt2;
       					//ROS_INFO("Targeting line point below");
       				}
       			}

       			else // Move to the top
       			{
       				/*if(t != Point(0,0))
       				{
       					target_point = t;
       					//ROS_INFO("Targeting the node above");
       				}*/

       				if(norm(image_center - nearest_intersection) > nearest_threshold)
       				{
       					on_node = false;
       				}

       				if(  nearest_intersection.y < image_center.y && on_node == false)
       				{
       					target_point = nearest_intersection;
       				}

       				else
       				{
       					target_point = pt1;
       					//ROS_INFO("Targeting line point above");
       				}
       			}
       			int error = norm(image_center - target_point);

	       		if(error > outer_radi)
	       		{
	       			//ROS_INFO("I am tracking");
					     point_tracker();
	       		}

	       		if(error > outer_radi ) {point_send = false; //ROS_INFO("I am falseing");
	       	}

	       		if (point_send == false && error < point_th)
	       		{
	       			ROS_INFO("I am stopping point_send");
	       			grid_point.y = grid_point.y-1;
	       			on_node = true;
	       			//stop();
	       			
	       			point_send = true;
	       		}
       		}

       		else 
       		{
       			grid_follow = false;
       			node_hold = true;
       		}
       }

       geometry_msgs::Twist command;
      command.linear.y = -ex_pid;
      command.linear.x = ey_pid;
      command.linear.z = z_pid;
      command.angular.z = m_pid;
      command.angular.x = 0;
      command.angular.y = 0;   

      pid_pub.publish(command);

       circle(features,target_point,30,Scalar(0,255,0));

       //Final output
       cv::imshow("image",cv_ptr->image);
       cv::imshow("hough",canny_output);
       cv::imshow("features",features);
       cv::waitKey(33);   
   
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}
