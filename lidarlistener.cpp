
//Includes
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include <sensor_msgs/Range.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <math.h>
#include <numeric>
#include <iterator>
#include <vector>
#include <algorithm>
#include <cstdlib>
#include <std_msgs/String.h>
#include <dynamic_reconfigure/server.h>
#include <iostream>
#include <random>
#include <std_srvs/SetBool.h>
using namespace std; //do we need this?
//namespace, what needs to be done here? is below okay?
/*
namespace nearness{
NearnessController::NearnessController(const ros::NodeHandle &node_handle,
                                       const ros::NodeHandle &private_node_handle)
      :nh_(node_handle),
       pnh_(private_node_handle) {
      this->init();
  }
*/

//init nearness controller *************************************


void init();

//what do we need to init up here? seems like it was done in main according to mike?

 // End of init***********************************************
//functions ( do we need to predefine here before main?)
void convertHLaserscan2CVMat(const sensor_msgs::LaserScanPtr scan_ranges);
void computeHorizFourierCoeffs();
void computeForwardSpeedCommand();
void computeWFYawRateCommand();
void computeSFYawRateCommand();



//define global variables**********************
#define RAD2DEG(x) ((x)*180./M_PI)
//add controller gains
bool enable_control;
std::vector<float> scan_ranges;
//**************************************************************

//Callbacks*****************************************************
//callback for enable control (error std_msgs needs proper init?, do we need to do an init to set up like in the beginning?)
void enableControlCallback(const std_msgs::bool msg){
    enable_control = msg.data;
}
// to enable in terminal, $ rostopic pub enable_control std_msgs/bool "true"


//callback for the lidar scan, will clear and refresh scan vector called scan_ranges
void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    int count = scan->scan_time / scan->time_increment;
    ROS_INFO("I heard a laser scan %s[%d]:", scan->header.frame_id.c_str(), count);
    ROS_INFO("angle_range, %f, %f", RAD2DEG(scan->angle_min), RAD2DEG(scan->angle_max));
    scan_ranges.clear();
    for(int i = 0; i < count; i++) {
        scan_ranges.push_back(scan->ranges[i]);
        float degree = RAD2DEG(scan->angle_min + scan->angle_increment * i);
        ROS_INFO(": [%f, %f]", degree, scan->ranges[i]);
    }
}


//***********************************************************

// Main begin************************************************
int main(int argc, char **argv)
{

    ros::init(argc, argv, "lidarlistener");
    ros::NodeHandle n;
//Subscribers Setup**********************************************
//ros subscribers to laserscan 
    ros::Subscriber sub_laserscan = n.subscribe<sensor_msgs::LaserScan>("/scan", 1000, scanCallback);
//ros subscriber to enable control (error because enablecontrolcallback is failing earlier or std_msgs)
    ros::Subscriber sub_enable_control = n.subscribe<std_msgs::bool>("/enable_control", 1,enableControlCallback);
//Initially have control turned off until we "$rosrun enable_control" right? 
    enable_control = false;
//***************************************************************

//publishers set up**********************************************
/* 
 ros::Publisher pub_h_scan_nearness_ = nh_.advertise<std_msgs::Float32MultiArray>("horiz_nearness", 10);
  ros::Publisher pub_h_recon_wf_nearness_ = nh_.advertise<std_msgs::Float32MultiArray>("horiz_recon_wf_nearness", 10);
 ros::Publisher pub_h_fourier_coefficients_ = nh_.advertise<nearness_control_msgs::FourierCoefsMsg>("horiz_fourier_coefficients", 10);
  ros::Publisher pub_control_commands_stamped_ = nh_.advertise<geometry_msgs::TwistStamped>("control_commands_stamped", 10);
 ros::Publisher pub_control_commands_ = nh_.advertise<geometry_msgs::Twist>("control_commands", 10);
*/
//***************************************************************

     
// While loop to start converting/calculating (commented out for now while we troubleshoot above)

 while(ros::ok()){
 
       // Process the laser data       

/*
    // Convert incoming scan to cv matrix and reformat
 void convertHLaserscan2CVMat(const sensor_msgs::LaserScanPtr scan_ranges);
//how to access the scan is that ^?
    // Compute the Fourier harmonics of the signal

    computeHorizFourierCoeffs();

    // Feed back fourier coefficients for forward speed regulation
    computeForwardSpeedCommand();

    computeWFYawRateCommand();

       // Determine motion state ( safety box stuff_
*/
// If statement for enable control ******************************
       if(enable_control){
           // Publish the real control commands with rosserial to arduino

// Else for zeros to control command ****************************
       } else {
           // Publish zeros with rosserial
       }
           

 //check callbacks once
        ros::spinOnce();
    }




    return 0;

} //end  main
