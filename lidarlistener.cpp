
//Includes
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include <sensor_msgs/Range.h>
#include "std_msgs/Bool.h"
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
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


//namespace? what needs to be done here
/*
namespace nearness{
NearnessController::NearnessController(const ros::NodeHandle &node_handle,
                                       const ros::NodeHandle &private_node_handle)
      :nh_(node_handle),
       pnh_(private_node_handle) {
      this->init();
  }
*/

//define
#define RAD2DEG(x) ((x)*180./M_PI)

bool enable_control;

//callback for enable control
void enableControlCallback(const std_msgs::bool msg){
    enable_control = msg.data;
}


std::vector<float> scan_ranges;

//callback for the lidar scan
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
// Main 
int main(int argc, char **argv)
{
    ros::init(argc, argv, "lidarlistener");
    ros::NodeHandle n;

    ros::Subscriber sub_laserscan = n.subscribe<sensor_msgs::LaserScan>("/scan", 1000, scanCallback);
    ros::Subscriber sub_enable_control = n.subscribe<std_msgs::bool>("/enable_control", 1,enableControlCallback);
 
    enable_control = false;
     
//Add while loop to start converting/calculating (commented out for now while we troubleshoot above)

 /*   while(ros.ok()){
 
       // Process the laser data       
       // Determine motion state
       // Publish control commands to arduino through rosserial topics

void NearnessController::horizLaserscanCb(const sensor_msgs::LaserScanPtr h_laserscan_msg){

    // Convert incoming scan to cv matrix and reformat
    convertHLaserscan2CVMat(h_laserscan_msg);

    // Compute the Fourier harmonics of the signal
    computeHorizFourierCoeffs();

    // Feed back fourier coefficients for forward speed regulation
    computeForwardSpeedCommand();

    computeWFYawRateCommand();

       if(enable_control){
           // Publish the real control commands

       } else {
           // Publish zeros
       }
           
 
        ros::spinOnce();
    }
}
*/
    return 0;

}
