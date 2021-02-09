
//Includes
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include <sensor_msgs/Range.h>
#include "std_msgs/Bool.h"
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



//namespace, what needs to be done here?
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
std::vector<float> scan_ranges;

//callback for enable control (error std_msgs needs proper init?)
void enableControlCallback(const std_msgs::bool msg){
    enable_control = msg.data;
// how do we enable? what next?
}



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
// Main begin
int main(int argc, char **argv)
{
    ros::init(argc, argv, "lidarlistener");
    ros::NodeHandle n;

//ros subscribers to laserscan 
    ros::Subscriber sub_laserscan = n.subscribe<sensor_msgs::LaserScan>("/scan", 1000, scanCallback);

//ros subscriber to enable control (error because enablecontrolcallback is failing earlier or std_msgs)
    ros::Subscriber sub_enable_control = n.subscribe<std_msgs::bool>("/enable_control", 1,enableControlCallback);
//Initially have control turned off until we "$rosrun enable_control" right? 
    enable_control = false;
     
//Add while loop to start converting/calculating (commented out for now while we troubleshoot above) (while ros.ok does what exactly?)

  

 while(ros::ok()){
 /*
       // Process the laser data       

void NearnessController::horizLaserscanCb(const sensor_msgs::LaserScanPtr h_laserscan_msg){

    // Convert incoming scan to cv matrix and reformat
    convertHLaserscan2CVMat(h_laserscan_msg);

    // Compute the Fourier harmonics of the signal
    computeHorizFourierCoeffs();

    // Feed back fourier coefficients for forward speed regulation
    computeForwardSpeedCommand();

    computeWFYawRateCommand();

       // Determine motion state

       // Publish control commands to arduino through rosserial

       if(enable_control){
           // Publish the real control commands with rosserial

       } else {
           // Publish zeros with rosserial
       }
 */          
 
        ros::spinOnce();
    }




    return 0;

}
