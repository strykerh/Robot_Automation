
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

/*
void NearnessController::init() {

// Set up dynamic reconfigure
    reconfigure_server_.reset(new ReconfigureServer(config_mutex_, pnh_));
    ReconfigureServer::CallbackType f = boost::bind(&NearnessController::configCb, this, _1, _2);
    reconfigure_server_->setCallback(f);

// Set up subscribers and callbacks ( we covered the first two below i think?)
    sub_horiz_laserscan_ = nh_.subscribe("horiz_scan", 1, &NearnessController::horizLaserscanCb, this);
    subt_enable_control_ = nh_.subscribe("enable_control", 1, &NearnessController::enableControlCb, this);
    sub_tower_safety_ = nh_.subscribe("tower_safety", 1, &NearnessController::towerSafetyCb, this);
    sub_beacon_stop_ = nh_.subscribe("beacon_stop", 1, &NearnessController::beaconStopCb, this);

// Set up publishers
    pub_h_scan_reformat_ = nh_.advertise<std_msgs::Float32MultiArray>("horiz_depth_reformat", 10);
    pub_h_scan_nearness_ = nh_.advertise<std_msgs::Float32MultiArray>("horiz_nearness", 10);
    pub_h_sf_nearness_ = nh_.advertise<std_msgs::Float32MultiArray>("horiz_sf_nearness", 10);
    pub_h_recon_wf_nearness_ = nh_.advertise<std_msgs::Float32MultiArray>("horiz_recon_wf_nearness", 10);
    pub_h_fourier_coefficients_ = nh_.advertise<nearness_control_msgs::FourierCoefsMsg>("horiz_fourier_coefficients", 10);
    pub_control_commands_stamped_ = nh_.advertise<geometry_msgs::TwistStamped>("control_commands_stamped", 10);
    pub_control_commands_ = nh_.advertise<geometry_msgs::Twist>("control_commands", 10);

    // Initialize global variables

    // Wide Field Forward Speed and Yaw Rate Control Gains

    // Create safety boundary(if we use it to stop?)

    // Initialize tower safety counters

} // End of init***********************************************

*/


//define ( can/should we move into init?)
#define RAD2DEG(x) ((x)*180./M_PI)
//add controller gains
bool enable_control;
std::vector<float> scan_ranges;

//callback for enable control (error std_msgs needs proper init?, do we need to do an init to set up like in the beginning?)
void enableControlCallback(const std_msgs::bool msg){
    enable_control = msg.data;
// how do we enable in the terminal? what next?
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


// Main begin************************************************
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
     
// While loop to start converting/calculating (commented out for now while we troubleshoot above) (while ros::ok does what exactly?)******************************************************

  

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

// If statement for enable control ******************************
       if(enable_control){
           // Publish the real control commands with rosserial

// Else for zeros to control command ****************************
       } else {
           // Publish zeros with rosserial
       }
 */          
 
        ros::spinOnce();
    }




    return 0;

}
