
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
#include <cv_bridge/cv_bridge.h>
// #include <std_srvs/SetBool.h>

//init nearness controller *************************************


//void init();

 // End of init***********************************************
//functions ( do we need to predefine here before main?)
// void convertHLaserscan2CVMat(const sensor_msgs::LaserScanPtr scan_ranges);
// void computeHorizFourierCoeffs();
// void computeForwardSpeedCommand();
// void computeWFYawRateCommand();
// void computeSFYawRateCommand();



//define global variables**********************
#define RAD2DEG(x) ((x)*180./M_PI)
// converHtLaserscan2CVMat
int h_num_fourier_terms_;
//cv::Mat h_depth_cvmat_; // how do we get this to work? need cv namespace?
// computeHorizFourierCoeffs
float h_a_[10], h_b_[10];
    //cv::Mat h_nearness_;
// computeWFYawRateCommand
float h_wf_r_cmd_;
// publishControlCommandMsg
//geometry_msgs::TwistStamped control_command_;



float total_h_scan_points_ = 4000; //double check, use this for now
float h_sensor_max_dist_ = 6; //6 meters, not sure if should equal 6 though
//add controller gains
bool enable_control;
std::vector<float> scan_ranges;

//Callbacks*****************************************************
//callback for enable control
void enableControlCallback(const std_msgs::BoolConstPtr& msg){
  enable_control = msg->data;
}
// to enable in terminal, $ rostopic pub enable_control std_msgs/bool "true"


//callback for the lidar scan, will clear and refresh scan vector called scan_ranges
void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    int count = scan->scan_time / scan->time_increment;
    ROS_INFO("I heard a laser scan %s[%d]:", scan->header.frame_id.c_str(), count);
    //ROS_INFO("angle_range, %f, %f", RAD2DEG(scan->angle_min), RAD2DEG(scan->angle_max));
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
    ros::Subscriber sub_laserscan = n.subscribe("/scan", 1000, scanCallback);
//ros subscriber to enable control 
    ros::Subscriber sub_enable_control = n.subscribe("/enable_control", 1, enableControlCallback);
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


    // Convert incoming scan to cv matrix and reformat**************************
 void convertHLaserscan2CVMat(const sensor_msgs::LaserScanPtr scan_ranges);
 std::vector<float> h_depth_vector = scan_ranges; //->ranges;
 std::vector<float> h_depth_vector_noinfs = h_depth_vector;
 
    // handle infs due to sensor max distance (find total scan points)
    for(int i = 0; i<total_h_scan_points_; i++){
        if(isinf(h_depth_vector[i])){
            if(i == 0){
                if(isinf(h_depth_vector[i+1])){
                    h_depth_vector_noinfs[i] = h_sensor_max_dist_;
                } else {
                    h_depth_vector_noinfs[i] = h_depth_vector[i+1];
                }
            } else if(i == total_h_scan_points_){
                if(isinf(h_depth_vector[i-1])){
                    h_depth_vector_noinfs[i] = h_sensor_max_dist_;
                } else {
                    h_depth_vector_noinfs[i] = h_depth_vector[i-1];
                }
            } else {
                if(isinf(h_depth_vector[i-1]) && isinf(h_depth_vector[i+1])){
                    h_depth_vector_noinfs[i] = h_sensor_max_dist_;
                } else {
                    h_depth_vector_noinfs[i] = (h_depth_vector[i-1] + h_depth_vector[i+1])/2.0;
                }
            }
        }
        h_depth_vector = h_depth_vector_noinfs;
    }
 
 // Reformat the depth scan depending on the orientation of the scanner
    // scan_start_loc describes the location of the first scan index

 // Trim the scan down if the entire scan is not being used

// Check to see if anything has entered the safety boundary (lets do this after code is more progressed

// Publish the reformatted scan

// Last, convert to cvmat and saturate (need to initialize)
/*
    h_depth_cvmat_ = cv::Mat(1,num_h_scan_points_, CV_32FC1);
    std::memcpy(h_depth_cvmat_.data, h_depth_vector_trimmed.data(), h_depth_vector_trimmed.size()*sizeof(float));
    h_depth_cvmat_.setTo(h_sensor_min_dist_, h_depth_cvmat_ < h_sensor_min_dist_);
    h_depth_cvmat_.setTo(h_sensor_max_dist_, h_depth_cvmat_ > h_sensor_max_dist_);
*/

// END Convert incoming scan to cv matrix and reformat***************************


// Compute the Fourier harmonics of the signal***********************************
/*
  void computeHorizFourierCoeffs();
    float h_cos_gamma_arr[h_num_fourier_terms_ + 1][num_h_scan_points_];
    float h_sin_gamma_arr[h_num_fourier_terms_ + 1][num_h_scan_points_];

// Compute horizontal nearness
    h_nearness_ = cv::Mat::zeros(cv::Size(1, num_h_scan_points_), CV_32FC1);
    h_nearness_ = 1.0/ h_depth_cvmat_;
    std::vector<float> h_nearness_array(h_nearness_.begin<float>(), h_nearness_.end<float>());
    h_nearness_maxval_ = *std::max_element(h_nearness_array.begin(), h_nearness_array.end())

// Compute the Fourier Coefficients
    cv::Mat h_cos_gamma_mat(h_num_fourier_terms_ + 1, num_h_scan_points_, CV_32FC1, h_cos_gamma_arr);
    cv::Mat h_sin_gamma_mat(h_num_fourier_terms_ + 1, num_h_scan_points_, CV_32FC1, h_sin_gamma_arr);

    for (int i = 0; i < h_num_fourier_terms_ + 1; i++) {
        for (int j = 0; j < num_h_scan_points_; j++) {
            h_cos_gamma_arr[i][j] = cos(i * h_gamma_vector_[j]);
            h_sin_gamma_arr[i][j] = sin(i * h_gamma_vector_[j]);
        }
        h_a_[i] = h_nearness_.dot(h_cos_gamma_mat.row(i)) * h_dg_ / M_PI;
        h_b_[i] = h_nearness_.dot(h_sin_gamma_mat.row(i)) * h_dg_ / M_PI;
    }
        // Publish horizontal WFI Fourier coefficients
        // Convert array to vector
        std::vector<float> h_a_vector(h_a_, h_a_ + sizeof h_a_ / sizeof h_a_[0]);
        std::vector<float> h_b_vector(h_b_, h_b_ + sizeof h_b_ / sizeof h_b_[0]);

        nearness_control_msgs::FourierCoefsMsg h_fourier_coefs_msg;

        h_fourier_coefs_msg.header.stamp = ros::Time::now();
        h_fourier_coefs_msg.a = h_a_vector;
        h_fourier_coefs_msg.b = h_b_vector;

        pub_h_fourier_coefficients_.publish(h_fourier_coefs_msg);

} // End of computeHorizFourierCoeffs
    computeWFYawRateCommand();

       // Determine motion state ( safety box stuff)
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
