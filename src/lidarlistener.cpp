
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
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Pose.h>
#include <math.h>
#include <numeric>
#include <iterator>
#include <vector>
#include <algorithm>
#include <cstdlib>
#include <std_msgs/String.h>
#include <iostream>
#include <random>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/TransformStamped.h>
#include <nearness_control_msgs/FourierCoefsMsg.h>
#include <nearness_control_msgs/ClusterMsg.h>


// #include <std_srvs/SetBool.h>

//init nearness controller *************************************
using namespace cv;
using namespace std;

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
int h_num_fourier_terms_;
cv::Mat h_depth_cvmat_; 
float h_a_[10], h_b_[10];
cv::Mat h_nearness_;
float h_wf_r_cmd_;
geometry_msgs::TwistStamped control_command_;
float h_dg_;
<<<<<<< Updated upstream
int total_h_scan_points_ = 360; //double check, use this for now
double h_sensor_max_dist_ = 12; //12 meters, not sure if should equal 6 though
double h_sensor_min_dist_ = 0.15; //15 cm I believe (doublecheck)
double h_scan_limit_ = 10; //what is this? chose 10 randomly for now
float h_nearness_maxval_;
=======
int total_h_scan_points_ = 360; 
double h_sensor_max_dist_ = 12; //12 M
double h_sensor_min_dist_ = 0.15; //15 cm 
double h_scan_limit_ = 3.14; 
float h_nearness_maxval_; // =?
>>>>>>> Stashed changes

vector<float> h_gamma_vector_;
bool enable_control;
bool enable_reverse;
<<<<<<< Updated upstream
=======
bool servo_release = false;
>>>>>>> Stashed changes
vector<float> scan_ranges;

// Generate the horizontal gamma vector  also replaced num_h_scan_points_ with total_h_scan_points
void horizGammaVector(){
    for(int i=0; i<total_h_scan_points_; i++){
        h_gamma_vector_.push_back((float(i)/float(total_h_scan_points_))*(2*h_scan_limit_) - h_scan_limit_);
    }
    h_dg_ = (2.0*h_scan_limit_)/total_h_scan_points_;
}

//Safety box stuff
float radial_dist = 0.18;
float r_dist = 0.45;
bool boundary_stop = false;
bool left_side_flag = false;
bool right_side_flag = false;
bool safety_box_on = false;
vector<float>  safety_boundary_;
vector<float> box_index_;
float num_indices_;

//add controller gains
<<<<<<< Updated upstream
    double u_k_hb_1_; // =? //to start with for now
    double u_k_hb_2_; // =? //to start with for now
    double u_k_ha_1_; // =? //to start with for now
    double u_k_ha_2_; // =? //to start with for now
    double r_k_hb_1_; // =? //to start with for now
    double r_k_hb_2_; // =? //to start with for now
    double r_k_vb_1_; // =? //to start with for now
    double r_k_vb_2_; // =? //to start with for now
    double r_max_;
    double u_cmd_ = 1;
    
=======

    double r_k_hb_1_ = 2.0; //to start with for now
    double r_k_hb_2_ = 2.0; // =? //to start with for now

    double r_max_;  //=1;// //maximum turn value, determined through testing
    double u_cmd_ = 60; //forward speed command, determine during testing
    double rev_u_cmd_ = -u_cmd_;  //reverse speed command
>>>>>>> Stashed changes


//Callbacks*****************************************************
//callback for enable control
void enableControlCallback(const std_msgs::BoolConstPtr& msg){
  enable_control = msg->data;
}
void enableReverseCallback(const std_msgs::BoolConstPtr& msg){
  enable_reverse = msg->data;
}
// to enable in terminal, $ rostopic pub enable_control std_msgs/bool "true"

//callback for reverse control
void enableReverseCallback(const std_msgs::BoolConstPtr& msg){
  enable_reverse = msg->data;
}

//do we need a callback like this for boundary_stop?
/*
void enableBoundaryStopCallback(const std_msgs::BoolConstPtr& msg){
  boundary_stop = msg->data;
}
*/


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
//end call backs ********************************************************





// safety box stuff: expects a scan 0-2pi from due right (0deg) CCW to create safety box and compare to

void generateSafetyBox(){  // creates a safety box around the vehicle
  // polar format: 0-2pi in degrees with corresponding radius: safety_boundary
  // Box is rectangular in the rear, front is semiciricular
  // Radius is just above the minimum sensing distance of lidar in front 15+3cm
  
    // ("Generating safety box."); // print with ROS_INFO
    
    // Generate the left boundary
    for(int i = 0; i < total_h_scan_points_; i++){

      if(h_dg_*i < h_scan_limit_ ){ //from 0-pi
        safety_boundary_.push_back(radial_dist);
      }
      else if( ((h_dg_*i)-h_scan_limit_) < atan(r_dist/radial_dist) ){ //pi to back left corner
        safety_boundary_.push_back(radial_dist/cos((h_dg_*i)-h_scan_limit_));
      }
      else if( ((h_dg_*i)-(1.5*h_scan_limit_)) < atan(radial_dist/r_dist)){ //back left to back right
        safety_boundary_.push_back(r_dist/sin((h_dg_*i)-h_scan_limit_));
      }
      else{ //remaining, back right to 0
        safety_boundary_.push_back(radial_dist/sin((h_dg_*i)-(1.5*h_scan_limit_)));
      }
    }
    safety_box_on = true;  // what is this for?
  }

void checkSafetyBox(std::vector<float> scan_ranges){
  /*
  flag_too_close_front_ = false;?
  flag_too_close_left_ = false; already declared globally?
  flag_too_close_right_ = false;?
  */
  int left_ = 0;
  int right_ = 0;
  int rear_ = 0;
  for(int i = 0; i < total_h_scan_points_; i++){
    if((scan_ranges[i] < safety_boundary_[i])){
      box_index_.push_back(i);
    }
  }
  num_indices_ = box_index_.size();
  int num_ind = (int) num_indices_;//convert unsigned int to int
  if (num_ind == 0){
    return ;
        // cout << "No boundary violation" << endl; or equiv with ROS
  }
  else{
    for(int i = 0; i < num_ind; i++){
      if ((h_dg_*box_index_[i] < h_scan_limit_/2)||(h_dg_*box_index_[i] >= (((3/2)*M_PI) + atan(radial_dist/r_dist)))){
        right_++;
      }
      else if ((h_dg_*box_index_[i] >= h_scan_limit_/2 )&& (h_dg_*box_index_[i] < h_scan_limit_+ atan(r_dist/radial_dist))){
        left_++;
      }
      else { // add front flag??
        rear_++;
      }
    }
  }
    if(right_!=0){
      right_side_flag = true;
    }
    if(left_!=0){
      left_side_flag = true;
    }
    if(right_side_flag && left_side_flag){
      boundary_stop = true;
    }
} 


// end safetybox *********************************************************




// Main begin ***********************************************************
int main(int argc, char **argv)
{
// initialize ros
    ros::init(argc, argv, "lidarlistener");
    ros::NodeHandle n;

//Subscribers Setup**********************************************
//ros subscribers to laserscan
    ros::Subscriber sub_laserscan = n.subscribe("/scan", 1000, scanCallback);
//ros subscriber to enable control 
    ros::Subscriber sub_enable_control = n.subscribe("/enable_control", 1, enableControlCallback);
    enable_control = false;
//ros subscriber to enable reverse
    ros::Subscriber sub_enable_reverse = n.subscribe("/enable_reverse", 1, enableReverseCallback);
    enable_reverse = false;
<<<<<<< Updated upstream
=======
//is a subscriber needed for safety box stop?
/*
    ros::Subscriber sub_boundary_stop = n.subscribe("/boundary_stop", 1, enableBoundaryStopCallback);
    boundary_stop = false;
*/

>>>>>>> Stashed changes
//any other subscribers needed? (besides arduino rosserial)
//***************************************************************

//publishers set up**********************************************
// changed all "nh_." to "n." , runs now, is this okay?
  ros::Publisher pub_h_scan_nearness_ = n.advertise<std_msgs::Float32MultiArray>("horiz_nearness", 10);
  ros::Publisher pub_h_recon_wf_nearness_ = n.advertise<std_msgs::Float32MultiArray>("horiz_recon_wf_nearness", 10);
  ros::Publisher pub_h_fourier_coefficients_ = n.advertise<nearness_control_msgs::FourierCoefsMsg>("horiz_fourier_coefficients", 10);  
ros::Publisher pub_control_commands_stamped_ = n.advertise<geometry_msgs::TwistStamped>("control_commands_stamped", 10); 
 ros::Publisher pub_control_commands_ = n.advertise<geometry_msgs::Twist>("control_commands", 10);

//***************************************************************

// While loop to start converting/calculating and pushing control commands

 while(ros::ok()){

// Convert incoming scan to cv matrix and reformat**************************
 vector<float> h_depth_vector = scan_ranges; 
 vector<float> h_depth_vector_noinfs = h_depth_vector;
 
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
    
 /* not included but option to...
 // Reformat the depth scan depending on the orientation of the scanner
    // scan_start_loc describes the location of the first scan index
// Reformat the depth scan depending on the orientation of the scanner
    // scan_start_loc describes the location of the first scan index
 // Trim the scan down if the entire scan is not being used
*/

// Publish the reformatted scan

// Last, convert to cvmat and saturate 

    h_depth_cvmat_ = cv::Mat(1,total_h_scan_points_, CV_32FC1);

    std::memcpy(h_depth_cvmat_.data, h_depth_vector.data(),  h_depth_vector.size()*sizeof(float)); // replace with h_depth_vector_trimmed if we trim
    h_depth_cvmat_.setTo(h_sensor_min_dist_, h_depth_cvmat_ < h_sensor_min_dist_);
    h_depth_cvmat_.setTo(h_sensor_max_dist_, h_depth_cvmat_ > h_sensor_max_dist_);

// END Convert incoming scan to cv matrix and reformat***************************

// Compute the Fourier harmonics of the signal***********************************

//computeHorizFourierCoeffs();
    float h_cos_gamma_arr[h_num_fourier_terms_ + 1][total_h_scan_points_];
    float h_sin_gamma_arr[h_num_fourier_terms_ + 1][total_h_scan_points_];

// Compute horizontal nearness
    h_nearness_ = cv::Mat::zeros(cv::Size(1, total_h_scan_points_), CV_32FC1); //num_h_scan_points_ replaced with total_h_scan_points_
    h_nearness_ = 1.0/ h_depth_cvmat_;
    std::vector<float> h_nearness_array(h_nearness_.begin<float>(), h_nearness_.end<float>());
    h_nearness_maxval_ = *std::max_element(h_nearness_array.begin(), h_nearness_array.end());

// Compute the Fourier Coefficients
    cv::Mat h_cos_gamma_mat(h_num_fourier_terms_ + 1, total_h_scan_points_, CV_32FC1, h_cos_gamma_arr);
    cv::Mat h_sin_gamma_mat(h_num_fourier_terms_ + 1, total_h_scan_points_, CV_32FC1, h_sin_gamma_arr);

//perform fourier projections
    for (int i = 0; i < h_num_fourier_terms_ + 1; i++) {
        for (int j = 0; j < total_h_scan_points_; j++) {
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
} 
// End of computeHorizFourierCoeffs******************************************

// Generate WF control commands*******************************************        
{
    h_wf_r_cmd_ = r_k_hb_1_*h_b_[1] + r_k_hb_2_*h_b_[2];
    
    // Saturate the wide field yaw rate command
    if(h_wf_r_cmd_ < -r_max_) {
        h_wf_r_cmd_ = -r_max_;
    } else if(h_wf_r_cmd_ > r_max_) {
        h_wf_r_cmd_ = r_max_;
    }

}
// End Generate control commands********************************************   


<<<<<<< Updated upstream
// Determine motion state ( safety box stuff, add in when code is more developed)
// End Generate control commands********************************************   
=======
// when boundary_stop is set to true, set enable_control to false 

        if(boundary_stop){
            enable_control = false;
            servo_release = true;

}

//End safety boundary/end boundary
>>>>>>> Stashed changes


// If statement for enable control ******************************
       if(enable_control){
           // Publish the real control commands with rosserial to arduino

                //real control command
                control_command_.twist.linear.x = u_cmd_;
                control_command_.twist.linear.y = 0;
                control_command_.twist.linear.z = 0;
                control_command_.twist.angular.z = h_wf_r_cmd_;

                //Publishing to arduino


}
// Else for zeros to control command ****************************
        
        else if(enable_reverse){

<<<<<<< Updated upstream

}
=======
    double rev_h_wf_r_cmd_ = -h_wf_r_cmd_;

                control_command_.twist.linear.x = rev_u_cmd_;
                control_command_.twist.linear.y = 0;
                control_command_.twist.linear.z = 0;
                control_command_.twist.angular.z = rev_h_wf_r_cmd_;

}
        

>>>>>>> Stashed changes
        else {
           // Publish zeros with rosserial
           //set commands to zero
                control_command_.twist.linear.x = 0;
                control_command_.twist.linear.y = 0;
                control_command_.twist.linear.z = 0;
                control_command_.twist.angular.z = 0;
          // Publish to arduino
<<<<<<< Updated upstream

=======
                if (servo_release){
// ros serial to release servo latch
                }
>>>>>>> Stashed changes
       }


 //check callbacks once
        ros::spinOnce();
    }




   return 0;

} //end  main
