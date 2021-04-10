#include <ros.h>
#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif
#include <HardwareSerial.h>
#include <SoftwareSerial.h>
#include "RoboClaw.h"

#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Twist.h>

#include <Servo.h>
Servo myservo;
int pos = 90;
int pos1 = 0;

bool enableControl;
bool enableReverse;
double u_cmd = 0.0;
double h_wf_r_cmd = 0.0;

SoftwareSerial mySerial(10,11);
RoboClaw roboclaw(&mySerial,10000);

#define address 0x80

ros::NodeHandle n;

void messageCb(geometry_msgs::Twist& msg){
 u_cmd = msg.linear.x;
 h_wf_r_cmd = msg.angular.z;
  
  
}
void enableControlCb(const std_msgs::Bool& msg){
 enableControl = msg.data;
 
}
void enableReverseCb(const std_msgs::Bool& msg){
 enableReverse = msg.data;
 
}

ros::Subscriber<geometry_msgs::Twist> sub("control_commands", &messageCb );
ros::Subscriber<std_msgs::Bool> enable_Control("enable_controls", &enableControlCb );
ros::Subscriber<std_msgs::Bool> enable_Reverse("enable_reverse", &enableReverseCb );

float sat(float num, float min_val, float max_val){
   if (num >= max_val){
        return max_val;
    } else if (num <= min_val){
        return min_val;
    } else {
      return num;
    }
}




void setup() {
  Serial.begin(57600);  // Set your Serial Monitor is set at 250000
  roboclaw.begin(9600);
  n.initNode();
  n.subscribe(sub);
  n.subscribe(enable_Control);
  n.subscribe(enable_Reverse);

  
}
void loop() {
    
    float sat_u_cmd = sat(u_cmd,0,1.0);
    float sat_h_wf_r_cmd = sat(h_wf_r_cmd,-1,1);
    
    float val1 = sat_u_cmd + sat_h_wf_r_cmd;
    float val2 = sat_u_cmd - sat_h_wf_r_cmd;
    //float val1 = sat_u_cmd;
    //float val2 = sat_u_cmd;
    
    int right_speed = int(val1*128);
    int left_speed = int(val2*128);


     Serial.print("loop");
    
        if(enableControl == true){

             if (val1 < 0){
              val1 = 0;
              right_speed = int(val1*128);
         
             roboclaw.BackwardM1(address, right_speed); 
             roboclaw.BackwardM2(address, left_speed);
             }
             
             else if (val2 < 0){
              val2 = 0;
              left_speed = int(val2*128);

             roboclaw.BackwardM1(address, right_speed); 
             roboclaw.BackwardM2(address, left_speed);
             }
             else 
             roboclaw.BackwardM1(address, right_speed); 
             roboclaw.BackwardM2(address, left_speed);

             
//      myservo.attach(9);
//      for (pos = 90; pos >= 1; pos -= 1) { // goes from 0 degrees to 180 degrees
//    // in steps of 1 degree
//     myservo.write(pos);              // tell servo to go to position in variable 'pos'
//      delay(20);
//      }  
       //myservo.detach();
            
      
           }
          
      
  else if (enableReverse == true )
    {
      roboclaw.ForwardM1(address, 30);
      roboclaw.ForwardM2(address,30);
      myservo.detach();
    }
  
  else
   {
    roboclaw.ForwardM1(address, 0); 
    roboclaw.ForwardM2(address, 0);
    myservo.detach();
    
 }
   

 
   n.spinOnce();
   delay(10);

}
