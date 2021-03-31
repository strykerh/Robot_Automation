
#include <ros.h>
#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif
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

float h_wf_r_cmd;
float u_cmd;


SoftwareSerial serial(10,11);  
RoboClaw roboclaw(&serial,10000);

#define address 0x80

ros::NodeHandle n;

void messageCb(geometry_msgs::Twist& msg){
 u_cmd == msg.linear.x;
 h_wf_r_cmd == msg.angular.z;
 //digitalWrite(13, HIGH-digitalRead(13));  //toggle led  
  
}
void enableControlCb(const std_msgs::Bool& msg){
 enableControl == msg.data;
 
}
void enableReverseCb(const std_msgs::Bool& msg){
 enableReverse == msg.data;
 
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
  //myservo.attach(9);
  Serial.begin(9600);      // Set your Serial Monitor is set at 250000
  
  n.initNode();
  n.subscribe(sub);
  
  n.initNode();
  n.subscribe(enable_Control);
  
  n.initNode();
  n.subscribe(enable_Reverse);

  
}
void loop() {

    
    float sat_u_cmd = sat(0,120,u_cmd);
    float sat_h_wf_r_cmd = sat(0,10,h_wf_r_cmd);
    
    float val1 = sat_u_cmd - sat_h_wf_r_cmd;
    float val2 = -sat_u_cmd + sat_h_wf_r_cmd;


    
        if(enableControl == true)
            {
                
             //Serial.print("Forward");
             roboclaw.BackwardM1(address, val1); 
             roboclaw.BackwardM2(address, val2); 
         
            
       n.spinOnce();
           }
          
      
  else if (enableReverse == true )
    {
      //Serial.print("Backward");
      roboclaw.ForwardM1(address, 40);
      roboclaw.ForwardM2(address,40);
      myservo.detach();
    // n.spinOnce();
     //delay(1);
    }
  
  else
   {
    //Serial.print("Stop");
    roboclaw.ForwardM1(address, 0); 
    roboclaw.ForwardM2(address, 0);
    myservo.detach();
    n.spinOnce();
    
 }
   

  
   n.spinOnce();

}
