 
#define CUSTOM_SETTINGS
#define INCLUDE_TERMINAL_MODULE
#include <ros.h>
//#include <Dabble.h>
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
bool incoming_value;
bool enableControl;
bool enableReverse;
bool servoAttach;
bool servoRelease;

double h_wf_r_cmd;
double u_cmd;


SoftwareSerial serial(10,11);  
RoboClaw roboclaw(&serial,10000);

//line 30
#define address 0x80

ros::NodeHandle n;

void messageCb(const geometry_msgs::Twist& msg){
 u_cmd = msg.linear.x;
 h_wf_r_cmd = msg.angular.z;
 
 
}

void enableControlCb(const std_msgs::Bool& msg){
 enableControl = msg.data;
 
}

void enableReverseCb(const std_msgs::Bool& msg){
 enableReverse = msg.data;
 
}
//line 50
void servoAttachCb(const std_msgs::Bool& msg){
 servoAttach = msg.data;
 
}

void servoReleaseCb(const std_msgs::Bool& msg){
 servoRelease = msg.data;
 
}

ros::Subscriber<geometry_msgs::Twist> sub("control_commands", &messageCb );
ros::Subscriber<std_msgs::Bool> enable_Control("enable_controls", &enableControlCb );
ros::Subscriber<std_msgs::Bool> enable_Reverse("enable_reverse", &enableReverseCb );
ros::Subscriber<std_msgs::Bool> servo_Attach("servo_attach", &servoAttachCb );
ros::Subscriber<std_msgs::Bool> servo_Release("servo_release", &servoReleaseCb );


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
  //Dabble.begin(9600);      // This is the baude rate of the HM-10
  roboclaw.begin(38400);
  
  n.initNode();
  n.subscribe(sub);
  
  n.initNode();
  n.subscribe(enable_Control);
  
  n.initNode();
  n.subscribe(enable_Reverse);
  
  n.initNode();
  n.subscribe(servo_Attach);
  
  n.initNode();
  n.subscribe(servo_Release);
  
  
}
void loop() {
   

   geometry_msgs::Twist msg;
   std_msgs::Bool msgs;

   
    //msg = incoming_value;


//Dabble.processInput();             //this function is used to refresh data obtained from smartphone.Hence calling this function is mandatory in order to get data properly from your mobile.
  
 if(Serial.available())
  {
    incoming_value = Serial.read();   // //Read the incoming data and store it into variable Incoming_value
  
    Serial.print(incoming_value);
    
    float sat_u_cmd = sat(0,120,u_cmd);
    float sat_h_wf_r_cmd = sat(0,10,h_wf_r_cmd);
    
    float val1 = sat_u_cmd - sat_h_wf_r_cmd;
    float val2 = -sat_u_cmd + sat_h_wf_r_cmd;
    
        if(enableControl == true)
            {
             Serial.print("Forward");
             roboclaw.ForwardM1(address, val1); 
             roboclaw.ForwardM2(address, val2); 
      
               if (servoAttach)
                { 
                myservo.attach(9);
              //servoAttach == false;
                   for (pos1 = 0; pos1 < 90; pos1 += 1) { // goes from 0 degrees to 180 degrees
                  // in steps of 1 degree
                  myservo.write(pos1);              // tell servo to go to position in variable 'pos'
                  delay(1);
                     }
                }
               else if (servoRelease )
                {
                //servoRelease == false;
                myservo.detach(); 
               }
               else
                {
              myservo.detach();
          
                 }
        
           }

      
  else if (enableReverse == true )
    {
      Serial.print("Backward");
      roboclaw.BackwardM1(address, val1);
      roboclaw.BackwardM2(address,val2);
      myservo.detach();
    }

  else
   {
    Serial.print("Stop");
    roboclaw.ForwardM1(address, 0); 
    roboclaw.ForwardM2(address, 0);
    myservo.detach();
   }
    

  }
  
   n.spinOnce();

}
