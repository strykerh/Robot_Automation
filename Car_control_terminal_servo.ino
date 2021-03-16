#define CUSTOM_SETTINGS
#define INCLUDE_TERMINAL_MODULE
#define INCLUDE_GAMEPAD_MODULE
//#include <Dabble.h>
#include <SoftwareSerial.h>
#include <RoboClaw.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Twist.h>

#include <Servo.h>
Servo myservo;
int pos = 90;
int pos1 = 0;
double incoming_value;


SoftwareSerial serial(10,11);  
RoboClaw roboclaw(&serial,10000);

#define address 0x80

ros::NodeHandle nh;

void messageCb(const geometry_msgs::Twist& msg){
 incoming_value = msg.data;
 
}

ros::Subscriber<geometry_msgs::Twist> sub("control_commands", &messageCb );

void setup() {
  myservo.attach(9);
  Serial.begin(9600);      // Set your Serial Monitor is set at 250000
  //Dabble.begin(9600);      // This is the baude rate of the HM-10
  roboclaw.begin(38400);
  nh.initNode();
  nh.subscribe(sub);
}
void loop() {
  while(ros::ok()){
  geometry_msgs::Twist msg;
  msg.data = incoming_value;

//Dabble.processInput();             //this function is used to refresh data obtained from smartphone.Hence calling this function is mandatory in order to get data properly from your mobile.
  
  if(Terminal.available())
  {
    incoming_value = Terminal.read();   // //Read the incoming data and store it into variable Incoming_value
    Serial.print(incoming_value);
    
    
    if(incoming_value == 'F')
     {
       Terminal.print(incoming_value);
       roboclaw.ForwardM1(address,60); 
       roboclaw.ForwardM2(address,60); 
      }

      
    else if (incoming_value == 'B')
    {
      roboclaw.BackwardM1(address,60);
      roboclaw.BackwardM2(address,60);
    }

      
    else if (incoming_value == 'R')
    {
      roboclaw.ForwardM2(address,50);
    }
    
    else if (incoming_value == 'L')
    {
      roboclaw.ForwardM1(address,50);
    }

    else if (incoming_value == 'X')
    {
      roboclaw.BackwardM1(address,0);
      roboclaw.BackwardM2(address,0);
    }

    else if (incoming_value == 'S')
    {
      myservo.attach(9);
      for (pos = 90; pos >= 1; pos -= 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
     myservo.write(pos);              // tell servo to go to position in variable 'pos'
      delay(20);
      }  
    }

    else if (incoming_value == 'O')
    { 
      myservo.attach(9);
      for (pos1 = 0; pos1 < 90; pos1 += 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
     myservo.write(pos1);              // tell servo to go to position in variable 'pos'
      delay(20);
      }

    }
    
     else 
      {
        roboclaw.ForwardM1(address,0);
        roboclaw.ForwardM2(address,0); 
      }
  }

  
  else 
  {
  
    myservo.detach(); 
  }
  

  }
  
  nh.spinOnce();
