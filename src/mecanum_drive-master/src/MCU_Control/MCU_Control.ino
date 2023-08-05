#include <ros.h>
#include <Servo.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/UInt32MultiArray.h>
#include "CytronMotorDriver.h"
#include "QuadEncoder.h"
#include <std_msgs/Int32.h>
Servo myservo; 

CytronMD LF(PWM_PWM, 1,0); 
CytronMD RF(PWM_PWM, 2,3);
CytronMD LB(PWM_PWM, 5,4);
CytronMD RB(PWM_PWM, 10,11);

uint32_t LFcurPosVal;
uint32_t RFcurPosVal;
uint32_t LBcurPosVal;
uint32_t RBcurPosVal;

QuadEncoder LFenc(1, 30, 31, 0); 
QuadEncoder RBenc(2, 33, 36, 0);
QuadEncoder LBenc(3, 37, 6, 0);
QuadEncoder RFenc(4, 7, 8, 0);

ros::NodeHandle  nh;

void moveRobot_cb(const std_msgs::Int32MultiArray& cmd_msg) 
{
  LF.setSpeed(cmd_msg.data[0]);
  LB.setSpeed(cmd_msg.data[1]);
  RF.setSpeed(cmd_msg.data[2]);
  RB.setSpeed(cmd_msg.data[3]);
}
int pos=0;
void movePen_cb(const std_msgs::Int32& pen_msg)
{
   if(pen_msg.data==1) //draw go down cw
   {
      
      myservo.write(10);              
      delay(500);                            
    }
    else //lift go up ccw
    {
       myservo.write(20);              
      delay(500); 
   }
}

ros::Subscriber<std_msgs::Int32MultiArray> sub_bot("wheels_desired_rate", moveRobot_cb);

std_msgs::UInt32MultiArray ticks_msg;
ros::Publisher ticks_pub("wheel_ticks", &ticks_msg);

ros::Subscriber<std_msgs::Int32> sub_pen("move_pen", movePen_cb);

void setup(){
   
   Serial.begin(9600);  
   nh.initNode();
   myservo.attach(23);
   nh.subscribe(sub_bot);
   nh.subscribe(sub_pen);
   
}

void loop()
{
    nh.spinOnce();
    delay(500);
}
