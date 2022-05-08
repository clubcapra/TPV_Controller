#include <stdlib.h>
#include <ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Float64.h>
#include <Servo.h>
#include <FeedBackServo.h>

#define KP -0.2
//Dave was here
#define G_X 0.7
#define G_Y 0.7

//Command to send
float cmd1 = 0;
float cmd2 = 0;

//Live position of servos
float pos1 = 0;
float pos2 = 0;

ros::NodeHandle nh;

Servo servo1;
Servo servo2;


long lasttime;

void messageServo1(const std_msgs::Float64& toggle_msg){
  cmd1 = toggle_msg.data;
}
void messageServo2(const std_msgs::Float64& toggle_msg){
  cmd2 = toggle_msg.data;
}

std_msgs::Float64 tpv_pos_x;
std_msgs::Float64 tpv_pos_y;

ros::Subscriber<std_msgs::Float64> sub1("tpv_x", &messageServo1);
ros::Subscriber<std_msgs::Float64> sub2("tpv_y", &messageServo2);
ros::Publisher pub1("tpv_pos_x", &tpv_pos_x);
ros::Publisher pub2("tpv_pos_y", &tpv_pos_y);

float angle1, angle2;

void setup()
{
  servo1.attach(5);
  servo2.attach(6);
  nh.getHardware()->setBaud(57600);
  nh.initNode();
  nh.subscribe(sub1);
  nh.subscribe(sub2);
  nh.advertise(pub1);
  nh.advertise(pub2);

  attachInterrupt(digitalPinToInterrupt(2), interupt1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(3), interupt2, CHANGE);
}



int unitsFC = 360;                          // Units in a full circle
int dutyScale = 1000;                       // Scale duty cycle to 1/1000ths
int dcMin = 31;                            // Minimum duty cycle
int dcMax = 971;                            // Maximum duty cycle
int q2min = unitsFC/4;                      // For checking if in 1st quadrant
int q3max = q2min * 3;                      // For checking if in 4th quadrant

int turns1 = 0;
unsigned long rise1, fall1, tLow1, tHigh1;
unsigned long rise2, fall2, tLow2, tHigh2;
float thetaPre1, thetaPre2;
int turns2 = 0;

void loop()
{
//----------------------------------------------------------------
  if(1){
    int tCycle = tHigh1 + tLow1;
    if((tCycle > 1000) && (tCycle < 1200)){
      float dc = (dutyScale * tHigh1) / (float)tCycle;
      float theta = (unitsFC - 1) - ((dc - dcMin) * unitsFC) / (dcMax - dcMin + 1);
    
      
      if(theta < 0.0)
          theta = 0.0;
      else if(theta > (unitsFC - 1.0))
          theta = unitsFC - 1.0;
    
      if((theta < q2min) && (thetaPre1 > q3max))
          turns1++;
      else if((thetaPre1 < q2min) && (theta > q3max))
          turns1--;
    
      if(turns1 >= 0)
          angle1 = (turns1 * unitsFC) + theta;
      else if(turns1 < 0)
          angle1 = ((turns1 + 1) * unitsFC) - (unitsFC - theta);
    
      thetaPre1 = theta;
    }
  }

//----------------------------------------------------------------
  if(1){
    int tCycle = tHigh2 + tLow2;
    if((tCycle > 1000) && (tCycle < 1200)){
      float dc = (dutyScale * tHigh2) / (float)tCycle;
      float theta = (unitsFC - 1) - ((dc - dcMin) * unitsFC) / (dcMax - dcMin + 1);
    
      
      if(theta < 0.0)
          theta = 0.0;
      else if(theta > (unitsFC - 1.0))
          theta = unitsFC - 1.0;
    
      if((theta < q2min) && (thetaPre2 > q3max))
          turns2++;
      else if((thetaPre2 < q2min) && (theta > q3max))
          turns2--;
    
      if(turns1 >= 0)
          angle2 = (turns2 * unitsFC) + theta;
      else if(turns1 < 0)
          angle2 = ((turns2 + 1) * unitsFC) - (unitsFC - theta);
    
      thetaPre2 = theta;
    }
  }
//-------------------------------------------------------------------


  if(millis() > 200){
    pos1 = pos1 + G_X*cmd1*(millis() - lasttime);
    pos2 = pos2 + G_Y*cmd2*(millis() - lasttime);
  
    servo1.write(constrain(((angle1-pos1)*KP) + 90, 0 , 180));
    servo2.write(constrain(((angle2-pos2)*KP) + 90, 0 , 180));
    
    tpv_pos_x.data = angle1;
    pub1.publish(&tpv_pos_x);
  
    tpv_pos_y.data = angle2;
    pub1.publish(&tpv_pos_y);
    
    lasttime = millis();
    nh.spinOnce();
    delay(1);
  }
  else{
    pos1 = angle1;
    pos2 = angle2;
  }
}


void interupt1(){
  if(digitalRead(2)) {
    rise1 = micros();
    tLow1 = rise1 - fall1;

  }
  else{
    fall1 = micros();
    tHigh1 = fall1 - rise1;
  }
}

void interupt2(){
  if(digitalRead(3)) {
    rise2 = micros();
    tLow2 = rise2 - fall2;

  }
  else{
    fall2 = micros();
    tHigh2 = fall2 - rise2;
  }
}
