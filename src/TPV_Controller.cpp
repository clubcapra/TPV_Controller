#include <stdlib.h>
#include <ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Float64.h>
#include <Servo.h>

#define UPDATE_RATE 60                //UPDATE RATE IN HZ
#define UPDATE_PERIOD 1/UPDATE_RATE   //UPDATE RATE IN S

#define SERVO1_CMD_PIN 5
#define SERVO1_FB_PIN  2
#define SERVO2_CMD_PIN 6
#define SERVO2_FB_PIN  3

#define VBUS1_PIN  A0
#define VBUS2_PIN  A1

#define VBUS_RATIO 6.23

struct fb_servo { //Feedback servo struct
  int turns = 0;
  unsigned long rise=0, fall=0, tLow=0, tHigh=0, last_reading = 0, last_command = 0;
  float thetaPre = 0;
  float rate = 0;  //velocity command
  float cmd = 0;
  float pos = 0;
  float rate_gain = 0.7;
  float kp_gain = -0.2;
  int valid = 0;
  Servo servo_handle;
};

void interupt1();
void interupt2();
void messageServo1(const std_msgs::Float64& toggle_msg);
void messageServo2(const std_msgs::Float64& toggle_msg);
void fb_servo_update(fb_servo fb_servo_handle);
void fb_servo_calc_angle(fb_servo servo);

fb_servo servo1;
fb_servo servo2;

ros::NodeHandle nh;

std_msgs::Float64 tpv_pos_x;
std_msgs::Float64 tpv_pos_y;
std_msgs::Float64 vbus1;
std_msgs::Float64 vbus2;

ros::Subscriber<std_msgs::Float64> sub1("tpv_x", &messageServo1);
ros::Subscriber<std_msgs::Float64> sub2("tpv_y", &messageServo2);
ros::Publisher pub1("tpv_pos_x", &tpv_pos_x);
ros::Publisher pub2("tpv_pos_y", &tpv_pos_y);
ros::Publisher pub3("vbus1", &vbus1);
ros::Publisher pub4("vbus2", &vbus2);

void setup()
{
  servo1.servo_handle.attach(SERVO1_CMD_PIN);
  servo2.servo_handle.attach(SERVO2_CMD_PIN);

  nh.getHardware()->setBaud(57600);
  nh.initNode();
  nh.subscribe(sub1);
  nh.subscribe(sub2);
  nh.advertise(pub1);
  nh.advertise(pub2);
  nh.advertise(pub3);
  nh.advertise(pub4);

  attachInterrupt(digitalPinToInterrupt(SERVO1_FB_PIN), interupt1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(SERVO2_FB_PIN), interupt2, CHANGE);

  delay(500);

  fb_servo_calc_angle(servo1);
  fb_servo_calc_angle(servo2);

  if(servo1.valid){
    servo1.cmd = servo1.pos;
  }
  if(servo2.valid){
    servo2.cmd = servo1.pos;
  }
}

void loop()
{
    unsigned long timestamp = micros();
    fb_servo_update(servo1);
    fb_servo_update(servo2);
    
    tpv_pos_x.data = servo2.pos;
    pub1.publish(&tpv_pos_x);
  
    tpv_pos_y.data = servo1.pos;
    pub1.publish(&tpv_pos_y);

    vbus1.data = analogRead(VBUS1_PIN) * VBUS_RATIO;
    pub3.publish(&vbus1);

    vbus2.data = analogRead(VBUS2_PIN) * VBUS_RATIO;
    pub4.publish(&vbus2);

    nh.spinOnce();
    while(micros()-timestamp < UPDATE_PERIOD * 1000){

    }
}


void messageServo1(const std_msgs::Float64& toggle_msg){
  servo1.rate = toggle_msg.data;
}
void messageServo2(const std_msgs::Float64& toggle_msg){
  servo2.rate = toggle_msg.data;
}

void fb_servo_update(fb_servo fb_servo_handle){
  fb_servo_calc_angle(fb_servo_handle);
  if(fb_servo_handle.valid){
    fb_servo_handle.cmd = fb_servo_handle.pos + fb_servo_handle.rate_gain*fb_servo_handle.rate*(millis() - fb_servo_handle.last_command);
    fb_servo_handle.servo_handle.write(constrain(((fb_servo_handle.cmd-fb_servo_handle.pos)*fb_servo_handle.kp_gain) + 90, 0 , 180));
  }
  else{
    fb_servo_handle.servo_handle.write(90);
  }
}

void fb_servo_calc_angle(fb_servo fb_servo_handle){
  int unitsFC = 360;                          // Units in a full circle
  int dutyScale = 1000;                       // Scale duty cycle to 1/1000ths
  int dcMin = 31;                            // Minimum duty cycle
  int dcMax = 971;                            // Maximum duty cycle
  int q2min = unitsFC/4;                      // For checking if in 1st quadrant
  int q3max = q2min * 3;                      // For checking if in 4th quadrant

  int tCycle = fb_servo_handle.tHigh + fb_servo_handle.tLow;
  if(micros() - servo1.last_reading < 1200){
    if((tCycle > 1000) && (tCycle < 1200)){
      fb_servo_handle.valid=1;

      float dc = (dutyScale * fb_servo_handle.tHigh) / (float)tCycle;
      float theta = (unitsFC - 1) - ((dc - dcMin) * unitsFC) / (dcMax - dcMin + 1);
      
      if(theta < 0.0)
          theta = 0.0;
      else if(theta > (unitsFC - 1.0))
          theta = unitsFC - 1.0;
    
      if((theta < q2min) && (fb_servo_handle.thetaPre > q3max))
          fb_servo_handle.turns++;
      else if((fb_servo_handle.thetaPre < q2min) && (theta > q3max))
          fb_servo_handle.turns--;
    
      if(fb_servo_handle.turns >= 0)
          fb_servo_handle.pos = (fb_servo_handle.turns * unitsFC) + theta;
      else if(fb_servo_handle.turns < 0)
          fb_servo_handle.pos = ((fb_servo_handle.turns + 1) * unitsFC) - (unitsFC - theta);
    
      fb_servo_handle.thetaPre = theta;
    }
    else{
      fb_servo_handle.valid=0;
      fb_servo_handle.last_command=micros();
    }
  }
  else{
    fb_servo_handle.valid=0;
    fb_servo_handle.last_command=micros();
  }

}

void interupt1(){
    if(!digitalRead(SERVO1_FB_PIN)) {
      servo1.rise = micros();
      servo1.tLow = servo1.rise - servo1.fall;
      servo1.last_reading = micros();
    }
    else{
      servo1.fall = micros();
      servo1.tHigh = servo1.fall - servo1.rise;
    }
}

void interupt2(){
  if(!digitalRead(SERVO2_FB_PIN)) {
    servo1.rise = micros();
    servo1.tLow = servo1.rise - servo1.fall;
    servo1.last_reading = micros();
  }
  else{
    servo1.fall = micros();
    servo1.tHigh = servo1.fall - servo1.rise;
  }
}
