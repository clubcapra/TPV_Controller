#include <stdlib.h>
#include <ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Float64.h>
#include <Servo.h>

#define UPDATE_RATE 15.0                //UPDATE RATE IN HZ
#define UPDATE_PERIOD 1.0/UPDATE_RATE   //UPDATE RATE IN S

#define SERVO1_CMD_PIN 5
#define SERVO1_FB_PIN  2
#define SERVO2_CMD_PIN 6
#define SERVO2_FB_PIN  3

#define VBUS1_PIN  A0
#define VBUS2_PIN  A1

#define DOP1_PIN  A2
#define DOP2_PIN  A3
#define DOP3_PIN  A4
#define DOP4_PIN  A5

#define X_OFFSET -275
#define Y_OFFSET -35


#define VBUS_RATIO 6.23

struct fb_servo { //Feedback servo struct
  int turns = 0;
  unsigned long rise=0, fall=0, tLow=0, tHigh=0, last_reading = 0, last_command = 0;
  float thetaPre = 0;
  float rate = 0;  //velocity command
  float cmd = 0;
  float pos = 0;
  float rate_gain = 4;
  float kp_gain = 0.1;
  float ki_gain = 0.001;
  float i = 0;
  float i_max = 800;
  int valid = 0;
  Servo servo_handle;
};

void messageServo1(const std_msgs::Float64& toggle_msg);
void messageServo2(const std_msgs::Float64& toggle_msg);
void messageDop1(const std_msgs::UInt16& toggle_msg);
void messageDop2(const std_msgs::UInt16& toggle_msg);
void messageDop3(const std_msgs::UInt16& toggle_msg);
void messageDop4(const std_msgs::UInt16& toggle_msg);

void fb_servo_update(fb_servo* fb_servo_handle);
void fb_servo_calc_angle(fb_servo* servo);
void interupt1();
void interupt2();

fb_servo servo1;
fb_servo servo2;

ros::NodeHandle nh;

std_msgs::Float64 tpv_pos_x;
std_msgs::Float64 tpv_pos_y;
std_msgs::Float64 vbus1;
std_msgs::Float64 vbus2;

ros::Subscriber<std_msgs::Float64> sub1("tpv_x", &messageServo1);
ros::Subscriber<std_msgs::Float64> sub2("tpv_y", &messageServo2);
ros::Subscriber<std_msgs::UInt16> sub3("DOP1", &messageDop1);
ros::Subscriber<std_msgs::UInt16> sub4("DOP2", &messageDop2);
ros::Subscriber<std_msgs::UInt16> sub5("DOP3", &messageDop3);
ros::Subscriber<std_msgs::UInt16> sub6("DOP4", &messageDop4);

ros::Publisher pub1("tpv_pos_x", &tpv_pos_x);
ros::Publisher pub2("tpv_pos_y", &tpv_pos_y);
ros::Publisher pub3("vbus1", &vbus1);
ros::Publisher pub4("vbus2", &vbus2);

void setup()
{
  pinMode(DOP1_PIN,OUTPUT);
  pinMode(DOP2_PIN,OUTPUT);
  pinMode(DOP3_PIN,OUTPUT);
  pinMode(DOP4_PIN,OUTPUT);

  servo1.servo_handle.attach(SERVO1_CMD_PIN);
  servo2.servo_handle.attach(SERVO2_CMD_PIN);

  nh.getHardware()->setBaud(57600);
  nh.initNode();

  nh.subscribe(sub1);
  nh.subscribe(sub2);
  nh.subscribe(sub3);
  nh.subscribe(sub4);
  nh.subscribe(sub5);
  nh.subscribe(sub6);

  nh.advertise(pub1);
  nh.advertise(pub2);
  nh.advertise(pub3);
  nh.advertise(pub4);

  attachInterrupt(digitalPinToInterrupt(SERVO1_FB_PIN), interupt1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(SERVO2_FB_PIN), interupt2, CHANGE);

  delay(500);
  servo1.kp_gain = -70;
  servo2.kp_gain = 25;

  fb_servo_calc_angle(&servo1);
  fb_servo_calc_angle(&servo2);

  if(servo1.valid){
    servo1.cmd = servo1.pos;
  }
  if(servo2.valid){
    servo2.cmd = servo2.pos;
  }
}

void loop()
{
    unsigned long timestamp = micros();
    fb_servo_update(&servo1);
    fb_servo_update(&servo2);
    
    tpv_pos_x.data = servo1.pos - X_OFFSET;
    pub1.publish(&tpv_pos_x);
  
    tpv_pos_y.data = servo2.pos - Y_OFFSET;
    pub2.publish(&tpv_pos_y);
    
    Serial.println(servo1.tLow);

    vbus1.data = analogRead(VBUS1_PIN)/1023.0 * 5 * VBUS_RATIO;
    pub3.publish(&vbus1);

    vbus2.data = analogRead(VBUS2_PIN)/1023.0* 5 * VBUS_RATIO;
    pub4.publish(&vbus2);

    nh.spinOnce();


    while(micros()-timestamp < UPDATE_PERIOD * 1000000.0){
    }
}

void messageDop1(const std_msgs::UInt16& toggle_msg){
  digitalWrite(DOP1_PIN, toggle_msg.data);
}

void messageDop2(const std_msgs::UInt16& toggle_msg){
  digitalWrite(DOP2_PIN, toggle_msg.data);
}

void messageDop3(const std_msgs::UInt16& toggle_msg){
  digitalWrite(DOP3_PIN, toggle_msg.data);
}

void messageDop4(const std_msgs::UInt16& toggle_msg){
  digitalWrite(DOP4_PIN, toggle_msg.data);
}

void messageServo1(const std_msgs::Float64& toggle_msg){
  servo1.rate = toggle_msg.data;
}

void messageServo2(const std_msgs::Float64& toggle_msg){
  servo2.rate = toggle_msg.data;
}

void fb_servo_update(fb_servo* fb_servo_handle){
  fb_servo_calc_angle(fb_servo_handle);
  if(fb_servo_handle->valid){
    fb_servo_handle->cmd = fb_servo_handle->cmd + fb_servo_handle->rate_gain*fb_servo_handle->rate;
    float error = (fb_servo_handle->cmd-fb_servo_handle->pos);
    fb_servo_handle->i = constrain(fb_servo_handle->i + error, -fb_servo_handle->i_max , fb_servo_handle->i_max);
    float servo_command = constrain((error*fb_servo_handle->kp_gain + fb_servo_handle->i*fb_servo_handle->ki_gain) + 90, 0 , 180);
    fb_servo_handle->servo_handle.write(fb_servo_handle->rate * fb_servo_handle->kp_gain + 90);
  }
  else{
    fb_servo_handle->servo_handle.write(90);
  }
}

void fb_servo_calc_angle(fb_servo* fb_servo_handle){
  int unitsFC = 360;                          // Units in a full circle
  int dutyScale = 1000;                       // Scale duty cycle to 1/1000ths
  int dcMin = 31;                            // Minimum duty cycle
  int dcMax = 971;                            // Maximum duty cycle
  int q2min = unitsFC/4;                      // For checking if in 1st quadrant
  int q3max = q2min * 3;                      // For checking if in 4th quadrant

  int tCycle = fb_servo_handle->tHigh + fb_servo_handle->tLow;
  if((micros() - fb_servo_handle->last_reading) < 1200){
    if((tCycle > 1000) && (tCycle < 1200)){
      fb_servo_handle->valid=1;

      float dc = (dutyScale * fb_servo_handle->tHigh) / (float)tCycle;
      float theta = (unitsFC - 1) - ((dc - dcMin) * unitsFC) / (dcMax - dcMin + 1);
      
      if(theta < 0.0)
          theta = 0.0;
      else if(theta > (unitsFC - 1.0))
          theta = unitsFC - 1.0;
    
      if((theta < q2min) && (fb_servo_handle->thetaPre > q3max))
          fb_servo_handle->turns++;
      else if((fb_servo_handle->thetaPre < q2min) && (theta > q3max))
          fb_servo_handle->turns--;
    
      if(fb_servo_handle->turns >= 0)
          fb_servo_handle->pos = (fb_servo_handle->turns * unitsFC) + theta;
      else if(fb_servo_handle->turns < 0)
          fb_servo_handle->pos = ((fb_servo_handle->turns + 1) * unitsFC) - (unitsFC - theta);
    
      fb_servo_handle->thetaPre = theta;
    }
    else{
      fb_servo_handle->valid=0;
      fb_servo_handle->last_command=micros();
    }
  }
  else{
    fb_servo_handle->valid=0;
    fb_servo_handle->last_command=micros();
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
    servo2.rise = micros();
    servo2.tLow = servo2.rise - servo2.fall;
    servo2.last_reading = micros();
  }
  else{
    servo2.fall = micros();
    servo2.tHigh = servo2.fall - servo2.rise;
  }
}
