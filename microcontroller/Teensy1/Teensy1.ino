#include <ros.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/Imu.h>
#include <Encoder.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include "TeensyThreads.h"
#define PI 3.1415926535897932384626433832795
#define USE_TEENSY_HW_SERIAL
//ThreadWrap(Serial, SerialX);
//#define Serial ThreadClone(SerialX)
/**********Function Prototypes**************/
 void frCb(const std_msgs::Float32& msg);
 void rrCb(const std_msgs::Float32& msg);
 void balCb(const std_msgs::Float32& msg);
 void fr_pid();
 void rr_pid();
 void bal_pid();
 void getimu();
 void calibrateSteering();
 /***********************************/
 
 /************VARIABLES********************/
Encoder front(6, 7);
Encoder rear(8, 9);
Encoder balancer(10, 11);
ros::NodeHandle nh;
std_msgs::Float32 frontFeedback;
std_msgs::Float32 rearFeedback;
std_msgs::Float32 balancerFeedback;
volatile sensor_msgs::Imu imu_dat;
ros::Publisher fr_feedback("bike/out/steering/front", &frontFeedback);
ros::Publisher rr_feedback("bike/out/steering/rear", &rearFeedback);
ros::Publisher bal_feedback("bike/out/balancer", &balancerFeedback);
ros::Publisher imupub("/imu/data", &imu_dat);
ros::Subscriber<std_msgs::Float32> fr_command("bike/in/steering/front", &frCb);
ros::Subscriber<std_msgs::Float32> rr_command("bike/in/steering/rear", &rrCb);
ros::Subscriber<std_msgs::Float32> bal_command("bike/in/balancer", &balCb);

volatile short int frSp = 0;
volatile short int rrSp = 0;
volatile short int balSp = 0;
volatile short int frErr = 0;
volatile short int rrErr = 0;
volatile short int balErr = 0;

Adafruit_BNO055 bno = Adafruit_BNO055(55,0x28);
volatile sensors_event_t orientationData , angVelocityData , linearAccelData;
/************************************************/
void setup()
{
  pinMode(0, OUTPUT);//front steering pwm
  pinMode(1, OUTPUT);//front steering direction
  pinMode(2, OUTPUT);//rear steering pwm
  pinMode(3, OUTPUT);//rear steering direction
  pinMode(4, OUTPUT);//balancer pwm
  pinMode(5, OUTPUT);//balancer direction
  //pinMode(15, INPUT);//calibration reset button
  //attachInterrupt(digitalPinToInterrupt(15), calibrateSteering, CHANGE);
  if(!bno.begin())
  {
   //if not detected, infinite loop
    while(1);
  }
  delay(1000);//wait for imu
  //nh.getHardware()->setBaud(256000);
  nh.initNode();
  nh.advertise(fr_feedback);
  nh.advertise(rr_feedback);
  nh.advertise(bal_feedback);
  nh.advertise(imupub);
  nh.subscribe(fr_command);
  nh.subscribe(rr_command);
  nh.subscribe(bal_command);

  /*****IMU Stuff***********/
  bno.setExtCrystalUse(true);
  imu_dat.header.frame_id = "imu";
  /*************************/
  
  /****Threads Start********/
  threads.addThread(fr_pid);
  threads.addThread(rr_pid);
  threads.addThread(bal_pid);
  threads.addThread(getimu);
  /***********************/
}

unsigned long startMillis;
unsigned long currentMillis;
int delaytime;
void loop()
{
  startMillis=millis();
  frontFeedback.data = front.read();
  rearFeedback.data = rear.read();
  balancerFeedback.data = balancer.read();
  fr_feedback.publish(&frontFeedback);
  rr_feedback.publish(&rearFeedback);
  bal_feedback.publish(&balancerFeedback);
  imupub.publish(&imu_dat);//slows down serial, comment for debugging
  nh.spinOnce();
  currentMillis=millis();
  currentMillis=currentMillis-startMillis;
  delaytime=10-currentMillis-1;
  if(delaytime<0) delaytime=0;
  threads.delay(delaytime);
}

void frCb(const std_msgs::Float32& msg){
  frSp=msg.data;
}

void rrCb(const std_msgs::Float32& msg){
  rrSp=msg.data;
}

void balCb(const std_msgs::Float32& msg){
  balSp=msg.data;
}

void fr_pid(){//ISR for front steering stepper
  while(1){
    /*
  if(abs(myEnc.read())>1400){ //for max and min turning angle
    analogWrite(11, 0);
    return;
  }*/
    frErr=frSp-front.read();
    if(frErr>=0){
      digitalWrite(1, LOW);
    }else{
      digitalWrite(1, HIGH);
      frErr=frErr*(-1);
    }
    if(frErr*100>60000){
      analogWriteFrequency(0, 60000);
    }else{
      analogWriteFrequency(0, frErr*100);
    }
    analogWrite(0, 200);
    threads.delay(2); 
  }
}

void rr_pid(){//ISR for front steering stepper
  while(1){
    /*
  if(abs(myEnc.read())>1400){
    analogWrite(11, 0);
    return;
  }*/
    rrErr=rrSp-rear.read();
    if(rrErr>=0){
      digitalWrite(3, LOW);
    }else{
      digitalWrite(3, HIGH);
      rrErr=rrErr*(-1);
    }
    if(rrErr*100>60000){
      analogWriteFrequency(2, 60000);
    }else{
      analogWriteFrequency(2, rrErr*100);
    }
    analogWrite(2, 200);
    threads.delay(2);
  }
}

void bal_pid(){//ISR for front steering stepper
  while(1){
    /*
  if(abs(myEnc.read())>1400){
    analogWrite(11, 0);
    return;
  }*/
    balErr=balSp-balancer.read();
    if(balErr>=0){
      digitalWrite(5, LOW);
    }else{
      digitalWrite(5, HIGH);
      balErr=balErr*(-1);
    }
    if(balErr*100>60000){
      analogWriteFrequency(4, 60000);
    }else{
      analogWriteFrequency(4, balErr*100);
    }
    analogWrite(4, 200);
    threads.delay(2);
  }
}

void getimu(){
  while(1){
    bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
    bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
    bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
    imu_dat.orientation.x = orientationData.orientation.x;
    imu_dat.orientation.y = orientationData.orientation.y;
    imu_dat.orientation.z = orientationData.orientation.z;
    imu_dat.angular_velocity.x = angVelocityData.gyro.x
    imu_dat.angular_velocity.y = angVelocityData.gyro.y
    imu_dat.angular_velocity.z = angVelocityData.gyro.z
    imu_dat.linear_acceleration.x = linearAccelData.acceleration.x
    imu_dat.linear_acceleration.y = linearAccelData.acceleration.y
    imu_dat.linear_acceleration.z = linearAccelData.acceleration.z
    imu_dat.header.stamp = nh.now();
    threads.delay(10); //100 hz loop
  }
}

void calibrateSteering() {
  front.readAndReset();
  rear.readAndReset();
  balancer.readAndReset();
}
