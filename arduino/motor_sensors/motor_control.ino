#include <PID_v1.h> // PID library
#include <ros.h> // ROS serial
#include <ros/time.h>
#include "Wire.h" // IMU wire library
#include "I2Cdev.h"
#include <geometry_msgs/Vector3.h> 
#include <tf/transform_broadcaster.h>
#include <MPU6050.h> // MPU 6050 IMU library
#include <sensor_msgs/Range.h>
#include "NewPing.h"

MPU6050 mpu;

//Create an object that represents the ROS node
ros::NodeHandle nh;

//Create sensor messages
geometry_msgs::TransformStamped t; 
geometry_msgs::TransformStamped t2;
tf::TransformBroadcaster broadcaster;

//Encoders
const byte encoder0pinA = 3;// A pin for motor 1
const byte encoder0pinB = 42;// B pin for motor 1

const byte encoder1pinA = 2;// A pin for motor 2
const byte encoder1pinB = 48;// B pin for motor 2

byte encoder0PinALast; // Last encoder reading for motor 1
byte encoder1PinALast; // Last encoder reading for motor 2

double duration1; // Pulses/sec for the encoder in motor 1
double duration2; // Pulses/sec for the encoder in motor 2

double abs_duration1, abs_duration2; // Speed for motor 1 and 2

//Motor PWM
double PWM1, PWM2; // PWM signals for motor 1 and motor 2

double set1, set2 = 0; // Velocity set points for motor 1 and motor 2
double setanglespeed; // Angular speed set point
double setspeed; // Net speed

int r = 0.035; // Wheel radius (m)
int l = 0.11 ; // Robot track width (m)

boolean Direction1; // Motor 1 direction
boolean Direction2; // Motor 2 direction

const int ENA = 8; // Motor1 output pin
const int IN1 = 9; // Motor1 direction pin 1
const int IN2 = 10; // Motor1 direction pin 2

const int ENB = 13; // Motor2 output pin
const int IN3 = 11; // Motor2 direction pin 1
const int IN4 = 12; // Motor2 direction pin 2

//IMU
float yaw, yawRate,xAcc,yAcc = 0;
float timeStep = 0.01;

//Ultrasonic
const int ult1Trig = 6;
const int ult1Echo = 7;
const int ult2Trig = 4;
const int ult2Echo = 5;
int maxLength = 15;
NewPing Sonar1(ult1Trig, ult1Echo, maxLength);
NewPing Sonar2(ult2Trig, ult2Echo, maxLength);

sensor_msgs::Range sonar1;
sensor_msgs::Range sonar2;

ros::Publisher sonar_1("/sonar_1", &sonar1);
ros::Publisher sonar_2("/sonar_2", &sonar2);

float read1 = 0;
float read2 = 0;

char world[] = "world";
char id1[] ="/sonar1";
char id2[] ="/sonar2";
char imu[] = "/imu";


// Timers
unsigned long timer, range_timer = 0;

double set = 0; // Setpoint speed -- I guess will be given order from ROS
double val_output; // PWM output
double Kp=0.6, Ki=3.8, Kd=0; // PID gains
PID myPID1(&abs_duration1, &PWM1, &set1, Kp, Ki, Kd, DIRECT); // PID setup for motor 1
PID myPID2(&abs_duration2, &PWM2, &set1, Kp, Ki, Kd, DIRECT); // PID setup for motor 1
boolean result1, result2; // PID control outcome

void setup()
{
  nh.initNode();
  broadcaster.init(nh); 
  nh.advertise(sonar_1);
  nh.advertise(sonar_2);
  sonar1.radiation_type = sensor_msgs::Range::ULTRASOUND;
  sonar1.field_of_view = (10.0/180.0) * 3.14;
  sonar1.min_range = 0.0;
  sonar1.max_range = 20.0;
  sonar1.header.frame_id = id1;
  sonar2.radiation_type = sensor_msgs::Range::ULTRASOUND;
  sonar2.field_of_view = (10.0/180.0) * 3.14;
  sonar2.min_range = 0.0;
  sonar2.max_range = 20.0;
  sonar2.header.frame_id = id2;
  

  // Initialize MPU6050
  while(!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))
  {
    Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
    delay(500);
  }
  
  // Calibrate gyroscope. The calibration must be at rest.
  // If you don't want calibrate, comment this line.
  mpu.calibrateGyro();

  // Set threshold sensivty. Default 3.
  // If you don't want use threshold, comment this line or set 0.
  mpu.setThreshold(3);
  
  // Motor driver setup
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);

  // Encoder initialization
  EncoderInit();

  // PID setup
  myPID1.SetMode(AUTOMATIC);// PID is set to automatic mode
  myPID1.SetSampleTime(50);// PID sampling every 0.05 seconds
  myPID2.SetMode(AUTOMATIC);// PID is set to automatic mode
  myPID2.SetSampleTime(50);// PID sampling every 0.05 seconds
}

void loop()
{
  //Serial.print("Pulse:");
  //Serial.println(duration1);
  //Serial.print("Pulse:");
  //Serial.println(duration2);
  abs_duration1=abs(duration1);
  abs_duration2=abs(duration2);
  result1=myPID1.Compute();
  result2=myPID2.Compute();
  if(result1 && result2)
  {

    duration1, duration2 = 0; //Count clear, wait for the next count
    }
  timer = millis();
  Vector normGyro = mpu.readNormalizeGyro();
  Vector normAccel = mpu.readNormalizeAccel();
  
  if ( (millis()-range_timer) > 50){
    yaw = yaw+normGyro.ZAxis*(millis()-range_timer)*0.001;
    t.header.frame_id = world;
    t.child_frame_id = imu;
    t.transform.rotation.w = cos((yaw/2));
    t.transform.rotation.z = sin((yaw/2));
    t.transform.rotation.y = 0;
    t.transform.rotation.x = 0;
    t.header.stamp = nh.now();
    broadcaster.sendTransform(t); 
    t2.header.frame_id = imu;
    t2.child_frame_id = id1;
    t2.transform.rotation.w = 1;
    t2.transform.rotation.z = 0;
    t2.transform.rotation.y = 0;
    t2.transform.rotation.x = 0;
    t2.header.stamp = nh.now();
    broadcaster.sendTransform(t2);
    t2.child_frame_id = id2;
    broadcaster.sendTransform(t2);

    sonar1.range = Sonar1.ping_cm();
    sonar2.range = Sonar2.ping_cm();
    sonar1.header.stamp = nh.now();
    sonar2.header.stamp = nh.now();
    sonar_1.publish(&sonar1);
    sonar_2.publish(&sonar2);

    range_timer = millis();
    }
    nh.spinOnce();
}

void EncoderInit()
{
  Direction1 = true;//default -> Forward
  Direction2 = true;
  pinMode(encoder0pinB,INPUT);
  attachInterrupt(1, wheelSpeed1, CHANGE);
  attachInterrupt(0, wheelSpeed2, CHANGE);
}

void wheelSpeed1()
{
  int Lstate1 = digitalRead(encoder0pinA);
  if((encoder0PinALast == LOW) && Lstate1==HIGH)
  {
    int val1 = digitalRead(encoder0pinB);
    if(val1 == LOW && Direction1)
    {
      Direction1 = false; //Reverse
    }
    else if(val1 == HIGH && !Direction1)
    {
      Direction1 = true;  //Forward
    }
  }
  encoder0PinALast = Lstate1;

  if(!Direction1){
    duration1--;
  }
  else{
    duration1++;
  }
}

void wheelSpeed2()
{
  int Lstate2 = digitalRead(encoder1pinA);
  if((encoder1PinALast == LOW) && Lstate2==HIGH)
  {
    int val2 = digitalRead(encoder1pinB);
    if(val2 == LOW && Direction2)
    {
      Direction2 = false; //Reverse
    }
    else if(val2 == HIGH && !Direction2)
    {
      Direction2 = true;  //Forward
    }
  }
  encoder1PinALast = Lstate2;

  if(!Direction2){
    duration2++;
  }
  else{
    duration2--;
  }
}

// Motor 1 command
void move_motor1()
{
    // Saturation: Max PWM = 255 and Min PWM = -255
    if (PWM1>255){
      PWM1 = 250;
    }

    else if(PWM1<-255){
      PWM1 = -250;
    }

     // PWM + Direction
     analogWrite(ENA,PWM1);
     digitalWrite(IN1,HIGH);
     digitalWrite(IN2,LOW);
}

// Motor 2 command
void move_motor2()
{
    // Saturation: Max PWM = 255 and Min PWM = -255
    if (PWM2>255){
      PWM2 = 250;
    }

    else if(PWM2<-255){
      PWM2 = -250;
    }

     // PWM + Direction
     analogWrite(ENB,PWM2);
     digitalWrite(IN3,HIGH);
     digitalWrite(IN4,LOW);
}

void calculate(double thetadot, double setspeed){
  
}
