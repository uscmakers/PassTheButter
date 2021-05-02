#include <PID_v1.h> // PID library
#include <ros.h> // ROS serial
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
#include <ros/time.h>

// Robot specs
double r = 0.035; // Wheel radius (m)
double l = 0.11 ; // Robot track width (m)

// Velocity setpoints for each motor
double velSet1 = 0; // Velocity set point for motor 1 [RPM]
double velSet2 = 0; // Velocity set point for motor 2 [RPM]

// Pins for the encoder on motor 1
const byte encoder0pinA = 2;// A pin for motor 1
const byte encoder0pinB = 4;// B pin for motor 1

// Pins for the encoder on motor 2
const byte encoder1pinA = 3;// A pin for motor 2
const byte encoder1pinB = 5;// B pin for motor 2

byte encoder0PinALast; // Last encoder reading for motor 1
byte encoder1PinALast; // Last encoder reading for motor 2

double duration1; // Speed for the encoder in motor 1 [RPM]
double duration2=0; // Displacement in motor 1's angular position [rev]
int pulse1; // Encoder 1 pulse

double duration3; // Speed for the encoder in motor 2 [RPM]
double duration4=0; // Displacement in motor 2's angular position [rev]
int pulse2; // Encoder 2 pulse

double abs_duration1; // Speed for motor 1 [RPM]
double abs_duration2 =0; // Displacement for motor 1 [rev]

double abs_duration3; // Speed for motor 2 [RPM]
double abs_duration4 =0; // Displacement for motor 2 [rev]

double PWM1; // PWM signals for motor 1
double PWM2; // PWM signals for motor 2

double velMax = 60; // Max velocity for motor 1 and 2 [RPM]

double posSet1=0; // Position set point for motor 1 [rev]
double posSet2=0; // Position set point for motor 2 [rev]

double pos1Abs; // Motor 1's absolute position value
double pos2Abs; // Motor 2's absolute position value

int motor1Sign =0; // Motor 1's rotational direction [cw/ccw]
int motor2Sign =0; // Motor 2's rotational direction [cw/ccw]

boolean result1; // Motor 1 speed PID control compute result
boolean result2; // Motor 2 speed PID control compute result
boolean result3; // Motor 1 position PID control compute result
boolean result4; // Motor 2 position PID control compute result

boolean Direction1; // Motor 1 direction
boolean Direction2; // Motor 2 direction

// Motor 1 pins
const int ENA = 6; // Motor1 output pin
const int IN1 = 7; // Motor1 direction pin 1
const int IN2 = 8; // Motor1 direction pin 2

// Motor 2 pins
const int ENB = 11; // Motor2 output pin
const int IN3 = 9; // Motor2 direction pin 1
const int IN4 = 10; // Motor2 direction pin 2

// Callback function that gives velocity and position input;
void subCallback(const geometry_msgs::Point& cmd_pos){
 
  if (abs(cmd_pos.x)!=0.0) // Input for the straight movement
  {
    duration2 = 0;
    pulse1 = 0;
    pulse2 = 0;
    duration4 = 0;
    posSet1 = 0;
    posSet2 = 0;

    if(cmd_pos.x>0){
      motor1Sign=1;
      motor2Sign=1;
    }

    else if(cmd_pos.x<0){
      motor1Sign=0;
      motor2Sign=0;
    }
    
    rotateMotor(abs(cmd_pos.x));

  }

  if(abs(cmd_pos.z)!=0.0) // Input for the rotational movement
  {
    duration2 = 0;
    pulse1 = 0;
    pulse2 = 0;
    duration4 = 0;
    posSet1 = 0;
    posSet2 = 0;


    if(cmd_pos.z>0){
      motor1Sign=0;
      motor2Sign=1;
    }

    else if(cmd_pos.z<0){
      motor1Sign=1;
      motor2Sign=0;
    }

  rotateMotor(abs(cmd_pos.z));
}
}

// ROS node for publisher and subscriber
ros::NodeHandle nh;

// Create ROS messages
geometry_msgs::Twist encoder1;
geometry_msgs::Twist encoder2;

// Subscriber for receiving message through commands
ros::Subscriber <geometry_msgs::Point> sub("/cmd_pos", &subCallback);

// Publisher for encoder data
ros::Publisher encoder_1("/encoder_1", &encoder1); // Motor 1 encoder publisher
ros::Publisher encoder_2("/encoder_2", &encoder2); // Motor 2 encoder publisher

double Kp11=2, Ki11=1.1, Kd11=0; // PID gains for motor 1 speed 
double Kp12=4, Ki12=1.4, Kd12=0; // PID gains for motor 2 speed 
double Kp21=908, Ki21=955, Kd21=0; // PID gains for motor 1 position
double Kp22=902, Ki22=858, Kd22=0; // PID gains for motor 2 position
PID myPIDVel1(&abs_duration1, &PWM1, &velSet1, Kp11, Ki11, Kd11, DIRECT); // PID setup for the speed of motor 1
PID myPIDVel2(&abs_duration3, &PWM2, &velSet2, Kp12, Ki12, Kd12, DIRECT); // PID setup for the speed of motor 2
PID myPIDPos1(&abs_duration2, &velSet1, &pos1Abs, Kp21, Ki21, Kd21, DIRECT); // PID setup for the position of motor 1
PID myPIDPos2(&abs_duration4, &velSet2, &pos2Abs, Kp22, Ki22, Kd22, DIRECT); // PID setup for the position of motor 1

void setup() {
  // Subscriber + Node intialize
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(encoder_1);
  nh.advertise(encoder_2);

  // Motor 1 initialize
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);

  // Motor 2 initialize
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);

  // Motor 1 and 2 encoders initialize
  EncoderInit1();
  EncoderInit2();


  // Motor 1 speed and position PID control initialize
  myPIDPos1.SetMode(AUTOMATIC);
  myPIDPos1.SetSampleTime(50);
  
  myPIDVel1.SetMode(AUTOMATIC);
  myPIDVel1.SetSampleTime(50);

  // Motor 2 speed and position PID control initialize
  myPIDPos2.SetMode(AUTOMATIC);
  myPIDPos2.SetSampleTime(50);
  
  myPIDVel2.SetMode(AUTOMATIC);
  myPIDVel2.SetSampleTime(50);
  
}

void loop() {

  // Motor 1 position PID control
  pos1Abs = abs(posSet1); // Absolute value of setpoint position
  result1=myPIDPos1.Compute();
  
  if(result1){

    if(velSet1>velMax){
      velSet1 = velMax; // Max speed limit
    }
    
  }

  
  // Motor 2 position PID control
  pos2Abs = abs(posSet2); // Absolute value of setpoint position
  result2=myPIDPos2.Compute();
  
  if(result2){

    if(velSet2>velMax){
      velSet2 = velMax; // Max speed limit
    }
    
  }
  
  // Motor 1 speed PID control
  result3=myPIDVel1.Compute();
  move_motor1(motor1Sign); // Rotate motor with specified direction (cw/ccw)
  abs_duration1 = abs(duration1); // Current speed
  
  if(result3)
  {
    abs_duration2 = abs(duration2); // Absolute value of distance moved
    duration1 = 0; // Clear speed measurement, update with the next measurement
    }

  // Motor 2 speed PID control
  result4=myPIDVel2.Compute();
  move_motor2(motor2Sign); // Rotate motor with specified direction (cw/ccw)
  abs_duration3 = abs(duration3); // Current speed
  
  if(result4)
  {
    abs_duration4 = abs(duration4); // Absolute value of distance moved
    duration3 = 0; // Clear speed measurement, update with the next measurement
    
    }
    
  encoder1.linear.x = abs_duration2; // Motor 1's angular displacement
  encoder1.angular.x = abs_duration1; // Motor 1's angular speed
  encoder2.linear.x = abs_duration4; // Motor 2's angular displacement
  encoder2.angular.x = abs_duration3; // Motor 2's angular speed

  // Publish encoder data
  encoder_1.publish(&encoder1);
  encoder_2.publish(&encoder2);

  nh.spinOnce();
  delay(50);

}

void EncoderInit1()
{
  Direction1 = true;// default: true -> Forward
  pinMode(encoder0pinB,INPUT);
  attachInterrupt(0, wheelSpeed1, CHANGE);
}

void EncoderInit2()
{
  Direction2 = true;// default: true -> Forward
  pinMode(encoder1pinB,INPUT);
  attachInterrupt(1, wheelSpeed2, CHANGE);
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
    duration1--, pulse1--;
  }
  else{
    duration1++, pulse1--;
  }
  duration2 = pulse1/1920.0; // Angular displacement in encoder 1
}


void wheelSpeed2()
{
  int Lstate2 = digitalRead(encoder1pinA);
  if((encoder1PinALast == LOW) && Lstate2==HIGH)
  {
    int val2 = digitalRead(encoder1pinB);
    if(val2 == LOW && Direction1)
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
    duration3--, pulse2--;
  }
  else{
    duration3++, pulse2++;
  }
  duration4 = pulse2/1920.0; // Angular displacement in encoder 2
}

void move_motor1(int sign1)
{
    // Saturation: Max PWM = 250
    if (PWM1>250){
      PWM1 = 250;
    }

    if(velSet1 == 0){
      PWM1 = 0; // No power input when steady state error = 0
    }

     // PWM + Direction
    if(sign1==1) // 1 -> cw
    {
     analogWrite(ENA,PWM1);
     digitalWrite(IN1,HIGH);
     digitalWrite(IN2,LOW);
    }

    else if(sign1==0) // 0 -> ccw
    {
     analogWrite(ENA,PWM1);
     digitalWrite(IN1,LOW);
     digitalWrite(IN2,HIGH);
    }
    
}

void move_motor2(int sign2)
{
    // Saturation: Max PWM = 250
    if (PWM2>250){
      PWM2 = 250;
    }

    if(velSet2 == 0){
      PWM2 = 0; // No power input when steady state error = 0
    }
    
     // PWM + Direction
     if(sign2==0) // 0 -> cw
     {
      analogWrite(ENB,PWM2);
      digitalWrite(IN3,HIGH);
      digitalWrite(IN4,LOW);
     }

     else if(sign2==1) // 1 -> ccw
     {
      analogWrite(ENB,PWM2);
      digitalWrite(IN3,LOW);
      digitalWrite(IN4,HIGH);
     }

}

void rotateMotor(double x){
  // Provide position setpoint
  posSet1 += x;
  posSet2 += x;
  
}
