#include <PID_v1.h> // PID library
#include <ros.h> // ROS serial
#include <geometry_msgs/Point.h>

// Robot specs
double r = 0.035; // Wheel radius (m)
double l = 0.11 ; // Robot track width (m)

double velSet1 = 0; // Velocity set point for motor 1 [RPM]
double velSet2 = 0; // Velocity set point for motor 2 [RPM]


// Encoders
const byte encoder0pinA = 2;// A pin for motor 1
const byte encoder0pinB = 4;// B pin for motor 1

const byte encoder1pinA = 3;// A pin for motor 2
const byte encoder1pinB = 5;// B pin for motor 2

byte encoder0PinALast; // Last encoder reading for motor 1
byte encoder1PinALast; // Last encoder reading for motor 2

double duration1; // Speed for the encoder in motor 1 [RPM]
double duration2=0; // Displacement in motor 1's angular position [rev]

double duration3; // Speed for the encoder in motor 2 [RPM]
double duration4=0; // Displacement in motor 2's angular position [rev]

double abs_duration1; // Speed for motor 1 [RPM]
double abs_duration2 =0; // Displacement for motor 1 [rev]

double abs_duration3; // Speed for motor 2 [RPM]
double abs_duration4 =0; // Displacement for motor 2 [rev]

double PWM1; // PWM signals for motor 1
double PWM2; // PWM signals for motor 2

double velMax = 90; // Max velocity for motor 1 and 2 [RPM]

double posSet1=0; // Position set point for motor 1 [rev]
double posSet2=0; // Position set point for motor 2 [rev]

double pos1Abs;
double pos2Abs;

boolean result1, result2, result3, result4;

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

unsigned long timer1, timer2;
double range_timer1, range_timer2 = 0;

// Callback function that gives velocity and position input;
void subCallback(const geometry_msgs::Point& cmd_pos){
  posSet1 = cmd_pos.x;
  posSet2 = cmd_pos.x;
}

ros::NodeHandle nh;
ros::Subscriber <geometry_msgs::Point> sub("/cmd_pos", subCallback);

double Kp1=1, Ki1=2, Kd1=0; // PID gains for speed 
double Kp2=28, Ki2=25, Kd2=5; // PID gains for position
PID myPIDVel1(&abs_duration1, &PWM1, &velSet1, Kp1, Ki1, Kd1, DIRECT); // PID setup for the speed of motor 1
PID myPIDVel2(&abs_duration3, &PWM2, &velSet2, Kp1, Ki1, Kd1, DIRECT); // PID setup for the speed of motor 2
PID myPIDPos1(&abs_duration2, &velSet1, &pos1Abs, Kp2, Ki2, Kd2, DIRECT); // PID setup for the position of motor 1
PID myPIDPos2(&abs_duration4, &velSet2, &pos2Abs, Kp2, Ki2, Kd2, DIRECT); // PID setup for the position of motor 1

void setup() {
  // put your setup code here, to run once:
  Serial.begin(57600);
  
  nh.initNode();
  nh.subscribe(sub);

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

  pos1Abs = abs(posSet1);
  pos2Abs = abs(posSet2);

  abs_duration1 = abs(duration1);
  abs_duration2 = abs(duration2);
  abs_duration3 = abs(duration3);
  abs_duration4 = abs(duration4);

  // Motor 1 speed PID control

  result1=myPIDPos1.Compute();
  
  if(result1){

    if(velSet1>velMax){
      velSet1 = velMax;
    }


    
  }

  // Motor 1 position PID control
  
  result3=myPIDVel1.Compute();
  move_motor1(posSet1);
  
  if(result3)
  {
    timer1 = millis();
    duration2 +=duration1*(timer1-range_timer1)*0.001/60;
    duration1 = 0; //Count clear, wait for the next count

    }
  range_timer1 = millis();

  // Motor 2 speed PID control

  result2=myPIDPos2.Compute();
  
  if(result2){

    if(velSet2>velMax){
      velSet2 = velMax;
    }


    
  }

  // Motor 2 speed PID control
  
  result4=myPIDVel2.Compute();
  move_motor2(posSet2);
  
  if(result4)
  {
    timer2 = millis();
    duration4 +=duration3*(timer2-range_timer2)*0.001/60;
    duration3 = 0; //Count clear, wait for the next count
    }


  range_timer2 = millis();


  nh.spinOnce();

  Serial.println(posSet1);

  
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
    duration3--;
  }
  else{
    duration3++;
  }
}

void move_motor1(double pos1)
{
    // Saturation: Max PWM = 255 and Min PWM = -255
    if (PWM1>250){
      PWM1 = 250;
    }



     // PWM + Direction
    if(posSet1>0){
     analogWrite(ENA,PWM1);
     digitalWrite(IN1,HIGH);
     digitalWrite(IN2,LOW);
    }

    else if(posSet1<0){
     analogWrite(ENA,PWM1);
     digitalWrite(IN1,LOW);
     digitalWrite(IN2,HIGH);
    }
    
}

void move_motor2(double pos2)
{
    // Saturation: Max PWM = 255 and Min PWM = -255
    if (PWM2>250){
      PWM2 = 250;
    }


     // PWM + Direction

     if(posSet2>0){
      analogWrite(ENB,PWM2);
      digitalWrite(IN3,HIGH);
      digitalWrite(IN4,LOW);
     }

     else if(posSet2<0){
      analogWrite(ENB,PWM2);
      digitalWrite(IN3,LOW);
      digitalWrite(IN4,HIGH);
     }

}

void goStraight(double x, double pos1, double pos2){
  pos1 = x;
  pos2 = x;
}

void rotate(double x){
  posSet1 = -(0.035/0.11)*x/6.28;
  posSet2 = (0.035/0.11)*x/6.28;
}

