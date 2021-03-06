// Diffbot enhancement for Teensy board
// Uses L298 motor controller or Ardumoto
// Motor scale for needed input -99 to 99, centered at 0 (stop)
// Callback routine multiplies number by 2.55 for L298 range
// Encoder ticks published

#include <ros.h>
#include <std_msgs/Int32.h>
#include <diffbot_msgs/Encoder.h>
#include <std_msgs/Empty.h>

#define FORWARD 0
#define REVERSE 1
int x;
int leftpwr;
int rightpwr;

// Motor definitions to make life easier:
#define MOTOR_A 1
#define MOTOR_B 0

// Pin Assignments //
#define DIRA 2 // Direction control1 for motor A (L298 IN1)
#define PWMA 3 // PWM control (speed) for motor A (L298 ENA)
#define DIRC 4 // Direction control2 for motor A (L298 IN2)
#define DIRB 5 // Direction control1 for motor B (L298 IN3)
#define PWMB 6 // PWM control (speed) for motor B (L298 ENB)
#define DIRD 7 // Direction control2 for motor B (L298 IN4)

// Encoder setup
#define ENCODER_OPTIMIZE_INTERRUPTS //Only for Teensy
#include <Encoder.h>
Encoder encoderLeft(8, 9); // MOTOR A encoder data
Encoder encoderRight(10, 11);// MOTOR B encoder data

ros::NodeHandle nh;

// Callbacks
void left_motorCb(const std_msgs::Int32& msg)
{
  //nh.loginfo("I heard new left input");
  
  leftpwr=msg.data;
    if (leftpwr > 0) {
    leftpwr = int(2.55*(leftpwr));
    //nh.loginfo("LEFT FORWARD");
    driveArdumoto(MOTOR_A, FORWARD, leftpwr);
  }
    else if (leftpwr < 0) {
    leftpwr = -int(2.55*(leftpwr));
    //nh.loginfo("LEFT REVERSE");
    driveArdumoto(MOTOR_A, REVERSE, leftpwr);
  }
    else if (leftpwr == 0) {
    //nh.loginfo("LEFT STOP");
    stopArdumoto(MOTOR_A);
  }
}
void resetCb( const std_msgs::Empty& reset)
{
  encoderLeft.write(0);
  encoderRight.write(0);
  nh.loginfo("Reset both wheel encoders to zero");
}
void right_motorCb(const std_msgs::Int32& msg)
{
  //nh.loginfo("I heard new right input");
  
  rightpwr=msg.data;
    if (rightpwr > 0) {
    rightpwr = int(2.55*(rightpwr));
    //nh.loginfo("RIGHT FORWARD");
    driveArdumoto(MOTOR_B, FORWARD, rightpwr);
  }
    else if (rightpwr < 0) {
    rightpwr = -int(2.55*(rightpwr));
    //nh.loginfo("RIGHT REVERSE");
    driveArdumoto(MOTOR_B, REVERSE, rightpwr);
  }
    else if (rightpwr == 0) {
    //nh.loginfo("RIGHT STOP");
    stopArdumoto(MOTOR_B);
  }
}

// ROS Publisher setup to publish left and right encoder ticks
// This uses the custom encoder ticks message that defines an array of two integers
diffbot_msgs::Encoder ticks;
ros::Publisher pub_encoders("encoder_ticks", &ticks);

ros::Subscriber<std_msgs::Int32> sub("motor_left", &left_motorCb);
ros::Subscriber<std_msgs::Empty> sub_reset("reset", resetCb);
ros::Subscriber<std_msgs::Int32> sub_2("motor_right", &right_motorCb);
 
void setup()
{
  pinMode(13, OUTPUT);
  nh.initNode();
  nh.subscribe(sub);
  nh.subscribe(sub_reset);
  nh.subscribe(sub_2);
  nh.loginfo("DiffBot Wheel Encoders:");
  nh.advertise(pub_encoders);
  
  pinMode(PWMA, OUTPUT);
  pinMode(DIRA, OUTPUT);
  pinMode(DIRC, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(DIRB, OUTPUT);
  pinMode(DIRD, OUTPUT);
  
  digitalWrite(PWMA, LOW);
  digitalWrite(DIRA, LOW);
  digitalWrite(DIRC, LOW);
  digitalWrite(PWMB, LOW);
  digitalWrite(DIRB, LOW);
  digitalWrite(DIRD, LOW);
}
 
long positionLeft  = -999;
long positionRight = -999; 
 
void loop()
{
  long newLeft, newRight;
  newLeft = encoderLeft.read();
  newRight = encoderRight.read();

  ticks.encoders[0] = newLeft;
  ticks.encoders[1] = newRight;
  pub_encoders.publish(&ticks);
  nh.spinOnce();
  
  delay(5);
  
  //nh.loginfo("In main loop now.");
  if (newLeft != positionLeft || newRight != positionRight) {
    //String str = "Left = " + String(newLeft) + ", Right = " + String(newRight);
    //nh.loginfo(str);
    positionLeft = newLeft;
    positionRight = newRight;  
    }
}
// Power motors both directions
void driveArdumoto(byte motor, byte dir, byte spd)
{
  if (dir == 1) {
    x = 0;
  }
  else
    x = 1;
  if (spd > 255) {
  spd = 255;
  }
  else if (spd < 0) {
  spd = 0;
  }

    if(motor == MOTOR_A) {
    digitalWrite(DIRA, dir);
    analogWrite(PWMA, spd);
    digitalWrite(DIRC, x);
  }
    else if (motor == MOTOR_B) {
    digitalWrite(DIRB, dir);
    analogWrite(PWMB, spd);
    digitalWrite(DIRD, x);
  }
}

// stopArdumoto makes ONE motor stop
void stopArdumoto(byte motor)
{
  driveArdumoto(motor, 0, 0);
}

