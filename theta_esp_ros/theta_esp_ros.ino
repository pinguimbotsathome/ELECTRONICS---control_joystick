/*
  Theta Locomotion Control Algorithm (ESP)
  Developed by Felipe Machado and Mateus Santos da Silva
  Last modification: 19/09/2023
  Version: 0.5

- usable to read speed from hall efect sensor (speedLeftWheel and speedRightWheel)
- control joystick by typing the bits values (writeJoystickManually)
- control joystick by PI controler (controle_vel_linear)

TODO:
[ ] refactor
*/

#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

#include "tf/tf.h"
#include "tf/tfMessage.h"

ros::NodeHandle nh;
geometry_msgs::Twist msg;
nav_msgs::Odometry odom;

tf::TransformBroadcaster odom_broadcaster;

ros::Publisher odom_pub("odom", &odom);

ros::Time current_time, last_time;

double x = 0.0;
double y = 0.0;
double th = 0.0;

struct Hall {
  int pin;
  volatile bool endPulse;
  volatile unsigned long timeStart;
  volatile unsigned long timeEnd;
  unsigned long currentTime;
  unsigned long previousTime;
};

Hall ARightHall = { 5, true, 0, 0, 0, 0 };   // white - orange
Hall BRightHall = { 18, true, 0, 0, 0, 0 };  // brown - blue
Hall ALeftHall = { 19, true, 0, 0, 0, 0 };   // gray - orange
Hall BLeftHall = { 21, true, 0, 0, 0, 0 };   // blue - blue


// variables - change radius and stoppedTime accordingly

int stoppedTime = 500;      // 1s = 1 000
float radiusWheel = 0.165;  // in meters


struct wheel {
  float velLinear;
};

int Xchannel = 25;
int Ychannel = 26;
int X_joy;  // = 105;
int Y_joy;  // = 105;


struct robot {
  float trackWidth = 0.5;
  float velLinear;
  float velAngular;
};


void IRAM_ATTR rightMotorISR(Hall* hall) {  // IRAM_ATTR to run on RAM

  ARightHall.timeStart = BRightHall.timeEnd;
  BRightHall.timeEnd = ARightHall.timeEnd;
  ARightHall.timeEnd = xTaskGetTickCountFromISR();  // 1 tick = 1ms by default

  hall->endPulse = !hall->endPulse;
}

void IRAM_ATTR leftMotorISR(Hall* hall) {

  ALeftHall.timeStart = BLeftHall.timeEnd;
  BLeftHall.timeEnd = ALeftHall.timeEnd;
  ALeftHall.timeEnd = xTaskGetTickCountFromISR();

  hall->endPulse = !hall->endPulse;
}

void ros_receiver(const geometry_msgs::Twist &cmd_vel){
  float ros_linear = cmd_vel.linear.x;
  float ros_ang = cmd_vel.angular.z;
  robotVelocity2joystick(ros_linear, ros_ang);

}

ros::Subscriber <geometry_msgs::Twist> sub("/cmd_vel",ros_receiver);


void loop() {

  // writeJoystickManually();
  // Serial.println("- - - - - - - - - - - - - - - - - - - - - - - - - - - -");


  wheel leftWheel = speedLeftWheel(&ALeftHall, &BLeftHall);
  float leftWheel_filtered = filterLeft(leftWheel.velLinear);
  // Serial.print("LEFT = " + String(leftWheel_filtered));

  wheel rightWheel = speedRightWheel(&ARightHall, &BRightHall);
  float rightWheel_filtered = filterRight(rightWheel.velLinear);
  // Serial.println("  RIGHT = " + String(rightWheel_filtered));

  robot theta = wheelsVelocity2robotVelocity(leftWheel_filtered, rightWheel_filtered);
  // Serial.print("vLIN = " + String(theta.velLinear));
  // Serial.println("    vANG = " + String(theta.velAngular));
  // Serial.println("- - - - - - - - - - - - - - - - - - - - - - - - - - - -");

  
  // robotVelocity2joystick(contrVelLin, contrVelAng);

  nh.spinOnce();
  // writeJoystickManually();
  // Serial.println("- - - - - - - - - - - - - - - - - - - - - - - - - - - -");
}



void ros_sender(robot theta)
{


  double vth = theta.velAngular;
  double vx = theta.velLinear;
  // double x = 0.0;
  // double y = 0.0;
  // double th = 0.0;

  // double vx = 0.1;
  // double vy = -0.1;
  // double vth = 0.1;

  // ros::Time current_time, last_time;
  // current_time = ros::Time::now();
  // last_time = ros::Time::now();
 
  current_time = ros::Time::now();

  //compute odometry in a typical way given the velocities of the robot
  double dt = (current_time - last_time).toSec();
  double delta_x = (vx * cos(th)) * dt;
  double delta_y = (vx * sin(th)) * dt;
  double delta_th = vth * dt;

  x += delta_x;
  y += delta_y;
  th += delta_th;

  //since all odometry is 6DOF we'll need a quaternion created from yaw
  geometry_msgs::Quaternion odom_quat = tf::createQuaternionFromYaw(th);

  //first, we'll publish the transform over tf
  geometry_msgs::TransformStamped odom_trans;
  odom_trans.header.stamp = current_time;
  odom_trans.header.frame_id = "odom";
  odom_trans.child_frame_id = "base_link";

  odom_trans.transform.translation.x = x;
  odom_trans.transform.translation.y = y;
  odom_trans.transform.translation.z = 0.0;
  odom_trans.transform.rotation = odom_quat;

  //send the transform
  odom_broadcaster.sendTransform(odom_trans);

  //next, we'll publish the odometry message over ROS
  odom.header.stamp = current_time;
  odom.header.frame_id = "odom";

  //set the position
  odom.pose.pose.position.x = x;
  odom.pose.pose.position.y = y;
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation = odom_quat;

  //set the velocity
  odom.child_frame_id = "base_link";
  odom.twist.twist.linear.x = vx;
  odom.twist.twist.angular.z = vth;

  //publish the message
  odom_pub.publish(&odom);

  last_time = current_time;

  
}


robot wheelsVelocity2robotVelocity(float leftWheel_velLinear, float rightWheel_velLinear) {
  // https://www.roboticsbook.org/S52_diffdrive_actions.html

  robot localRobot;

  localRobot.velLinear = (rightWheel_velLinear + leftWheel_velLinear) / 2;
  localRobot.velAngular = (rightWheel_velLinear - leftWheel_velLinear) / localRobot.trackWidth;
  // velAngular is negative clockwise on ROS

  return localRobot;
}


void robotVelocity2joystick(float velLinear, float velAngular) {
  float velLinearMAX = 0.6;   // (m/s) going forward
  float velLinearMIN = -0.7;  // (m/s) going reverse

  float velAngularMAX = 2.3;  // (rad/s) 1 rad = 60°
  float velAngularMIN = -1.7;

  Y_joy = 255 * (velLinear - velLinearMIN) / (velLinearMAX - velLinearMIN);
  X_joy = 255 * (velAngular - velAngularMIN) / (velAngularMAX - velAngularMIN);

  dacWrite(Xchannel, X_joy);
  dacWrite(Ychannel, Y_joy);

  // Serial.print("X_joy: " + String(X_joy));
  // Serial.println("      Y_joy: " + String(Y_joy));
}


wheel speedLeftWheel(Hall* Ahall, Hall* Bhall) {
  static wheel localWheel;
  Ahall->currentTime = xTaskGetTickCount();  // micros();

  // If the wheels have stopped for stoppedTime, reset hall values
  if (Ahall->currentTime - Ahall->previousTime >= stoppedTime) {

    Ahall->endPulse = true;
    Ahall->timeStart = 0;
    Bhall->timeEnd = 0;
    Ahall->timeEnd = 0;
    Ahall->previousTime = Ahall->currentTime;

    localWheel.velLinear = 0;
  }

  // If there is a complete wheel turn from A Hall sensor
  if (Ahall->timeStart && Ahall->endPulse) {
    int deltaTime = (Ahall->timeEnd - Ahall->timeStart);  // in miliseconds
    float freq = 1 / (deltaTime / 1000.0);                // divide by float to save whole number
    float rpmWheel = freq * (60 / 32.0);
    float angularFrequency = ((rpmWheel * (2 * PI / 60.0)));  // (rad/s)
    localWheel.velLinear = angularFrequency * radiusWheel;    // (m/s)

    if ((Ahall->timeEnd - Bhall->timeEnd) > (Bhall->timeEnd - Ahall->timeStart)) {
      localWheel.velLinear = localWheel.velLinear * (-1);
    }

    // Reset the Hall sensor values and update the previousTime variable
    Ahall->timeStart = 0;
    Bhall->timeEnd = 0;
    Ahall->timeEnd = 0;
    Ahall->previousTime = Ahall->currentTime;
  }
  return localWheel;
}

wheel speedRightWheel(Hall* Ahall, Hall* Bhall) {
  static wheel localWheel;
  Ahall->currentTime = xTaskGetTickCount();  // micros();

  // If the wheels have stopped for stoppedTime, reset hall values
  if (Ahall->currentTime - Ahall->previousTime >= stoppedTime) {

    Ahall->endPulse = true;
    Ahall->timeStart = 0;
    Bhall->timeEnd = 0;
    Ahall->timeEnd = 0;
    Ahall->previousTime = Ahall->currentTime;

    localWheel.velLinear = 0;
  }

  // If there is a complete wheel turn from A Hall sensor
  if (Ahall->timeStart && Ahall->endPulse) {
    int deltaTime = (Ahall->timeEnd - Ahall->timeStart);  // in miliseconds
    float freq = 1 / (deltaTime / 1000.0);                // divide by float to save whole number
    float rpmWheel = freq * (60 / 32.0);
    float angularFrequency = ((rpmWheel * (2 * PI / 60.0)));  // (rad/s)
    localWheel.velLinear = angularFrequency * radiusWheel;    // (m/s)

    if ((Ahall->timeEnd - Bhall->timeEnd) > (Bhall->timeEnd - Ahall->timeStart)) {
      localWheel.velLinear = localWheel.velLinear * (-1);
    }

    // Reset the Hall sensor values and update the previousTime variable
    Ahall->timeStart = 0;
    Bhall->timeEnd = 0;
    Ahall->timeEnd = 0;
    Ahall->previousTime = Ahall->currentTime;
  }

  return localWheel;
}


float filterLeft(float speed_measured) {  // iir filter aka EMA filter
                                          //https://blog.stratifylabs.dev/device/2013-10-04-An-Easy-to-Use-Digital-Filter/
  static float alpha = 0.001;             // low number for a low pass filter
  static float filteredValue = 0.0;

  if (filteredValue == 0.0) {
    filteredValue = speed_measured;
  } else {
    filteredValue = alpha * speed_measured + (1 - alpha) * filteredValue;
  }
  return filteredValue;
}


float filterRight(float speed_measured) {
  static float alpha = 0.001;
  static float filteredValue = 0.0;

  if (filteredValue == 0.0) {
    filteredValue = speed_measured;
  } else {
    filteredValue = alpha * speed_measured + (1 - alpha) * filteredValue;
  }
  return filteredValue;
}

void setup() {
  Serial.begin(115200);

  //ros setup:
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(odom_pub);

  pinMode(Xchannel, OUTPUT);
  pinMode(Ychannel, OUTPUT);

  pinMode(ARightHall.pin, INPUT);
  pinMode(BRightHall.pin, INPUT);
  pinMode(ALeftHall.pin, INPUT);
  pinMode(BLeftHall.pin, INPUT);

  attachInterrupt(
    digitalPinToInterrupt(ARightHall.pin), [] {
      rightMotorISR(&ARightHall);
    },
    FALLING);
  attachInterrupt(
    digitalPinToInterrupt(BRightHall.pin), [] {
      rightMotorISR(&BRightHall);
    },
    FALLING);

  attachInterrupt(
    digitalPinToInterrupt(ALeftHall.pin), [] {
      leftMotorISR(&ALeftHall);
    },
    FALLING);
  attachInterrupt(
    digitalPinToInterrupt(BLeftHall.pin), [] {
      leftMotorISR(&BLeftHall);
    },
    FALLING);

  dacWrite(Xchannel, 130);
  dacWrite(Ychannel, 130);

  current_time = nh.now();
  last_time = nh.now();
}

