// Libraries

#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

#include "tf/tf.h"
#include "tf/tfMessage.h"


// ROS comm

ros::NodeHandle nh;
geometry_msgs::Twist msg;
nav_msgs::Odometry odom;

tf::TransformBroadcaster odom_broadcaster;

ros::Publisher odom_pub("odom", &odom);

ros::Time current_time, last_time;

double poseX = 0.0;
double poseY = 0.0;
double poseTanH = 0.0;


void ros_receiver(const geometry_msgs::Twist& cmd_vel) {
  float ros_linear = cmd_vel.linear.poseX;
  float ros_ang = cmd_vel.angular.z;

  robotVelocity2joystick(ros_linear, ros_ang);
}

ros::Subscriber<geometry_msgs::Twist> sub("/cmd_vel", ros_receiver);

void ros_sender(robot theta) {
  double vth = theta.velAngular;
  double vx = theta.velLinear;

  current_time = nh.now();

  // //compute odometry in a typical way given the velocities of the robot
  double dt = (current_time - last_time).toSec();
  double delta_x = (vx * cos(poseTanH)) * dt;
  double delta_y = (vx * sin(poseTanH)) * dt;
  double delta_th = vth * dt;

  poseX += delta_x;
  poseY += delta_y;
  poseTanH += delta_th;

  // //since all odometry is 6DOF we'll need a quaternion created from yaw
  geometry_msgs::Quaternion odom_quat = tf::createQuaternionFromYaw(poseTanH);

  // //first, we'll publish the transform over tf
  geometry_msgs::TransformStamped odom_trans;
  odom_trans.header.stamp = current_time;
  odom_trans.header.frame_id = "odom";
  // odom_trans.child_frame_id = "base_link";

  odom_trans.transform.translation.poseX = poseX;
  odom_trans.transform.translation.poseY = poseY;
  odom_trans.transform.translation.z = 0.0;
  odom_trans.transform.rotation = odom_quat;

  // //send the transform
  // odom_broadcaster.sendTransform(odom_trans);

  //next, we'll publish the odometry message over ROS
  odom.header.stamp = current_time;
  odom.header.frame_id = "odom";

  //set the position
  odom.pose.pose.position.poseX = poseX;
  odom.pose.pose.position.poseY = poseY;
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation = odom_quat;

  //set the velocity
  odom.child_frame_id = "base_link";
  odom.twist.twist.linear.poseX = vx;
  odom.twist.twist.angular.z = vth;

  //publish the message
  odom_pub.publish(&odom);
  // nh.spinOnce();

  last_time = current_time;
}


// Variables to get speed and write on joystick

int stoppedTime = 500;      // 1s = 1000
float radiusWheel = 0.165;  // in meters
float trackWidth = 0.5;
static float alpha = 0.01;               //0.0005     // for filtering
const unsigned long interval2pub = 100;  // 100 = 0.1s = 100Hz
unsigned long previousMillis2pub;

int Xchannel = 25;
int Ychannel = 26;
int X_joy;
int Y_joy;

struct wheel {
  float velLinear;
};

struct robot {
  float velLinear;
  float velAngular;
};

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


// interruptions for hall effect sensor

void IRAM_ATTR rightMotorISR(Hall* hall) {

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


void setup() {
  Serial.begin(115200);

  //ros setup
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(odom_pub);
  current_time = nh.now();
  last_time = nh.now();

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
}

void loop() {
  wheel leftWheel = speedLeftWheel(&ALeftHall, &BLeftHall);
  float leftWheel_filtered = filterLeft(leftWheel.velLinear);

  wheel rightWheel = speedRightWheel(&ARightHall, &BRightHall);
  float rightWheel_filtered = filterRight(rightWheel.velLinear);

  robot theta = wheelsVelocity2robotVelocity(leftWheel_filtered, rightWheel_filtered);

  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis2pub >= interval2pub) {
    previousMillis2pub = currentMillis;
    ros_sender(theta);
  }

  nh.spinOnce();
}


robot wheelsVelocity2robotVelocity(float leftWheel_velLinear, float rightWheel_velLinear) {
  // https://www.roboticsbook.org/S52_diffdrive_actions.html

  robot localRobot;

  localRobot.velLinear = (rightWheel_velLinear + leftWheel_velLinear) / 2;
  localRobot.velAngular = (rightWheel_velLinear - leftWheel_velLinear) / trackWidth;
  // velAngular is negative clockwise on ROS

  return localRobot;
}

void robotVelocity2joystick(float velLinear, float velAngular) {
  float velLinearMAX = 0.6;   // (m/s) going forward
  float velLinearMIN = -0.7;  // (m/s) going reverse

  float velAngularMAX = 2.3;  // (rad/s) 1 rad = 60°
  float velAngularMIN = -1.7;

  velLinear = constrain(velLinear, velLinearMIN, velLinearMAX);
  velAngular = constrain(velAngular, velAngularMIN, velAngularMAX);

  Y_joy = 255 * (velLinear - velLinearMIN) / (velLinearMAX - velLinearMIN);
  X_joy = 255 * (velAngular - velAngularMIN) / (velAngularMAX - velAngularMIN);

  dacWrite(Xchannel, X_joy);
  dacWrite(Ychannel, Y_joy);
}


wheel speedLeftWheel(Hall* Ahall, Hall* Bhall) {
  static wheel localWheel;
  Ahall->currentTime = xTaskGetTickCount();

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
  Ahall->currentTime = xTaskGetTickCount();

  if (Ahall->currentTime - Ahall->previousTime >= stoppedTime) {

    Ahall->endPulse = true;
    Ahall->timeStart = 0;
    Bhall->timeEnd = 0;
    Ahall->timeEnd = 0;
    Ahall->previousTime = Ahall->currentTime;

    localWheel.velLinear = 0;
  }

  if (Ahall->timeStart && Ahall->endPulse) {
    int deltaTime = (Ahall->timeEnd - Ahall->timeStart);
    float freq = 1 / (deltaTime / 1000.0);
    float rpmWheel = freq * (60 / 32.0);
    float angularFrequency = ((rpmWheel * (2 * PI / 60.0)));
    localWheel.velLinear = angularFrequency * radiusWheel;

    if ((Ahall->timeEnd - Bhall->timeEnd) > (Bhall->timeEnd - Ahall->timeStart)) {
      localWheel.velLinear = localWheel.velLinear * (-1);
    }

    Ahall->timeStart = 0;
    Bhall->timeEnd = 0;
    Ahall->timeEnd = 0;
    Ahall->previousTime = Ahall->currentTime;
  }

  return localWheel;
}

float filterLeft(float speed_measured) {
  // iir filter aka EMA filter
  //https://blog.stratifylabs.dev/device/2013-10-04-An-Easy-to-Use-Digital-Filter/
  // low number for a low pass filter
  static float filteredValue;

  if (speed_measured > 1.0 || speed_measured < -1.0) {
    return filteredValue;
  }

  if (filteredValue == 0.0) {
    filteredValue = speed_measured;
  } else {
    filteredValue = alpha * speed_measured + (1 - alpha) * filteredValue;
  }
  constrain(filteredValue, velLinearMIN, velLinearMAX);
  return filteredValue;
}

float filterRight(float speed_measured) {
  static float filteredValue;

  if (speed_measured > 1.0 || speed_measured < -1.0) {
    return filteredValue;
  }

  if (filteredValue == 0.0) {
    filteredValue = speed_measured;
  } else {
    filteredValue = alpha * speed_measured + (1 - alpha) * filteredValue;
  }
  return filteredValue;
}