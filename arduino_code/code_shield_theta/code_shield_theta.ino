/*
  Theta Locomotion Control Algorithm (ESP)
  Developed by Felipe Machado and Mateus Santos da Silva
  Last modification: 25/08/2023
  Version: 0.4

usable to read speed from hall efect sensor,
control joystick by typing the bits values

TODO:
[ ] convert values XLinearROS and ZAngROS to XJoy and YJoy
[ ] do the feedback loop control to asset desired speed
*/


// control joystick
int Xchannel = 25;
int Ychannel = 26;
int X_joy = 105;
int Y_joy = 105;

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
int stoppedTime = 1000000;  // 1s = 1 000 000
float radiusWheel = 0.165;  // in meters


struct wheel {
  float velocity;
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


void setup() {
  Serial.begin(115200);

  pinMode(Xchannel, OUTPUT);
  pinMode(Ychannel, OUTPUT);

  pinMode(ARightHall.pin, INPUT_PULLUP);
  pinMode(BRightHall.pin, INPUT_PULLUP);
  pinMode(ALeftHall.pin, INPUT_PULLUP);
  pinMode(BLeftHall.pin, INPUT_PULLUP);

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
}

void loop() {

  struct wheel rightWheel;
  struct wheel leftWheel;

  leftWheel = speedLeftWheel(&ALeftHall, &BLeftHall);
  Serial.println(leftWheel.velocity);
  // rightWheel = speedRightWheel(&ARightHall, &BRightHall);
}


// void writeToJoystick() {
//   float X_ROS;
//   float Z_ROS;

//   if (Serial.available() > 0) {  // No Line Ending

//     X_ROS = Serial.parseFloat();
//     Z_ROS = Serial.parseFloat();
//     // joystickConverter(X_ROS, Z_ROS, X_joy, Y_joy);
//     Serial.println("------------------------------");

//     dacWrite(Xchannel, X_joy);
//     dacWrite(Ychannel, Y_joy);

//     Serial.println("Input:  X_ROS: " + String(X_ROS) + ", Z_ROS: " + String(Z_ROS));
//     Serial.println("Output: X_joy: " + String(X_joy) + ", Y_joy: " + String(Y_joy));
//   }
// }


wheel speedLeftWheel(Hall* Ahall, Hall* Bhall) {
  static wheel localWheel;
  Ahall->currentTime = micros();

  // If the wheels have stopped for stoppedTime, reset hall values
  if (Ahall->currentTime - Ahall->previousTime >= stoppedTime) {

    Ahall->endPulse = true;
    Ahall->timeStart = 0;
    Bhall->timeEnd = 0;
    Ahall->timeEnd = 0;
    Ahall->previousTime = Ahall->currentTime;

    localWheel.velocity = 0;
  }

  // If there is a turn complete from A Hall sensor
  if (Ahall->timeStart && Ahall->endPulse) {
    int deltaTime = (Ahall->timeEnd - Ahall->timeStart);  // in miliseconds
    float freq = 1 / (deltaTime / 1000.0);                  // divide by float to save whole number
    float rpmWheel = freq * (60 / 32.0);
    float velAngWheel = ((rpmWheel * (2 * PI / 60.0)));
    localWheel.velocity = velAngWheel * radiusWheel;       // m/s

    // Determine the direction of wheel rotation based on the readings of both Hall sensors
    if ((Ahall->timeEnd - Bhall->timeEnd) > (Bhall->timeEnd - Ahall->timeStart)) {
      localWheel.velocity = localWheel.velocity*(-1);  // ReVerSe
    } 

    // Reset the Hall sensor values and update the previousTime variable
    Ahall->timeStart = 0;
    Bhall->timeEnd = 0;
    Ahall->timeEnd = 0;
    Ahall->previousTime = Ahall->currentTime;
  }

  return localWheel;
}
