/*
  Theta Locomotion Control Algorithm (ESP)
  Developed by Felipe Machado and Mateus Santos da Silva
  Last modification: 08/06/2023
  Version: 0.3
*/
// Joystick control
int Xchannel = 25;
int Ychannel = 26;
int Xvalue = 130;
int Yvalue = 130;
// Speed settings



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


// Variables - Change radius and stoppedTime accordingly
int deltaTime;
int direction;  // char*
float velLinear;
float freq, rpmWheel, velAng;
float radiusWheel = 0.165;  // in meters
int stoppedTime = 1000000;  // 1s = 1 000 000


struct wheel {
  int direction;  // 0 = stop; 1 = reverse; 2 = forward;
  float velocity;
};


// portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
void IRAM_ATTR rightMotorISR(Hall* hall) {  // IRAM_ATTR to run on RAM
  // portENTER_CRITICAL(&mux);
  ARightHall.timeStart = BRightHall.timeEnd;
  BRightHall.timeEnd = ARightHall.timeEnd;
  ARightHall.timeEnd = xTaskGetTickCountFromISR();  // 1 tick = 1ms by default
  hall->endPulse = !hall->endPulse;
  // portEXIT_CRITICAL(&mux);
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

  // Serial.println();
  Serial.print("RIGHT  = ");
  wheel leftWheel = speedRightWheel(&ALeftHall, &BLeftHall);
  // wheel leftWheel = speedRightWheel(&ARightHall, &BRightHall);
  Serial.print(leftWheel.direction);
  Serial.print("  ");
  Serial.print(leftWheel.velocity);
  Serial.println(" m/s");


  // Serial.print("                 RIGHT = ");
  // rightWheel = speedRightWheel(&ARightHall, &BRightHall);
  // Serial.print(rightWheel.direction);
  // Serial.print("  ");
  // Serial.print(rightWheel.velocity);
  // Serial.println(" m/s");
  // Serial.println();
  // Serial.println("- - - - - - - - - - - - - - - - - - - - - - - - - - - -");
}

wheel speedRightWheel(Hall* Ahall, Hall* Bhall) {
  static wheel localWheel;
  Ahall->currentTime = micros();
  // If the wheels have stopped for stoppedTime, reset hall values
  if (Ahall->currentTime - Ahall->previousTime >= stoppedTime) {
    Ahall->endPulse = true;
    Ahall->timeStart = 0;
    Bhall->timeEnd = 0;
    Ahall->timeEnd = 0;
    Ahall->previousTime = Ahall->currentTime;
    localWheel.direction = 0;  //SToP
    localWheel.velocity = 0;
  }
  // If there is a turn complete from A Hall sensor
  if (Ahall->timeStart && Ahall->endPulse) {
    // Determine the direction of wheel rotation based on the readings of both Hall sensors
    if ((Ahall->timeEnd - Bhall->timeEnd) > (Bhall->timeEnd - Ahall->timeStart)) {
      localWheel.direction = 1;  // ReVerSe
    } else {
      localWheel.direction = 2;  // ForWarD
    }
    deltaTime = (Ahall->timeEnd - Ahall->timeStart);  // in miliseconds
    freq = 1 / (deltaTime / 1000.0);                  // divide by float to save whole number
    rpmWheel = freq * (60 / 32.0);
    velAng = ((rpmWheel * (2 * PI / 60.0)));
    localWheel.velocity = velAng * radiusWheel;  // m/s
    // Reset the Hall sensor values and update the previousTime variable
    Ahall->timeStart = 0;
    Bhall->timeEnd = 0;
    Ahall->timeEnd = 0;
    Ahall->previousTime = Ahall->currentTime;
  }
  return localWheel;
}

