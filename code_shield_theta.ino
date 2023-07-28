/*
Date: 28/07/2023
Version: 2.0
*/

// control joystick
int Xchannel = 25;
int Ychannel = 26;
int Xvalue = 130;
int Yvalue = 130;


struct Hall {
  int pin;
  volatile bool endPulse;
  volatile unsigned long timeStart;
  volatile unsigned long timeEnd;
  unsigned long currentTime;
  unsigned long previousTime;
};

int APinRight = 5;
int APinLeft = 19;
Hall ARightHall = { APinRight, true, 0, 0, 0, 0 };  // white - orange
Hall BRightHall = { 18, true, 0, 0, 0, 0 };         // brown - blue
Hall ALeftHall = { APinLeft, true, 0, 0, 0, 0 };    // gray - orange
Hall BLeftHall = { 21, true, 0, 0, 0, 0 };          // blue - blue


// variables - change radius and stoppedTime accordingly
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

  // portENTER_CRITICAL(&mux);
  ALeftHall.timeStart = BLeftHall.timeEnd;
  BLeftHall.timeEnd = ALeftHall.timeEnd;
  ALeftHall.timeEnd = xTaskGetTickCountFromISR();

  hall->endPulse = !hall->endPulse;
  // portEXIT_CRITICAL(&mux);
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
  writeToJoystick();
  /*
  struct wheel rightWheel;
  struct wheel leftWheel;


  Serial.println();
  Serial.print("LEFT  = ");
  leftWheel = speedLeftWheel(&ALeftHall, &BLeftHall);
  Serial.print(leftWheel.direction);
  Serial.print("  ");
  Serial.print(leftWheel.velocity);
  Serial.print(" km/h");

  Serial.print("                 RIGHT = ");
  rightWheel = speedRightWheel(&ARightHall, &BRightHall);
  Serial.print(rightWheel.direction);
  Serial.print("  ");
  Serial.print(rightWheel.velocity);
  Serial.println(" km/h");
  Serial.println();
  Serial.println("- - - - - - - - - - - - - - - - - - - - - - - - - - - -");
  */
}

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
    localWheel.velocity = velAng * radiusWheel * 3.6;  // 3.6 for km/h

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
  Ahall->currentTime = micros();  // change to xTaskGetTickCount ??????

  // Serial.print(currentTime);
  // Serial.print(" " + String(previousTime));
  // Serial.println("  " + String(stoppedTime));

  // If the wheels have stopped for stoppedTime, reset hall values
  if (Ahall->currentTime - Ahall->previousTime >= stoppedTime) {

    Ahall->endPulse = true;
    Ahall->timeStart = 0;
    Bhall->timeEnd = 0;
    Ahall->timeEnd = 0;
    Ahall->previousTime = Ahall->currentTime;

    localWheel.direction = 0;  //"STP - ";
    localWheel.velocity = 0;   // velLinear = 0;

    // return (2.0, 1.0);
    // Serial.println(direction + String(velLinear) + " km/h");
  }


  // If there is a turn complete from A Hall sensor
  if (Ahall->timeStart && Ahall->endPulse) {

    // Determine the direction of wheel rotation based on the readings of both Hall sensors
    if ((Ahall->timeEnd - Bhall->timeEnd) > (Bhall->timeEnd - Ahall->timeStart)) {
      localWheel.direction = 1;  // "RVS - ";
    } else {
      localWheel.direction = 2;  //"FWD - ";
    }

    deltaTime = (Ahall->timeEnd - Ahall->timeStart);  // in miliseconds
    freq = 1 / (deltaTime / 1000.0);
    rpmWheel = freq * (60 / 32.0);  // divide by float to save whole number
    velAng = ((rpmWheel * (2 * PI / 60.0)));
    localWheel.velocity = velAng * radiusWheel * 3.6;  // 3.6 for km/h

    // Serial.println(deltaTime);

    // Reset the Hall sensor values and update the previousTime variable
    Ahall->timeStart = 0;
    Bhall->timeEnd = 0;
    Ahall->timeEnd = 0;
    Ahall->previousTime = Ahall->currentTime;

    // return (2.0, 1.0);
    // Serial.println(direction + String(velLinear) + " km/h");

    // String return_string = (direction + String(velLinear) + " m/s");
  }
  // localVelDir.vel = velLinear;
  // localVelDir.dir = direction;
  return localWheel;
}

















void writeToJoystick() {

  // try this
  /*
  if(Serial.available()){
    Xvalue = Serial.readStringUntil('\n');
    dacWrite(Xchannel, Xvalue);
    Serial.print("Xvalue: ");
    Serial.println(Xvalue);

    Yvalue = Serial.parseInt();
    dacWrite(Ychannel, Yvalue);
    Serial.print("Yvalue: ");
    Serial.println(Yvalue);
  }
  */

  // dacWrite(Xchannel, Xvalue);
  // dacWrite(Ychannel, Yvalue);


  
  while (Serial.available() == 0) {
  }
  Serial.println("------------------------------");

  Xvalue = Serial.parseInt();
  dacWrite(Xchannel, Xvalue);
  Serial.print("Xvalue: ");
  Serial.println(Xvalue);

  Yvalue = Serial.parseInt();
  dacWrite(Ychannel, Yvalue);
  Serial.print("Yvalue: ");
  Serial.println(Yvalue);
  
}
