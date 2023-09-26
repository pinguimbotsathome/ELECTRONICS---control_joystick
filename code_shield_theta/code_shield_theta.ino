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


float a = 0.0;
float b = 0.0;




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


void setup() {
  Serial.begin(115200);

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

  if (Serial.available() > 0) {  // No Line Ending

    a = Serial.parseFloat();
    Serial.println(a);
    b = Serial.parseFloat();
    Serial.println(b);
  }

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

  // float contrVelLin = controle_vel_linear(a, theta.velLinear);
  // float contrVelAng = controle_vel_angular(0.0, theta.velAngular);
  // robotVelocity2joystick(contrVelLin, contrVelAng);
  robotVelocity2joystick(a, b);

  // writeJoystickManually();
  // Serial.println("- - - - - - - - - - - - - - - - - - - - - - - - - - - -");
}



float controle_vel_linear(float vel_desejada, float vel_medida) {
  float kp = 0.45;
  float ki = 0.1;

  static float prevError;
  static float integral;
  static unsigned long prevTime;
  static float prevControle;

  unsigned long sampleTime = 200;
  unsigned long tempoAtual = millis();
  if (tempoAtual - prevTime >= sampleTime) {
    prevTime = tempoAtual;
    float error = vel_desejada - vel_medida;
    float proporcional = kp * error;
    integral += ki * error;

    float Controle = proporcional + integral;

    if (Controle > 0.5) {
      Controle = 0.5;
    } else if (Controle < -0.6) {
      Controle = -0.6;
    }

    prevControle = Controle;
    prevError = error;

    // Serial.print("LINEAR - vel_des: " + String(vel_desejada));
    // Serial.print("   vel_med: " + String(vel_medida));
    // Serial.println("   con: " + String(Controle));

    return Controle;
  }
  return prevControle;
}

float controle_vel_angular(float vel_desejada, float vel_medida) {
  float kp = 0.4;
  float ki = 0.1;

  static float prevError;
  static float integral;
  static unsigned long prevTime;
  static float prevControle;

  unsigned long sampleTime = 200;
  unsigned long tempoAtual = millis();
  if (tempoAtual - prevTime >= sampleTime) {
    prevTime = tempoAtual;
    float error = vel_desejada - vel_medida;
    float proporcional = kp * error;
    integral += ki * error;

    float Controle = proporcional + integral;

    if (Controle > 1.5) {
      Controle = 1.5;
    } else if (Controle < -1.5) {
      Controle = -1.5;
    }

    prevControle = Controle;
    prevError = error;

    // Serial.print("ANGULAR - vel_des: " + String(vel_desejada));
    // Serial.print("   vel_med: " + String(vel_medida));
    // Serial.println("   con: " + String(Controle));

    return Controle;
  }
  return prevControle;
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


void writeJoystickManually() {
  if (Serial.available() > 0) {  // No Line Ending

    int X_joy = Serial.parseFloat();
    int Y_joy = Serial.parseFloat();

    Serial.println("X: " + String(X_joy));
    Serial.println("Y: " + String(Y_joy));

    dacWrite(Xchannel, X_joy);
    dacWrite(Ychannel, Y_joy);
  }
}
