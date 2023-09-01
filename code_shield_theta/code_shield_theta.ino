/*
  Theta Locomotion Control Algorithm (ESP)
  Developed by Felipe Machado and Mateus Santos da Silva
  Last modification: 25/08/2023
  Version: 0.4

- usable to read speed from hall efect sensor (speedLeftWheel and speedRightWheel)
- control joystick by typing the bits values (writeJoystickManually)

TODO:
[ ] do the feedback loop control to asset desired speed
*/

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

  // writeJoystickManually();
  // Serial.println("- - - - - - - - - - - - - - - - - - - - - - - - - - - -");


  wheel leftWheel = speedLeftWheel(&ALeftHall, &BLeftHall);
  // Serial.println("LEFT = " + String(leftWheel.velLinear));
  wheel rightWheel = speedRightWheel(&ARightHall, &BRightHall);
  // Serial.println("RIGHT = " + String(rightWheel.velLinear));
  robot theta = wheelsVelocity2robotVelocity(leftWheel.velLinear, rightWheel.velLinear);
  Serial.println("vLIN = " + String(theta.velLinear));
  // Serial.println("vANG = " + String(theta.velAngular));
  // Serial.println("- - - - - - - - - - - - - - - - - - - - - - - - - - - -");


  // robotVelocity2joystick(0.0, 0.0);
  // Serial.println("- - - - - - - - - - - - - - - - - - - - - - - - - - - -");
}



void controle_vel_linear(float velocidade_desejada, float velocidade_medida) {

  float velocidade_controlada;  // Control output
  float kp = 0.5;               // Proportional gain
  float ki = 2.0;               // Integral gain
  float kd = 0.01;              // Derivative gain

  float prevError = 0.0;
  float integral = 0.0;
  long prevTime = 0;
  unsigned long sampleTime = 100;  //Sample time in milliseconds

  unsigned long tempoAtual = millis();
  if (tempoAtual - prevTime >= sampleTime) {
    double error = velocidade_desejada - velocidade_medida;
    integral += error * (tempoAtual - prevTime);

    double derivative = (error - prevError) / (tempoAtual - prevTime);

    prevError = error;
    prevTime = tempoAtual;

    velocidade_controlada = kp * error + ki * integral + kd * derivative;
    // return velocidade_controlada;

    Serial.print("velocidade_desejada: ");
    Serial.print(velocidade_desejada);
    Serial.print(" velocidade_medida: ");
    Serial.print(velocidade_medida);
    Serial.print(" velocidade_controlada: ");
    Serial.println(velocidade_controlada);
  }
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
  float velLinearMAX = 0.7;   // (m/s) going forward
  float velLinearMIN = -0.5;  // (m/s) going reverse

  float velAngularMAX = 1.5;  // (rad/s) 1 rad = 60°
  float velAngularMIN = -1.5;

  Y_joy = 255 * (velLinear - velLinearMIN) / (velLinearMAX - velLinearMIN);
  X_joy = 255 * (velAngular - velAngularMIN) / (velAngularMAX - velAngularMIN);

  dacWrite(Xchannel, X_joy);
  dacWrite(Ychannel, Y_joy);

  Serial.println("X_joy: " + String(X_joy));
  Serial.println("Y_joy: " + String(Y_joy));
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
  Ahall->currentTime = micros();

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


void writeJoystickManually() {
  if (Serial.available() > 0) {  // No Line Ending

    int X_joy = Serial.parseFloat();
    int Y_joy = Serial.parseFloat();

    dacWrite(Xchannel, X_joy);
    dacWrite(Ychannel, Y_joy);
  }
}
