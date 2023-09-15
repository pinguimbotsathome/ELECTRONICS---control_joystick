/*
  Theta Locomotion Control Algorithm (ESP)
  Developed by Felipe Machado and Mateus Santos da Silva
  Last modification: 06/09/2023
  Version: 0.5

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

Hall ALeftHall = { 5, true, 0, 0, 0, 0 };    // white - orange
Hall BLeftHall = { 18, true, 0, 0, 0, 0 };   // brown - blue
Hall ARightHall = { 19, true, 0, 0, 0, 0 };  // gray - orange
Hall BRightHall = { 21, true, 0, 0, 0, 0 };  // blue - blue


// variables - change radius and stoppedTime accordingly

int stoppedTime = 300;      // 0.3s = 300 000
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

  writeJoystickManually();
  // Serial.println("- - - - - - - - - - - - - - - - - - - - - - - - - - - -");


  wheel leftWheel = speedLeftWheel(&ALeftHall, &BLeftHall);
  float leftWheel_filtered = recursiveFilter(leftWheel.velLinear);
  // Serial.println("LEFT = " + String(leftWheel_filtered));

  wheel rightWheel = speedRightWheel(&ARightHall, &BRightHall);
  float rightWheel_filtered = recursiveFilter(rightWheel.velLinear);
  // Serial.println("RIGHT = " + String(rightWheel.velLinear));
  robot theta = wheelsVelocity2robotVelocity(leftWheel_filtered, rightWheel_filtered);
  // Serial.println("vLIN = " + String(theta.velLinear));
  // Serial.println("vANG = " + String(theta.velAngular));
  // Serial.println("- - - - - - - - - - - - - - - - - - - - - - - - - - - -");

  // Serial.println(theta.velLinear);

  float contrVelLin = controle_vel_linear(0.4, theta.velLinear);
  robotVelocity2joystick(contrVelLin, 0.0);


  // writeJoystickManually();
  // Serial.println("- - - - - - - - - - - - - - - - - - - - - - - - - - - -");
}



float controle_vel_linear(float vel_desejada, float vel_medida) {

  float vel_controlada;  // Control output
  float kp = 10.0;       // Proportional gain
  float ki = 0.001;      // Integral gain
  float kd = 0.0001;     // Derivative gain

  float prevError;
  float integral;
  long prevTime;
  unsigned long sampleTime = 100;  //Sample time in milliseconds

  unsigned long tempoAtual = xTaskGetTickCount();
  if (tempoAtual - prevTime >= sampleTime) {

    double error = vel_desejada - vel_medida;


    integral += error;
    double derivative = (error - prevError);

    prevError = error;
    prevTime = tempoAtual;

    vel_controlada = kp * error + ki * integral + kd * derivative;

    float a = vel_desejada + vel_controlada;
    if (a > 0.5) {
      a = 0.5;
    } else if (a < -0.6) {
      a = -0.6;
    }

    Serial.print("vel_desejada: " + String(vel_desejada));
    Serial.print("   vel_medida: " + String(vel_medida));
    Serial.println("   vel_controlada: " + String(a));
    return a;
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
  float velLinearMAX = 0.5;   // (m/s) going forward
  float velLinearMIN = -0.6;  // (m/s) going reverse

  float velAngularMAX = 1.5;  // (rad/s) 1 rad = 60°
  float velAngularMIN = -1.5;

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


float recursiveFilter(float speed_measured) {
  static int sample_interval = 10;  // microseconds
  static int sample_qtd = 10;
  static float speed_filtered;
  static unsigned long last_update_time;

  unsigned long current_time = xTaskGetTickCount();

  if (current_time - last_update_time >= sample_interval) {
    if (speed_measured <= speed_filtered + 0.7) {
      speed_filtered += (speed_measured - speed_filtered) / (float)sample_qtd;
      last_update_time = current_time;
    }
  }
  return speed_filtered;
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
