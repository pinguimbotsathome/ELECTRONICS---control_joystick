int ledPin = 10;
int ldrPin = A0;
int potPin = A4;

void setup() {
  Serial.begin(9600);
  pinMode(ledPin, OUTPUT);
  pinMode(ldrPin, INPUT);
  pinMode(potPin, INPUT);
  analogWrite(ledPin, 0);
}

void loop() {
  int ldrValue = analogRead(ldrPin);
  int potValue = analogRead(potPin);

  int ldrValue_filtered = LPF(ldrValue);

  Serial.print(ldrValue);Serial.print(" , ");
  Serial.println(ldrValue_filtered);
}

float LPF(float inputSignal) {
  // https://www.youtube.com/watch?v=eM4VHtettGg

  static float a = 0.8;
  static float b = 0.1;
  static float filteredSignal;
  static float inputSignalPrev;
  static float filteredSignalPrev;

  static unsigned long previousMicros;
  static int sampleInterval = 10000;

  unsigned long currentMicros = micros();
  if (currentMicros - previousMicros >= sampleInterval) {
    previousMicros = currentMicros;

    filteredSignal = a * filteredSignalPrev + b * (inputSignal + inputSignalPrev);

    inputSignalPrev = inputSignal;
    filteredSignalPrev = filteredSignal;
    return filteredSignal;
  }
}
