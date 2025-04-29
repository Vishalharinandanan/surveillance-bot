const int enA = 6;    // Left motor PWM pin
const int in1 = 11;
const int in2 = 12;

const int enB = 9;    // Right motor PWM pin
const int in3 = 8;
const int in4 = 10;

const int encoderLeft = 3, encoderRight = 2;
volatile long ticksLeft = 0, ticksRight = 0;

float targetRPM_Left = 0, targetRPM_Right = 0;
float measuredRPM_Left = 0, measuredRPM_Right = 0;
float pwmLeft = 0, pwmRight = 0;
float prevErrorLeft = 0, prevErrorRight = 0;

const float Kp = 1.6, Kd = 0.1;
const int TICKS_PER_REV = 350, SAMPLE_INTERVAL_MS = 100;
unsigned long lastRPMCalcTime = 0;
unsigned long lastSerialTime = 0;

void setup() {
  Serial.begin(9600);
  
  pinMode(enA, OUTPUT); pinMode(in1, OUTPUT); pinMode(in2, OUTPUT);
  pinMode(enB, OUTPUT); pinMode(in3, OUTPUT); pinMode(in4, OUTPUT);
  pinMode(encoderLeft, INPUT_PULLUP); pinMode(encoderRight, INPUT_PULLUP);
  
  attachInterrupt(digitalPinToInterrupt(encoderLeft), countLeft, RISING);
  attachInterrupt(digitalPinToInterrupt(encoderRight), countRight, RISING);
}

void loop() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    int comma = input.indexOf(',');
    if (comma > 0) {
      targetRPM_Left = input.substring(0, comma).toFloat();
      targetRPM_Right = input.substring(comma + 1).toFloat();
      lastSerialTime = millis();
    }
  }

  if (millis() - lastSerialTime > 500) {
    targetRPM_Left = 0;
    targetRPM_Right = 0;
  }

  unsigned long currentTime = millis();
  if (currentTime - lastRPMCalcTime >= SAMPLE_INTERVAL_MS) {
    noInterrupts();
    long ticksL = ticksLeft, ticksR = ticksRight;
    ticksLeft = 0; ticksRight = 0;
    interrupts();

    float dt = (currentTime - lastRPMCalcTime) / 60000.0;
    measuredRPM_Left = (ticksL / (float)TICKS_PER_REV) / dt;
    measuredRPM_Right = (ticksR / (float)TICKS_PER_REV) / dt;

    float errorLeft = targetRPM_Left - measuredRPM_Left;
    float errorRight = targetRPM_Right - measuredRPM_Right;
    float dLeft = (errorLeft - prevErrorLeft) / (SAMPLE_INTERVAL_MS / 1000.0);
    float dRight = (errorRight - prevErrorRight) / (SAMPLE_INTERVAL_MS / 1000.0);

    pwmLeft += Kp * errorLeft + Kd * dLeft;
    pwmRight += Kp * errorRight + Kd * dRight;

    pwmLeft = constrain(pwmLeft, 0, 255);
    pwmRight = constrain(pwmRight, 0, 255);

    // LEFT MOTOR DIRECTION
    if (pwmLeft >= 0) {
      digitalWrite(in1, LOW); digitalWrite(in2, HIGH);
      analogWrite(enA, (int)pwmLeft);
    } else {
      digitalWrite(in1, HIGH); digitalWrite(in2, LOW);
      analogWrite(enA, (int)(-pwmLeft));
    }

    // RIGHT MOTOR DIRECTION
    if (pwmRight >= 0) {
      digitalWrite(in3, LOW); digitalWrite(in4, HIGH);
      analogWrite(enB, (int)pwmRight);
    } else {
      digitalWrite(in3, HIGH); digitalWrite(in4, LOW);
      analogWrite(enB, (int)(-pwmRight));
    }

    prevErrorLeft = errorLeft;
    prevErrorRight = errorRight;
    lastRPMCalcTime = currentTime;
  }
}

void countLeft() { ticksLeft++; }
void countRight() { ticksRight++; }

