
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver PCA = Adafruit_PWMServoDriver(0x40);

// ---------------- ENCODERS ----------------
volatile long encoderCount[4] = {0, 0, 0, 0};
long prevEncoderCount[4] = {0, 0, 0, 0};

portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

// encoder pins
const int pinA = 34;
const int pinB = 35;
const int pinC = 32;
const int pinD = 33;
const int pinE = 25;
const int pinF = 26;
const int pinG = 27;
const int pinH = 14;

// ---------------- CONTROL ----------------
// rpm[4] = {0, 0, 0, 0};
float rpmRaw[4]  = {0, 0, 0, 0};   // direct RPM from encoder counts
float rpmFilt[4] = {0, 0, 0, 0};   // filtered RPM used for control
float targetRPM[4] = {50, 50, 50, 50};

float Kp[4] = {1, 1, 1, 1};
float Ki[4] = {0.0, 0.0, 0.0, 0.0};
float integral[4] = {0, 0, 0, 0};

float pwmCmd[4] = {260, 260, 260, 260};

const float DT = 0.05;   // 20 ms loop

const int PWM_STOP = 307;
const int PWM_FAST = 204;

// ---------------- ISR ----------------
void IRAM_ATTR encoderISR0A() {
  portENTER_CRITICAL_ISR(&mux);
  if (digitalRead(pinA) == digitalRead(pinB)) encoderCount[0]++;
  else encoderCount[0]--;
  portEXIT_CRITICAL_ISR(&mux);
}

void IRAM_ATTR encoderISR0B() {
  portENTER_CRITICAL_ISR(&mux);
  if (digitalRead(pinA) != digitalRead(pinB)) encoderCount[0]++;
  else encoderCount[0]--;
  portEXIT_CRITICAL_ISR(&mux);
}

void IRAM_ATTR encoderISR1A() {
  portENTER_CRITICAL_ISR(&mux);
  if (digitalRead(pinC) == digitalRead(pinD)) encoderCount[1]++;
  else encoderCount[1]--;
  portEXIT_CRITICAL_ISR(&mux);
}

void IRAM_ATTR encoderISR1B() {
  portENTER_CRITICAL_ISR(&mux);
  if (digitalRead(pinC) != digitalRead(pinD)) encoderCount[1]++;
  else encoderCount[1]--;
  portEXIT_CRITICAL_ISR(&mux);
}

void IRAM_ATTR encoderISR2A() {
  portENTER_CRITICAL_ISR(&mux);
  if (digitalRead(pinE) == digitalRead(pinF)) encoderCount[2]++;
  else encoderCount[2]--;
  portEXIT_CRITICAL_ISR(&mux);
}

void IRAM_ATTR encoderISR2B() {
  portENTER_CRITICAL_ISR(&mux);
  if (digitalRead(pinE) != digitalRead(pinF)) encoderCount[2]++;
  else encoderCount[2]--;
  portEXIT_CRITICAL_ISR(&mux);
}

void IRAM_ATTR encoderISR3A() {
  portENTER_CRITICAL_ISR(&mux);
  if (digitalRead(pinG) == digitalRead(pinH)) encoderCount[3]++;
  else encoderCount[3]--;
  portEXIT_CRITICAL_ISR(&mux);
}

void IRAM_ATTR encoderISR3B() {
  portENTER_CRITICAL_ISR(&mux);
  if (digitalRead(pinG) != digitalRead(pinH)) encoderCount[3]++;
  else encoderCount[3]--;
  portEXIT_CRITICAL_ISR(&mux);
}

// ---------------- RPM + CONTROL ----------------
void updateRPM() {
  long current[4];

  // safely copy encoder counts
  portENTER_CRITICAL(&mux);
  for (int i = 0; i < 4; i++) {
    current[i] = encoderCount[i];
  }
  portEXIT_CRITICAL(&mux);

  for (int i = 0; i < 4; i++) {
    long delta = current[i] - prevEncoderCount[i];
    prevEncoderCount[i] = current[i];

    float cps = delta / DT;   // counts per second

    // raw RPM from encoder counts
    rpmRaw[i] = (cps * 60.0) / (7.0 * 4.0 * 50.9);

    // ---------- LOW PASS FILTER ----------
    // alpha between 0 and 1
    // smaller alpha = smoother but slower
    // larger alpha = faster but noisier
    float alpha = 0.5;

    rpmFilt[i] = alpha * rpmRaw[i] + (1.0 - alpha) * rpmFilt[i];
  }
}

void controlMotors() {
  for (int i = 0; i < 4; i++) {

    float error = targetRPM[i] - rpmFilt[i];

    integral[i] += error * DT;
    integral[i] = constrain(integral[i], -100, 100);

    float control = Kp[i] * error + Ki[i] * integral[i];

    pwmCmd[i] -= control;

    pwmCmd[i] = constrain(pwmCmd[i], PWM_FAST, PWM_STOP);
  }

  PCA.setPWM(2, 0, (int)pwmCmd[0]);
  PCA.setPWM(3, 0, (int)pwmCmd[1]);
  PCA.setPWM(0, 0, (int)pwmCmd[2]);
  PCA.setPWM(1, 0, (int)pwmCmd[3]);
}

// ---------------- TASK 1: CONTROL LOOP ----------------
void controlTask(void *pvParameters) {
  TickType_t lastWake = xTaskGetTickCount();

  while (1) {
    updateRPM();
    controlMotors();

    vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(50));
  }
}

// ---------------- TASK 2: SERIAL DEBUG ----------------
void debugTask(void *pvParameters) {
  TickType_t lastWake = xTaskGetTickCount();

  while (1) {
    Serial.print("Raw RPM: ");
    Serial.print(rpmRaw[0]); Serial.print(" , ");
    Serial.print(rpmRaw[1]); Serial.print(" , ");
    Serial.print(rpmRaw[2]); Serial.print(" , ");
    Serial.println(rpmRaw[3]);

    Serial.print("Filt RPM: ");
    Serial.print(rpmFilt[0]); Serial.print(" , ");
    Serial.print(rpmFilt[1]); Serial.print(" , ");
    Serial.print(rpmFilt[2]); Serial.print(" , ");
    Serial.println(rpmFilt[3]);

    // Serial.print("PWM: ");
    // Serial.print((int)pwmCmd[0]); Serial.print(" , ");
    // Serial.print((int)pwmCmd[1]); Serial.print(" , ");
    // Serial.print((int)pwmCmd[2]); Serial.print(" , ");
    // Serial.println((int)pwmCmd[3]);

    vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(200)); // slow print
  }
}

// ---------------- SETUP ----------------
void setup() {
  Serial.begin(115200);

  Wire.begin();
  PCA.begin();
  PCA.setPWMFreq(50);
  pinMode(34, INPUT);
  pinMode(35, INPUT);
  pinMode(32, INPUT);
  pinMode(33, INPUT);
  pinMode(25, INPUT);
  pinMode(26, INPUT);
  pinMode(27, INPUT);
  pinMode(14, INPUT);

  attachInterrupt(digitalPinToInterrupt(34), encoderISR0A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(35), encoderISR0B, CHANGE);
  attachInterrupt(digitalPinToInterrupt(32), encoderISR1A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(33), encoderISR1B, CHANGE);
  attachInterrupt(digitalPinToInterrupt(25), encoderISR2A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(26), encoderISR2B, CHANGE);
  attachInterrupt(digitalPinToInterrupt(27), encoderISR3A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(14), encoderISR3B, CHANGE);

  PCA.setPWM(0, 0, 260);
  PCA.setPWM(1, 0, 260);
  PCA.setPWM(2, 0, 260);
  PCA.setPWM(3, 0, 260);

  xTaskCreatePinnedToCore(controlTask, "control", 4096, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(debugTask, "debug", 4096, NULL, 1, NULL, 0);

  Serial.println("AGV started clean architecture");
}

void loop() {}