#include "config.h"

// Counter variables (bisa positif/negatif sesuai arah)
volatile long encoder1Count = 0;
volatile long encoder2Count = 0;
volatile long encoder3Count = 0;
volatile long encoder4Count = 0;

// Counter absolut untuk RPM (selalu positif, hitung total pergerakan)
volatile unsigned long encoder1RPM = 0;
volatile unsigned long encoder2RPM = 0;
volatile unsigned long encoder3RPM = 0;
volatile unsigned long encoder4RPM = 0;

int lastEnc1A = 0;
int lastEnc2A = 0;
int lastEnc3A = 0;
int lastEnc4A = 0;

// Struktur data PID
struct PIDData {
  float error;
  float integral;
  float derivative;
  float previousError;
};

// PID channels:
// 0..2 = wheel RPM (M1, M3, M4)
// 3    = yaw hold
// 4..  = bebas (misal PID kinematik X/Y)
// NOTE: kita besarkan supaya ada ruang untuk PID tambahan (mis. pidKinematik2 legacy).
const int numOutputs = 12;
PIDData pidData[numOutputs];

// ISR untuk Encoder 1
void IRAM_ATTR encoder1ISR() {
  int encAState = digitalRead(ENC_MOTOR1_A);
  int encBState = digitalRead(ENC_MOTOR1_B);

  if (encAState != lastEnc1A) {
    if (encAState == encBState) {
      encoder1Count++;
    } else {
      encoder1Count--;
    }
    encoder1RPM++;  // Selalu increment untuk RPM (tidak peduli arah)
    lastEnc1A = encAState;
  }
}

// ISR untuk Encoder 2
void IRAM_ATTR encoder2ISR() {
  int encAState = digitalRead(ENC_MOTOR2_A);
  int encBState = digitalRead(ENC_MOTOR2_B);

  if (encAState != lastEnc2A) {
    if (encAState == encBState) {
      encoder2Count++;
    } else {
      encoder2Count--;
    }
    encoder2RPM++;  // Selalu increment untuk RPM (tidak peduli arah)
    lastEnc2A = encAState;
  }
}

// ISR untuk Encoder 3
void IRAM_ATTR encoder3ISR() {
  int encAState = digitalRead(ENC_MOTOR3_A);
  int encBState = digitalRead(ENC_MOTOR3_B);

  if (encAState != lastEnc3A) {
    if (encAState == encBState) {
      encoder3Count++;
    } else {
      encoder3Count--;
    }
    encoder3RPM++;  // Selalu increment untuk RPM (tidak peduli arah)
    lastEnc3A = encAState;
  }
}

// ISR untuk Encoder 4
void IRAM_ATTR encoder4ISR() {
  int encAState = digitalRead(ENC_MOTOR4_A);
  int encBState = digitalRead(ENC_MOTOR4_B);

  if (encAState != lastEnc4A) {
    if (encAState == encBState) {
      encoder4Count++;
    } else {
      encoder4Count--;
    }
    encoder4RPM++;  // Selalu increment untuk RPM (tidak peduli arah)
    lastEnc4A = encAState;
  }
}

void setupEncoder() {
  // Setup pin A/B sebagai INPUT
  pinMode(ENC_MOTOR1_A, INPUT);
  pinMode(ENC_MOTOR1_B, INPUT);
  pinMode(ENC_MOTOR2_A, INPUT);
  pinMode(ENC_MOTOR2_B, INPUT);
  pinMode(ENC_MOTOR3_A, INPUT);
  pinMode(ENC_MOTOR3_B, INPUT);
  pinMode(ENC_MOTOR4_A, INPUT);
  pinMode(ENC_MOTOR4_B, INPUT);

  // Attach interrupt:
  // M1 & M2: interrupt di pin A (karena pin B nya 39 & 35 tidak support interrupt)
  // M3 & M4: interrupt di pin B
  attachInterrupt(digitalPinToInterrupt(ENC_MOTOR1_A), encoder1ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_MOTOR2_A), encoder2ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_MOTOR3_B), encoder3ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_MOTOR4_B), encoder4ISR, CHANGE);

  // Inisialisasi PID data
  for (int i = 0; i < numOutputs; ++i) {
    pidData[i].error = 0.0;
    pidData[i].integral = 0.0;
    pidData[i].derivative = 0.0;
    pidData[i].previousError = 0.0;
  }

  // Baca state awal
  lastEnc1A = digitalRead(ENC_MOTOR1_A);
  lastEnc2A = digitalRead(ENC_MOTOR2_A);
  lastEnc3A = digitalRead(ENC_MOTOR3_A);
  lastEnc4A = digitalRead(ENC_MOTOR4_A);
}

// Fungsi untuk reset counter RPM (berguna untuk mengukur RPM dalam interval waktu tertentu)
void resetEncoderRPM() {
  encoder1RPM = 0;
  encoder2RPM = 0;
  encoder3RPM = 0;
  encoder4RPM = 0;
}

// ====== VARIABEL RPM ======
unsigned long milisRPM = 0;
float rpm_motor1 = 0;
float rpm_motor2 = 0;
float rpm_motor3 = 0;
float rpm_motor4 = 0;

// Fungsi helper untuk cek interval
bool checkInterval(unsigned long interval, unsigned long &lastTime) {
  if (millis() - lastTime >= interval) {
    lastTime = millis();
    return true;
  }
  return false;
}

// ====== PEMBACAAN RPM ======
void pembacaan_RPM() {
  if (checkInterval(INTERVAL_RPM, milisRPM)) {
    // Hitung RPM: encoder_count * (60000 / interval_ms) / PPR
    // 60000 = 60 detik * 1000 ms
    rpm_motor1 = encoder1RPM * (60000.0 / INTERVAL_RPM) / PPR;
    rpm_motor2 = encoder2RPM * (60000.0 / INTERVAL_RPM) / PPR;
    rpm_motor3 = encoder3RPM * (60000.0 / INTERVAL_RPM) / PPR;
    rpm_motor4 = encoder4RPM * (60000.0 / INTERVAL_RPM) / PPR;

    // Reset counter encoder untuk interval berikutnya
    encoder1RPM = 0;
    encoder2RPM = 0;
    encoder3RPM = 0;
    encoder4RPM = 0;
  }
}

// Fungsi untuk mendapatkan nilai RPM (opsional, untuk kemudahan akses)
float getRPM_Motor1() { return rpm_motor1; }
float getRPM_Motor2() { return rpm_motor2; }
float getRPM_Motor3() { return rpm_motor3; }
float getRPM_Motor4() { return rpm_motor4; }
