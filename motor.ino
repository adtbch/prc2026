#include "config.h"

// ====== Helper Functions ======
// Clamp nilai PWM ke range -MAX_PWM sampai MAX_PWM
static inline int clampPwm(float v) {
  if (v > MAX_PWM) return MAX_PWM;
  if (v < -MAX_PWM) return -MAX_PWM;
  return (int)v;
}

// ====== Setup PWM Channel ======
void setupPwmChannel(int channel, int pinEn) {
  ledcSetup(channel, PWM_FREQ, PWM_RES);
  ledcAttachPin(pinEn, channel);
  ledcWrite(channel, 0);
}

// ====== Kontrol Satu Motor L298N ======
// Mengatur motor dengan PWM: EN(PWM) + IN1/IN2
// pwm > 0: maju, pwm < 0: mundur, pwm = 0: berhenti (coast)
void setMotorL298N(int pwmChannel, int in1, int in2, float pwm) {
  int p = clampPwm(pwm);

  if (p > 0) {
    // Motor maju
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    ledcWrite(pwmChannel, p);
  } else if (p < 0) {
    // Motor mundur
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    ledcWrite(pwmChannel, -p);  // abs(p)
  } else {
    // Motor berhenti (coast)
    // Catatan: untuk brake, gunakan in1=HIGH, in2=HIGH
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    ledcWrite(pwmChannel, 0);
  }
}

// ====== Kontrol 3 Motor Gerak (M1, M3, M4) ======
// Parameter: pwm1, pwm3, pwm4 (range: -255 sampai 255)
// Positif = maju, Negatif = mundur, 0 = berhenti
void pwmMotor(float pwm1, float pwm3, float pwm4) {
  setMotorL298N(M1_PWM_CHANNEL, MOTOR1_A, MOTOR1_B, pwm1);
  setMotorL298N(M3_PWM_CHANNEL, MOTOR3_A, MOTOR3_B, pwm3);
  setMotorL298N(M4_PWM_CHANNEL, MOTOR4_A, MOTOR4_B, pwm4);
}

// ====== Kontrol Motor Lift (M2) ======
// Parameter: pwm (range: -255 sampai 255)
// Positif = naik, Negatif = turun, 0 = berhenti
void motorLift(float pwm) {
  setMotorL298N(M2_PWM_CHANNEL, MOTOR2_A, MOTOR2_B, pwm);
}