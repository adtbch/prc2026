// PID umum (tidak ada wrap-angle). Untuk yaw/angle, gunakan fungsi khusus.
double computePID(int index, double setpoint, double input,
                  double Kp, double Ki, double Kd,
                  double Minintegral, double Maxintegral) {
  // Hitung error
  pidData[index].error = setpoint - input;

  // Integral & derivative
  pidData[index].integral += pidData[index].error;
  pidData[index].integral = constrain(pidData[index].integral, Minintegral, Maxintegral);
  pidData[index].derivative = pidData[index].error - pidData[index].previousError;
  // buat index 3 kalo error abs 5 maka error 0
  if (index == YAW_PID_CHANNEL) {
    if (abs(pidData[index].error) < 3) {
      pidData[index].error = 0;
    }
  }
  // Deadband untuk PIDKIN_YAW_CHANNEL (channel 6) - kurangi oscillation
  if (index == PIDKIN_YAW_CHANNEL) {
    if (abs(pidData[index].error) < 3) {
      pidData[index].error = 0;
    }
  }
  // Output
  double output = Kp * pidData[index].error
                + Ki * pidData[index].integral
                + Kd * pidData[index].derivative;

  pidData[index].previousError = pidData[index].error;
  return output;
}

// ============================================================
// RPM CONTROL - OMNI 3 RODA (Motor 1, 3, 4)
// Target masuk dalam satuan RPM (boleh negatif untuk arah).
// Feedback pakai variabel kita: getRPM_Motor1(), getRPM_Motor3(), getRPM_Motor4().
// Output: PWM signed ke pwmMotor(pwm1, pwm3, pwm4).
// ============================================================

#include "config.h"

#ifndef RPM_PID_KP
  #define RPM_PID_KP 1.0
#endif
#ifndef RPM_PID_KI
  #define RPM_PID_KI 0.08
#endif
#ifndef RPM_PID_KD
  #define RPM_PID_KD 0.0
#endif

// Batas output PWM untuk kontrol RPM (default: MAX_PWM)
#ifndef RPM_PID_PWM_LIMIT
  #define RPM_PID_PWM_LIMIT MAX_PWM
#endif

// Minimal PWM saat target != 0 (buat ngalahin deadband motor). 0 = off.
#ifndef RPM_PID_MIN_PWM
  #define RPM_PID_MIN_PWM 0
#endif

// Jangan pakai minPWM untuk target RPM yang sangat kecil (biar tidak jitter/nyentak pada koreksi kecil).
// minPWM hanya aktif jika |targetRpm| >= RPM_PID_MIN_RPM_FOR_MINPWM.
#ifndef RPM_PID_MIN_RPM_FOR_MINPWM
  #define RPM_PID_MIN_RPM_FOR_MINPWM 60.0f
#endif

static inline float _absf_rpm(float x) { return x < 0 ? -x : x; }

void resetPidChannel(int idx) {
  pidData[idx].error = 0.0f;
  pidData[idx].integral = 0.0f;
  pidData[idx].derivative = 0.0f;
  pidData[idx].previousError = 0.0f;
}

// Set state awal agar derivative tidak spike (biasanya dipakai saat enable kontrol)
void primePidChannel(int idx, float error) {
  pidData[idx].error = error;
  pidData[idx].integral = 0.0f;
  pidData[idx].derivative = 0.0f;
  pidData[idx].previousError = error;
}

static float _rpmPidOne(int pidIdx,
                        float targetRpm,
                        float measuredRpm,
                        double kp,
                        double ki,
                        double kd,
                        float minPwm) {
  if (targetRpm == 0.0f) {
    resetPidChannel(pidIdx);
    return 0.0f;
  }

  const float pwmLimit = (float)RPM_PID_PWM_LIMIT;

  // Integral clamp: kira-kira supaya Ki*I tidak melewati pwmLimit
  double minI = 0.0, maxI = 0.0;
  if (ki != 0.0) {
    maxI = pwmLimit / ki;
    minI = -maxI;
  }

  // Encoder RPM kita adalah magnitude (selalu positif), jadi kita kontrol magnitude
  float tgt = _absf_rpm(targetRpm);
  float in = (measuredRpm < 0.0f) ? -measuredRpm : measuredRpm;

  double mag = computePID(pidIdx, (double)tgt, (double)in, kp, ki, kd, minI, maxI);

  // Jangan pernah membalik arah karena overspeed: clamp magnitude ke [0..pwmLimit]
  if (mag < 0.0) mag = 0.0;
  if (mag > pwmLimit) mag = pwmLimit;

  float out = (float)mag;
  if (minPwm > 0.0f && tgt >= (float)RPM_PID_MIN_RPM_FOR_MINPWM && out > 0.0f && out < minPwm) {
    out = minPwm;
  }

  return (targetRpm < 0.0f) ? -out : out;
}

// Urutan parameter mengikuti roda w1,w2,w3 dari kinematik:
// - w1 -> Motor 1
// - w2 -> Motor 3
// - w3 -> Motor 4
void rpmMotor(float rpm_w1, float rpm_w2, float rpm_w3) {
  // Per-wheel calibration:
  // - scale untuk kompensasi roda/motor yang "lebih berat" (biar jalan lurus)
  // - min PWM per-roda untuk ngalahin deadband yang beda-beda
  float pwm1 = _rpmPidOne(
    0,
    rpm_w1 * (float)RPM_W1_SCALE,
    getRPM_Motor1(),
    (double)RPM_W1_KP, (double)RPM_W1_KI, (double)RPM_W1_KD,
    (float)RPM_PID_MIN_PWM_W1
  );
  float pwm3 = _rpmPidOne(
    1,
    rpm_w2 * (float)RPM_W2_SCALE,
    getRPM_Motor3(),
    (double)RPM_W2_KP, (double)RPM_W2_KI, (double)RPM_W2_KD,
    (float)RPM_PID_MIN_PWM_W2
  );
  float pwm4 = _rpmPidOne(
    2,
    rpm_w3 * (float)RPM_W3_SCALE,
    getRPM_Motor4(),
    (double)RPM_W3_KP, (double)RPM_W3_KI, (double)RPM_W3_KD,
    (float)RPM_PID_MIN_PWM_W3
  );
  pwmMotor(pwm1, pwm3, pwm4);
}
 