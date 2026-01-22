#include "config.h"
#include <math.h>
/*
  LEGACY KINEMATICS API (adaptasi untuk project ini)

  Isi:
  - degreeToRadian / radianToDegree
  - kecepatanxy / kecepatanxy2 (menghasilkan PWM ke pwmMotor)
  - pidKinematik2 (PID posisi X/Y + yaw, output -> kecepatanxy)
  - executewaypoint (memakai pidKinematik2, cocok untuk style lama)

  Catatan penting:
  - Unit posisi yang dipakai di sini: CM (menggunakan variabel global x_cm/y_cm dari odometry.ino)
  - Yaw yang dipakai: yaw1 (derajat)
  - Output translasi/rotasi "batasxy/batasyaw" dianggap skala PWM (open-loop), karena fungsi akhir memanggil pwmMotor().
*/

// ============================================================
// Utils
// ============================================================
float degreeToRadian(float degree) {
  return degree * (PI / 180.0f);
}

float radianToDegree(float radian) {
  return radian * (180.0f / PI);
}

static inline float _wrap360_legacy(float a) {
  while (a < 0.0f) a += 360.0f;
  while (a >= 360.0f) a -= 360.0f;
  return a;
}

// shortest angle error in [-180..180]
static inline float _angleErrorDeg_legacy(float targetDeg, float currentDeg) {
  float e = _wrap360_legacy(targetDeg) - _wrap360_legacy(currentDeg);
  if (e > 180.0f) e -= 360.0f;
  if (e < -180.0f) e += 360.0f;
  return e;
}

static inline float _absf_legacy(float x) { return x < 0.0f ? -x : x; }

static inline void _normalize3_legacy(float &a, float &b, float &c, float limit) {
  float m = _absf_legacy(a);
  float t = _absf_legacy(b); if (t > m) m = t;
  t = _absf_legacy(c);       if (t > m) m = t;
  if (m > limit && m > 0.0001f) {
    float s = limit / m;
    a *= s; b *= s; c *= s;
  }
}

// ============================================================
// Open-loop speed mixing (PWM)
// ============================================================
// Mengatur kecepatan yang dipengaruhi koordinat x, y dan yaw
// - vx, vy, vtheta: skala PWM/command
// - Output: pwmMotor(pwm1, pwm3, pwm4)
void kecepatanxy(int vx, int vy, int vtheta) {
  float rpm1, rpm2, rpm3;

  // 3 roda omni 120°
  const float a1 = 0.0f;
  const float a2 = 120.0f;
  const float a3 = 240.0f; // dari 220 -> 240 biar simetris 120°

  rpm1 = cosf(a1 * DEG_TO_RAD) * (float)vx - sinf(a1 * DEG_TO_RAD) * (float)vy - (float)vtheta;
  rpm2 = cosf(a2 * DEG_TO_RAD) * (float)vx - sinf(a2 * DEG_TO_RAD) * (float)vy - (float)vtheta;
  rpm3 = cosf(a3 * DEG_TO_RAD) * (float)vx - sinf(a3 * DEG_TO_RAD) * (float)vy - (float)vtheta;

  // Normalisasi agar tidak melebihi limit PWM
#ifndef LEGACY_MAX_PWM
  const float maxpwm = 1023.0f;
#else
  const float maxpwm = (float)LEGACY_MAX_PWM;
#endif
  _normalize3_legacy(rpm1, rpm2, rpm3, maxpwm);

  // Samakan dengan kecepatanxy2(): gunakan RPM control (closed-loop)
  // Catatan: variabel "rpm1/2/3" di sini hanyalah command; akan diperlakukan sebagai target RPM oleh rpmMotor().
  rpmMotor(-rpm1, -rpm2, -rpm3);
}

// Mengatur kecepatan yang dipengaruhi koordinat x dan y (versi lama 4 roda).
// Robot ini 3 roda, jadi fungsi ini dipertahankan sebagai kompatibilitas:
// - kita hitung 3 komponen saja, lalu kirim ke pwmMotor.
void kecepatanxy2(int vx, int vy, int vtheta) {
  float rpm1, rpm2, rpm3;

  const float a1 = 0.0f;
  const float a2 = 120.0f;
  const float a3 = 240.0f;

  // Samakan konvensi dengan kecepatanxy():
  // - vy (+) = maju
  // Sebelumnya tanda vy kebalik sehingga vy (+) terasa mundur.
  rpm1 = cosf(a1 * DEG_TO_RAD) * (float)vx - sinf(a1 * DEG_TO_RAD) * (float)vy - (float)vtheta;
  rpm2 = cosf(a2 * DEG_TO_RAD) * (float)vx - sinf(a2 * DEG_TO_RAD) * (float)vy - (float)vtheta;
  rpm3 = cosf(a3 * DEG_TO_RAD) * (float)vx - sinf(a3 * DEG_TO_RAD) * (float)vy - (float)vtheta;

#ifndef LEGACY_MAX_PWM
  const float maxpwm = 1023.0f;
#else
  const float maxpwm = (float)LEGACY_MAX_PWM;
#endif
  _normalize3_legacy(rpm1, rpm2, rpm3, maxpwm);

//   pwmMotor(rpm1, rpm2, rpm3);
  rpmMotor(-rpm1, -rpm2, -rpm3);
}

// ============================================================
// GLOBAL / FIELD-CENTRIC helper (legacy)
// ============================================================
// Konvensi GLOBAL (sama seperti odometry & kinematics.ino):
// - xRightGlobal (+)  : kanan
// - yForwardGlobal (+): maju
//
// Fungsi ini mengubah command GLOBAL -> LOCAL (robot frame) memakai yaw1,
// lalu memanggil kecepatanxy2() (yang sifatnya LOCAL).
static inline float _degToRad_legacy(float d) { return d * 0.01745329251994329577f; }

void kecepatanxy2Global(float xRightGlobal, float yForwardGlobal, float vtheta) {
  float yawDeg = yaw1;
#if WORLD_YAW_INVERT
  yawDeg = -yawDeg;
#endif
  float yawRad = _degToRad_legacy(yawDeg);
  float c = cosf(yawRad);
  float s = sinf(yawRad);

  // global -> local (vx=kanan, vy=maju)
  float vx_local =  c * xRightGlobal + s * yForwardGlobal;
  float vy_local = -s * xRightGlobal + c * yForwardGlobal;

  kecepatanxy2((int)vx_local, (int)vy_local, (int)vtheta);
}

// ============================================================
// PID Kinematik Legacy -> pidKinematik2
// ============================================================
// PID koordinat X(cm), Y(cm) dan arah hadap robot (deg)
//
// Parameter:
// - vx, vy   : target posisi (cm)
// - sudut    : target yaw (deg 0..360)
// - batasxy  : limit output translasi (skala PWM)
// - batasyaw : limit output rotasi (skala PWM)
//
// NOTE:
// - Ini open-loop di ujungnya (kecepatanxy -> pwmMotor). Kalau mau closed-loop RPM, pakai pidKinematik() baru.
void pidKinematik2(float vx, float vy, float sudut, int batasxy, int batasyaw) {
  // Channel PID khusus legacy, biar tidak bentrok:
  // 7 = yaw, 8 = X, 9 = Y
#ifndef PIDKIN2_YAW_CHANNEL
  #define PIDKIN2_YAW_CHANNEL 7
#endif
#ifndef PIDKIN2_X_CHANNEL
  #define PIDKIN2_X_CHANNEL 8
#endif
#ifndef PIDKIN2_Y_CHANNEL
  #define PIDKIN2_Y_CHANNEL 9
#endif

  // Gain default (bisa kamu pindah ke config.h kalau mau)
#ifndef PIDKIN2_XY_KP
  const double kp2 = 15.0;
#else
  const double kp2 = (double)PIDKIN2_XY_KP;
#endif
#ifndef PIDKIN2_XY_KI
  const double ki2 = 0.0;
#else
  const double ki2 = (double)PIDKIN2_XY_KI;
#endif
#ifndef PIDKIN2_XY_KD
  const double kd2 = 0.0;
#else
  const double kd2 = (double)PIDKIN2_XY_KD;
#endif

#ifndef PIDKIN2_YAW_KP
  const double kp = 8.0;
#else
  const double kp = (double)PIDKIN2_YAW_KP;
#endif
#ifndef PIDKIN2_YAW_KI
  const double ki = 0.0;
#else
  const double ki = (double)PIDKIN2_YAW_KI;
#endif
#ifndef PIDKIN2_YAW_KD
  const double kd = 0.0;
#else
  const double kd = (double)PIDKIN2_YAW_KD;
#endif

  // Integral clamp aman (hindari div0)
  double minI2 = 0.0, maxI2 = 0.0;
  if (ki2 != 0.0) {
    maxI2 = (double)batasxy / ki2;
    minI2 = -maxI2;
  }
  double minI = 0.0, maxI = 0.0;
  if (ki != 0.0) {
    maxI = (double)batasyaw / ki;
    minI = -maxI;
  }

  // Current posisi (cm) dari odom
  const float curX = x_cm; // X kanan (+)
  const float curY = y_cm; // Y maju  (+)

  // PID posisi -> command
  double pidx = computePID((int)PIDKIN2_X_CHANNEL, (double)vx, (double)curX, kp2, ki2, kd2, minI2, maxI2);
  double pidy = computePID((int)PIDKIN2_Y_CHANNEL, (double)vy, (double)curY, kp2, ki2, kd2, minI2, maxI2);

  // PID yaw: pakai error sudut terpendek
  float yawErr = _angleErrorDeg_legacy(sudut, yaw1);
  double pidyaw = computePID((int)PIDKIN2_YAW_CHANNEL, (double)yawErr, 0.0, kp, ki, kd, minI, maxI);

  // Clamp output
  if (pidx > (double)batasxy) pidx = (double)batasxy;
  if (pidx < -(double)batasxy) pidx = -(double)batasxy;
  if (pidy > (double)batasxy) pidy = (double)batasxy;
  if (pidy < -(double)batasxy) pidy = -(double)batasxy;
  if (pidyaw > (double)batasyaw) pidyaw = (double)batasyaw;
  if (pidyaw < -(double)batasyaw) pidyaw = -(double)batasyaw;

  // Ikuti arah tanda versi lama.
  // Karena pidx/pidy dihitung di koordinat GLOBAL (x_cm/y_cm), pakai helper global
  // agar translasi tetap field-centric (tidak ikut muter saat yaw berubah).
  kecepatanxy2Global((float)(-pidx), (float)(-pidy), (float)(-pidyaw));
//   kecepatanxy((int)(-pidx), (int)(-pidy), (int)(-pidyaw));
}

// execute waypoint versi legacy
// Unit targetx/targety: cm
bool executewaypoint(float targetx, float targety, int targetyaw,
                     float treshold, int tresholdyaw,
                     int targetrpm, int targetrpmyaw) {
  pidKinematik2(targetx, targety, (float)targetyaw, targetrpm, targetrpmyaw);

  float deltaX = targetx - x_cm;
  float deltaY = targety - y_cm;
  float distance = sqrtf(deltaX * deltaX + deltaY * deltaY); // cm

  float deltayaw = _angleErrorDeg_legacy((float)targetyaw, yaw1);

  if (distance < treshold && _absf_legacy(deltayaw) <= (float)tresholdyaw) {
    // stop
    pwmMotor(0, 0, 0);
    return true;
  }
  return false;
}

