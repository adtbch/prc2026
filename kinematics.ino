#include "config.h"
#include <math.h>

// ============================================================
// KINEMATIK OMNI 3 RODA (KIWI DRIVE) - 120°
// ============================================================
// Konvensi sumbu robot (robot frame) SETELAH SWAP:
// - vx  (+X) : ke kanan (right)
// - vy  (+Y) : maju (forward)
// - w   (+)  : rotasi CCW (berlawanan jarum jam)
//
// Susunan roda 120° (sesuai model umum):
// θ1 =   0°, θ2 = 120°, θ3 = 240°  (θ = posisi roda terhadap +Y forward)
// Arah putar roda mengikuti arah "tangen" lingkaran (omni wheel).
//
// Inverse kinematics (body -> wheel) dengan swap vx<->vy:
// Input API: (vx=right, vy=forward) -> Internal: (vx=forward, vy=right)
// w1 = (        0*vx + 1*vy + L*w ) / r
// w2 = ( -0.866*vx - 0.5*vy + L*w ) / r
// w3 = ( +0.866*vx - 0.5*vy + L*w ) / r
//
// Forward kinematics (wheel -> body):
// vx = (r/√3) * (w3 - w2)
// vy = (r/3)  * (2*w1 - w2 - w3)
//  w = (r/(3L)) * (w1 + w2 + w3)
//
// Catatan penting:
// - Kalau kamu ingin "mixing PWM" saja (tanpa satuan meter/rad),
//   set r=1 dan L=1 (default di bawah), maka vx,vy,w bisa langsung skala PWM.
// - Jika arah roda / nomor roda beda dari gambar kamu, cukup swap w1/w2/w3
//   pada pemanggilan pwmMotor di moveRobot().

// Parameter fisik (opsional). Kalau belum didefinisikan di config.h, pakai 1.
#ifndef OMNI_WHEEL_RADIUS
  #define OMNI_WHEEL_RADIUS 1.0f
#endif
#ifndef OMNI_ROBOT_RADIUS
  #define OMNI_ROBOT_RADIUS 1.0f
#endif

// Invert arah motor (0 = normal, 1 = dibalik).
// Ini berguna kalau ada motor yang arah majunya kebalik / wiring terbalik.
#ifndef MOTOR1_INVERT
  #define MOTOR1_INVERT 0
#endif
#ifndef MOTOR3_INVERT
  #define MOTOR3_INVERT 0
#endif
#ifndef MOTOR4_INVERT
  #define MOTOR4_INVERT 0
#endif

// Minimal PWM agar motor mulai bergerak (deadband compensation).
// Set ke 0 untuk mematikan fitur ini.
#ifndef OMNI_MIN_PWM
  #define OMNI_MIN_PWM 0.0f
#endif

static inline float _absf(float x) { return x < 0 ? -x : x; }
static inline float _maybeInvert(float v, int inv) { return inv ? -v : v; }
static inline float _applyMinPwm(float v, float limit, float minPwm) {
  float a = _absf(v);
  if (a < 0.0001f) return 0.0f;
  if (minPwm <= 0.0f) return v;
  if (minPwm >= limit) return (v > 0.0f) ? limit : -limit;
  // Map 0..limit -> minPwm..limit (agar nilai kecil tidak "mati")
  float out = minPwm + (a / limit) * (limit - minPwm);
  if (out > limit) out = limit;
  return (v > 0.0f) ? out : -out;
}

// ============================================================
// YAW HEADING HOLD (PID) memakai yaw1 dari MPU (0..360 derajat)
// Output: koreksi w (satuan sama seperti input w pada moveRobot/moveRobotRpm)
// PID engine: computePID() (pid.ino)
// ============================================================

static float _yawTargetDeg = 0.0f;
static bool _yawTargetSet = false;
static unsigned long _yawPrevMs = 0;
static bool _yawInitialized = false;

static inline float _wrap360(float a) {
  while (a < 0.0f) a += 360.0f;
  while (a >= 360.0f) a -= 360.0f;
  return a;
}

// Error sudut terpendek dalam rentang [-180..180]
static inline float _angleErrorDeg(float targetDeg, float currentDeg) {
  float e = _wrap360(targetDeg) - _wrap360(currentDeg);
  if (e > 180.0f) e -= 360.0f;
  if (e < -180.0f) e += 360.0f;
  return e;
}

void resetYawHoldPid() {
  _yawPrevMs = millis();
  _yawInitialized = false;
  resetPidChannel((int)YAW_PID_CHANNEL);
}

void setYawTarget(float targetYawDeg) {
  _yawTargetDeg = _wrap360(targetYawDeg);
  _yawTargetSet = true;
  resetYawHoldPid();
}

void setYawTargetToCurrent() {
  setYawTarget(yaw1);
}

float computeYawPid(float targetYawDeg, float currentYawDeg) {
  // Samakan konvensi tanda yaw dengan world/global transform & odometry.
  // Jika yaw dari sensor arahnya kebalik (WORLD_YAW_INVERT=1), maka untuk PID juga
  // harus dibalik, supaya error sudut dan arah koreksi konsisten.
#if WORLD_YAW_INVERT
  targetYawDeg = -targetYawDeg;
  currentYawDeg = -currentYawDeg;
#endif

  unsigned long now = millis();
  float dt = 0.0f;
  if (_yawInitialized) {
    dt = (now - _yawPrevMs) / 1000.0f;
  }
  _yawPrevMs = now;

  float error = _angleErrorDeg(targetYawDeg, currentYawDeg);

  // Prime state agar derivative tidak spike
  if (!_yawInitialized) {
    _yawInitialized = true;
    primePidChannel((int)YAW_PID_CHANNEL, error);
    return 0.0f;
  }
  if (dt <= 0.0001f) {
    return 0.0f;
  }
  // Kalau loop sempat lambat (mis. karena Serial print), jangan matikan PID.
  // Cukup clamp dt supaya scaling Ki/Kd tetap stabil.
  if (dt > 0.25f) dt = 0.25f;

  // Scaling supaya ekuivalen dengan PID berbasis dt:
  // - computePID integral tidak dikali dt => Ki_eff = Ki*dt
  // - computePID derivative tidak dibagi dt => Kd_eff = Kd/dt
  const double kp = (double)YAW_PID_KP;
  const double ki_eff = (double)YAW_PID_KI * (double)dt;
  const double kd_eff = (double)YAW_PID_KD / (double)dt;

  // Anti-windup: limit integral (deg*s) -> sum(deg) dibatasi (I_LIMIT/dt)
  double maxI = (double)YAW_PID_I_LIMIT / (double)dt;

  double out = computePID(
    (int)YAW_PID_CHANNEL,
    (double)error,   // setpoint = error, input = 0
    0.0,
    kp,
    ki_eff,
    kd_eff,
    -maxI,
    +maxI
  );

  const double wLim = (double)YAW_PID_W_LIMIT;
  if (out > wLim) out = wLim;
  if (out < -wLim) out = -wLim;
#if YAW_PID_INVERT
  out = -out;
#endif
  return (float)out;
}

// Hitung kecepatan roda (w1,w2,w3) dari vx,vy,w.
// Output di sini adalah "wheel command" (bisa rad/s kalau r & L pakai meter).
void omni3Inverse(float vx, float vy, float w, float &w1, float &w2, float &w3) {
  const float r = (float)OMNI_WHEEL_RADIUS;
  const float L = (float)OMNI_ROBOT_RADIUS;

  // sin/cos 120° dan 240° (hemat trig)
  const float SQRT3_OVER_2 = 0.8660254037844386f; // √3/2

  // SWAP vx dan vy karena mapping hardware kebalik
  float vx_corrected = vy;
  float vy_corrected = vx;

  // θ1=0° => -sin=0, cos=1
  w1 = (vy_corrected + L * w) / r;
  // θ2=120° => -sin=-√3/2, cos=-1/2
  w2 = (-SQRT3_OVER_2 * vx_corrected - 0.5f * vy_corrected + L * w) / r;
  // θ3=240° => -sin=+√3/2, cos=-1/2
  w3 = ( SQRT3_OVER_2 * vx_corrected - 0.5f * vy_corrected + L * w) / r;
}

// Normalisasi 3 nilai agar |max| <= limit (biasanya limit = MAX_PWM atau speed)
static inline void normalize3(float &a, float &b, float &c, float limit) {
  float m = _absf(a);
  float t = _absf(b); if (t > m) m = t;
  t = _absf(c);       if (t > m) m = t;
  if (m > limit && m > 0.0001f) {
    float s = limit / m;
    a *= s; b *= s; c *= s;
  }
}

// ============================================================
// API sederhana untuk dipakai di loop()
// ============================================================
// Parameter mengikuti robot frame (setelah swap di omni3Inverse):
// - vx = right (+)
// - vy = forward (+)
// - w = CCW (+)
// speedLimit: batas maksimum output (misal 700 atau MAX_PWM atau RPM).
//
// Mapping roda -> motor:
// - w1 -> Motor 1
// - w2 -> Motor 3
// - w3 -> Motor 4
void moveRobot(float vx, float vy, float w, float speedLimit) {
  float w1, w2, w3;
  omni3Inverse(vx, vy, w, w1, w2, w3);

  // ============================================================
  // MODE 1: Closed-loop RPM (pakai rpmMotor)
  // - speedLimit dianggap batas RPM
  // - Tidak pakai OMNI_MIN_PWM (karena itu distorsi setpoint RPM)
  // ============================================================
#ifdef OMNI_USE_RPM_CONTROL
  normalize3(w1, w2, w3, speedLimit);

  // Invert arah motor pada SETPOINT RPM
  w1 = _maybeInvert(w1, MOTOR1_INVERT);
  w2 = _maybeInvert(w2, MOTOR3_INVERT);
  w3 = _maybeInvert(w3, MOTOR4_INVERT);

  // Sesuai komentar lama: tanda minus agar arah robot sama seperti mode PWM sebelumnya
  rpmMotor(-w1, -w2, -w3);
  return;
#endif

  // ============================================================
  // MODE 2: Open-loop PWM mixing (default lama)
  // - speedLimit dianggap batas PWM
  // ============================================================
  normalize3(w1, w2, w3, speedLimit);

  // pwmMotor(pwm1, pwm3, pwm4)
  w1 = _maybeInvert(w1, MOTOR1_INVERT);
  w2 = _maybeInvert(w2, MOTOR3_INVERT);
  w3 = _maybeInvert(w3, MOTOR4_INVERT);

  // Kompensasi deadband (jika OMNI_MIN_PWM > 0)
  w1 = _applyMinPwm(w1, speedLimit, (float)OMNI_MIN_PWM);
  w2 = _applyMinPwm(w2, speedLimit, (float)OMNI_MIN_PWM);
  w3 = _applyMinPwm(w3, speedLimit, (float)OMNI_MIN_PWM);

  pwmMotor(-w1, -w2, -w3);
}

// Convenience API: panggil langsung mode RPM tanpa define.
// Parameter mengikuti robot frame:
// - vx = right (+)
// - vy = forward (+)
// - w = CCW (+)
void moveRobotRpm(float vx, float vy, float w, float rpmLimit) {
  float w1, w2, w3;
  omni3Inverse(vx, vy, w, w1, w2, w3);
  normalize3(w1, w2, w3, rpmLimit);
  w1 = _maybeInvert(w1, MOTOR1_INVERT);
  w2 = _maybeInvert(w2, MOTOR3_INVERT);
  w3 = _maybeInvert(w3, MOTOR4_INVERT);
  
#if USE_PWM_IN_MOVE_ROBOT_RPM
  // Mode PWM: scaling w1, w2, w3 dengan factor (default 10x)
  w1 *= (float)PWM_SCALING_FACTOR;
  w2 *= (float)PWM_SCALING_FACTOR;
  w3 *= (float)PWM_SCALING_FACTOR;
  pwmMotor(-w1, -w2, -w3);
#else
  // Mode RPM: closed-loop control (default)
  rpmMotor(-w1, -w2, -w3);
#endif
}

// RPM + yaw hold:
// - targetYawDeg: heading target (0..360)
// - rpmLimit: batas RPM roda (speed limit)
void moveRobotRpmYawHold(float vx, float vy, float targetYawDeg, float rpmLimit) {
  float wCorr = computeYawPid(targetYawDeg, yaw1);
  moveRobotRpm(vx, vy, wCorr, rpmLimit);
}

// Versi yang pakai target internal:
// - Panggil setYawTargetToCurrent() sekali saat mulai, atau biarkan fungsi ini set otomatis.
void moveRobotRpmYawHold(float vx, float vy, float rpmLimit) {
  if (!_yawTargetSet) setYawTargetToCurrent();
  moveRobotRpmYawHold(vx, vy, _yawTargetDeg, rpmLimit);
}

// ============================================================
// GLOBAL / WORLD FRAME HELPERS
// ============================================================
// Konvensi GLOBAL (konsisten dengan odometry):
// - xRight (+)  : kanan
// - yForward (+): maju
//
// Konvensi LOKAL untuk moveRobotRpm (setelah swap di omni3Inverse):
// - parameter 1 (vx) = right (+)
// - parameter 2 (vy) = forward (+)
//
// Transform global -> local (yaw positif CCW):
// vx_local (right)   =  cos(yaw) * xRight_global   + sin(yaw) * yForward_global
// vy_local (forward) = -sin(yaw) * xRight_global   + cos(yaw) * yForward_global

static inline float _degToRad_k(float d) { return d * 0.01745329251994329577f; }

void moveRobotRpmGlobal(float xRightGlobal, float yForwardGlobal, float w, float rpmLimit) {
  float yawDeg = yaw1;
#if WORLD_YAW_INVERT
  yawDeg = -yawDeg;
#endif
  float yawRad = _degToRad_k(yawDeg);
  float c = cosf(yawRad);
  float s = sinf(yawRad);
  float vx_local =  c * xRightGlobal + s * yForwardGlobal;
  float vy_local = -s * xRightGlobal + c * yForwardGlobal;
  moveRobotRpm(vx_local, vy_local, w, rpmLimit);
}

void moveRobotRpmYawHoldGlobal(float xRightGlobal, float yForwardGlobal, float targetYawDeg, float rpmLimit) {
  float yawDeg = yaw1;
#if WORLD_YAW_INVERT
  yawDeg = -yawDeg;
#endif
  float yawRad = _degToRad_k(yawDeg);
  float c = cosf(yawRad);
  float s = sinf(yawRad);
  float vx_local =  c * xRightGlobal + s * yForwardGlobal;
  float vy_local = -s * xRightGlobal + c * yForwardGlobal;
  moveRobotRpmYawHold(vx_local, vy_local, targetYawDeg, rpmLimit);
}

// Versi dengan limit koreksi yaw runtime (batasYaw)
void moveRobotRpmYawHoldGlobal(float xRightGlobal, float yForwardGlobal, float targetYawDeg, float rpmLimit, float batasYaw) {
  float yawDeg = yaw1;
#if WORLD_YAW_INVERT
  yawDeg = -yawDeg;
#endif
  float yawRad = _degToRad_k(yawDeg);
  float c = cosf(yawRad);
  float s = sinf(yawRad);
  float vx_local =  c * xRightGlobal + s * yForwardGlobal;
  float vy_local = -s * xRightGlobal + c * yForwardGlobal;

  float wCorr = computeYawPid(targetYawDeg, yaw1);
  if (wCorr > batasYaw) wCorr = batasYaw;
  if (wCorr < -batasYaw) wCorr = -batasYaw;
  moveRobotRpm(vx_local, vy_local, wCorr, rpmLimit);
}

void moveRobotRpmYawHoldGlobal(float xRightGlobal, float yForwardGlobal, float rpmLimit) {
  if (!_yawTargetSet) setYawTargetToCurrent();
  moveRobotRpmYawHoldGlobal(xRightGlobal, yForwardGlobal, _yawTargetDeg, rpmLimit);
}

// Overload: default limit = MAX_PWM
void moveRobot(float vx, float vy, float w) {
  moveRobot(vx, vy, w, (float)MAX_PWM);
}