#include "config.h"
#include <math.h>

// Variabel global untuk command PID Kinematik (bisa dibaca dari luar untuk debug/LCD)
float cmdX = 0.0f;
float cmdY = 0.0f;
float cmdW = 0.0f;

// Low pass filter sederhana
static inline float lowPassFilter(float input, float prev, float alpha) {
  if (alpha <= 0.0f) return prev;
  if (alpha >= 1.0f) return input;
  return prev + alpha * (input - prev);
}

static inline float _wrap360_k(float a) {
  while (a < 0.0f) a += 360.0f;
  while (a >= 360.0f) a -= 360.0f;
  return a;
}

static inline float _angleErrorDeg_k(float targetDeg, float currentDeg) {
  float e = _wrap360_k(targetDeg) - _wrap360_k(currentDeg);
  if (e > 180.0f) e -= 360.0f;
  if (e < -180.0f) e += 360.0f;
  return e;
}

static void _pidKinematikImpl(float targetX_m, float targetY_m, float targetYawDeg,
                              float batasXY, float batasYaw, float rpmLimit) {
  static float filteredX = 0.0f;
  static float filteredY = 0.0f;

  // Filter setpoint (biar tidak "nyentak")
  filteredX = lowPassFilter(targetX_m, filteredX, (float)PIDKIN_ALPHA);
  filteredY = lowPassFilter(targetY_m, filteredY, (float)PIDKIN_ALPHA);

  // Feedback posisi dari odometry
  const float posX = getOdomX(); // kanan +
  const float posY = getOdomY(); // maju  +

  // PID X/Y (posisi -> command vx/vy) - menggunakan variabel global
  cmdX = 0.0f;
  cmdY = 0.0f;

  // Deadband posisi
  float ex = filteredX - posX;
  float ey = filteredY - posY;

  if (fabsf(ex) < (float)PIDKIN_XY_DEADBAND_M) {
    resetPidChannel(4);
    cmdX = 0.0f;
  } else {
    // Integral clamp (kalau Ki=0, clamp=0 agar aman)
    double minI = 0.0, maxI = 0.0;
    if ((double)PIDKIN_XY_KI != 0.0) {
      maxI = (double)batasXY / (double)PIDKIN_XY_KI;
      minI = -maxI;
    }
    double out = computePID(4, (double)filteredX, (double)posX,
                            (double)PIDKIN_XY_KP, (double)PIDKIN_XY_KI, (double)PIDKIN_XY_KD,
                            minI, maxI);
    if (out > (double)batasXY) out = (double)batasXY;
    if (out < -(double)batasXY) out = -(double)batasXY;
    cmdX = (float)out;
  }

  if (fabsf(ey) < (float)PIDKIN_XY_DEADBAND_M) {
    resetPidChannel(5);
    cmdY = 0.0f;
  } else {
    double minI = 0.0, maxI = 0.0;
    if ((double)PIDKIN_XY_KI != 0.0) {
      maxI = (double)batasXY / (double)PIDKIN_XY_KI;
      minI = -maxI;
    }
    double out = computePID(5, (double)filteredY, (double)posY,
                            (double)PIDKIN_XY_KP, (double)PIDKIN_XY_KI, (double)PIDKIN_XY_KD,
                            minI, maxI);
    if (out > (double)batasXY) out = (double)batasXY;
    if (out < -(double)batasXY) out = -(double)batasXY;
    cmdY = (float)out;
  }

  // Jalankan robot:
  // - X/Y dari PID posisi
  // - yaw PID dihitung di sini (mirip contoh kamu), lalu dipassing sebagai w ke moveRobotRpmGlobal
  float yawErr = _angleErrorDeg_k(targetYawDeg, yaw1); // [-180..180]
  cmdW = 0.0f;  // Reset global cmdW

  // Deadband yaw: kalau |error| <= deadband, anggap 0
  if (fabsf(yawErr) <= (float)PIDKIN_YAW_DEADBAND_DEG) {
    resetPidChannel((int)PIDKIN_YAW_CHANNEL);
    cmdW = 0.0f;
  } else {
    // Adaptive Kp: error kecil pakai Kp lebih besar, error besar dikurangi
    float absErr = fabsf(yawErr);
    float adaptiveKp = (float)PIDKIN_YAW_KP;
    
    // Scaling: mulai kurangi Kp dari error menengah (default 15°)
    // Tujuan: error kecil (mis. 5-12°) tetap responsif, tapi error > 15° tidak "kencang banget".
    const float s0 = (float)PIDKIN_YAW_KP_SCALE_START_DEG;
    const float s1 = (float)PIDKIN_YAW_KP_SCALE_END_DEG;
    const float minScale = (float)PIDKIN_YAW_KP_MIN_SCALE;
    if (absErr > s0) {
      float denom = (s1 - s0);
      if (denom < 0.001f) denom = 0.001f;
      float t = (absErr - s0) / denom; // 0..1
      if (t > 1.0f) t = 1.0f;
      float scale = 1.0f - t * (1.0f - minScale);
      if (scale < minScale) scale = minScale;
      adaptiveKp *= scale;
    }
    
    // Integral clamp aman (hindari div0 saat Ki=0)
    double minI = 0.0, maxI = 0.0;
    if ((double)PIDKIN_YAW_KI != 0.0) {
      maxI = (double)batasYaw / (double)PIDKIN_YAW_KI;
      minI = -maxI;
    }
    
    // computePID dengan adaptive Kp
    double out = computePID((int)PIDKIN_YAW_CHANNEL, (double)yawErr, 0.0,
                            (double)adaptiveKp, (double)PIDKIN_YAW_KI, (double)PIDKIN_YAW_KD,
                            minI, maxI);
    
    // Clamp output yaw
    if (out > (double)batasYaw) out = (double)batasYaw;
    if (out < -(double)batasYaw) out = -(double)batasYaw;
    cmdW = (float)out;
    
    // Minimum output untuk error kecil (boost response) - dibuat HALUS supaya tidak "sedat"
    // dekat boundary 0/360 karena noise + deadband.
    const float minOut = (float)PIDKIN_YAW_MIN_OUT;
    const float db = (float)PIDKIN_YAW_DEADBAND_DEG;
    const float maxErr = (float)PIDKIN_YAW_MIN_OUT_MAX_ERR_DEG;
    if (minOut > 0.0f && absErr > db && absErr < maxErr) {
      float denom = (maxErr - db);
      if (denom < 0.001f) denom = 0.001f;
      // minCmd naik 0 -> minOut saat error db -> maxErr
      float t = (absErr - db) / denom;
      if (t < 0.0f) t = 0.0f;
      if (t > 1.0f) t = 1.0f;
      float minCmd = minOut * t;
      if (fabsf(cmdW) < minCmd) {
        cmdW = (yawErr > 0.0f) ? minCmd : -minCmd;
      }
    }
  }

#if PIDKIN_USE_GLOBAL
  // Selalu pakai moveRobotRpmGlobal, karena yaw sudah dihitung sebagai cmdW
  // cmdX = command kanan (+), cmdY = command maju (+), cmdW = command rotasi CCW (+)
  moveRobotRpmGlobal(cmdX, cmdY, -cmdW, rpmLimit);
#else
  // Mode lokal: langsung passing cmdX dan cmdY (sudah sesuai konvensi setelah swap)
  moveRobotRpm(cmdX, cmdY, cmdW, rpmLimit);
#endif
}

void pidKinematik(float targetX_m, float targetY_m, float targetYawDeg,
                  float batasXY, float batasYaw) {
  _pidKinematikImpl(targetX_m, targetY_m, targetYawDeg, batasXY, batasYaw, batasXY);
}

void pidKinematikLimit(float targetX_m, float targetY_m, float targetYawDeg,
                       float batasXY, float batasYaw, float rpmLimit) {
  _pidKinematikImpl(targetX_m, targetY_m, targetYawDeg, batasXY, batasYaw, rpmLimit);
}


