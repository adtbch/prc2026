#include "config.h"
#include <math.h>

// ============================================================
// ODOMETRY SEDERHANA (untuk omni 3 roda: Motor 1,3,4)
// Output:
// - odomX: kanan  (+)
// - odomY: maju   (+)
//
// Catatan:
// - Agar jadi satuan meter, set OMNI_WHEEL_RADIUS di config.h (meter) dan PPR benar.
// - Kalau OMNI_WHEEL_RADIUS belum diset (default 1), maka unit odom bukan meter asli.
// ============================================================

static float _odomX = 0.0f; // meter
static float _odomY = 0.0f; // meter

// Untuk LCD/debug (cm)
float x_cm = 0.0f;
float y_cm = 0.0f;

static long _lastC1 = 0;
static long _lastC3 = 0;
static long _lastC4 = 0;

static unsigned long _lastMs = 0;
static bool _initialized = false;

static long _lastD3 = 0; // delta count terakhir (untuk LCD)
static long _lastD4 = 0;

static inline long _readEnc(volatile long &v) {
  noInterrupts();
  long x = v;
  interrupts();
  return x;
}

static inline float _degToRad(float d) { return d * 0.01745329251994329577f; }

void resetOdometry() {
  _odomX = 0.0f;
  _odomY = 0.0f;
  _lastD3 = 0;
  _lastD4 = 0;

  _lastC1 = _readEnc(encoder1Count);
  _lastC3 = _readEnc(encoder3Count);
  _lastC4 = _readEnc(encoder4Count);
  _lastMs = millis();
  _initialized = true;
}

void setupOdometry() {
  resetOdometry();
}

void updateOdometry() {
  unsigned long now = millis();
  if (!_initialized) {
    resetOdometry();
    return;
  }
  if ((now - _lastMs) < (unsigned long)ODOM_INTERVAL_MS) return;
  _lastMs = now;

  // Baca counts
  long c1 = _readEnc(encoder1Count);
  long c3 = _readEnc(encoder3Count);
  long c4 = _readEnc(encoder4Count);

  long dc1 = c1 - _lastC1;
  long dc3 = c3 - _lastC3;
  long dc4 = c4 - _lastC4;
  _lastC1 = c1;
  _lastC3 = c3;
  _lastC4 = c4;

  _lastD3 = dc3;
  _lastD4 = dc4;

  // Konversi count -> radian roda
  // dTheta = (deltaCount / PPR) * 2*pi
  const float TWO_PI_F = 6.2831853071795864769f;
  const float dTheta1 = ((float)dc1 / (float)PPR) * TWO_PI_F; // wheel 1 -> Motor 1
  const float dTheta2 = ((float)dc3 / (float)PPR) * TWO_PI_F; // wheel 2 -> Motor 3
  const float dTheta3 = ((float)dc4 / (float)PPR) * TWO_PI_F; // wheel 3 -> Motor 4

  // Wheel radius (meter). Default OMNI_WHEEL_RADIUS = 1 kalau belum diset.
  const float R = (float)OMNI_WHEEL_RADIUS;

  // Forward kinematics untuk displacement:
  // dX_forward = (R/sqrt(3)) * (dTheta3 - dTheta2)
  // dY_left    = (R/3)       * (2*dTheta1 - dTheta2 - dTheta3)
  const float INV_SQRT3 = 0.5773502691896257645f; // 1/sqrt(3)
  float dX_forward = R * INV_SQRT3 * (dTheta3 - dTheta2);
  float dY_left    = (R / 3.0f)    * (2.0f * dTheta1 - dTheta2 - dTheta3);

  // Ubah ke konvensi user: X kanan (+), Y maju (+)
  float dx = -dY_left;
  float dy =  dX_forward;

#ifdef ODOM_USE_IMU_YAW
  // Rotasi ke world frame pakai yaw1 (derajat). Asumsi yaw positif CCW.
  float yawDeg = yaw1;
#if WORLD_YAW_INVERT
  yawDeg = -yawDeg;
#endif
  float yawRad = _degToRad(yawDeg);
  float c = cosf(yawRad);
  float s = sinf(yawRad);
  // Robot frame (x kanan, y maju) -> world frame:
  // [Xw] = [ c -s ] [x]
  // [Yw]   [ s  c ] [y]
  float dxw = c * dx - s * dy;
  float dyw = s * dx + c * dy;
  dx = dxw;
  dy = dyw;
#endif

  // Invert koordinat di frame FINAL (setelah rotasi world-frame jika ODOM_USE_IMU_YAW aktif)
#if ODOM_INVERT_X
  dx = -dx;
#endif
#if ODOM_INVERT_Y
  dy = -dy;
#endif

  _odomX += dx;
  _odomY += dy;
  x_cm = _odomX * 100.0f;
  y_cm = _odomY * 100.0f;
}

float getOdomX() { return _odomX; }
float getOdomY() { return _odomY; }
long getOdomDeltaM3() { return _lastD3; }
long getOdomDeltaM4() { return _lastD4; }


