#include "config.h"
#include <math.h>

static inline float _wrap360_wp(float a) {
  while (a < 0.0f) a += 360.0f;
  while (a >= 360.0f) a -= 360.0f;
  return a;
}

// shortest angle error in [-180..180]
static inline float _angleErrorDeg_wp(float targetDeg, float currentDeg) {
  float e = _wrap360_wp(targetDeg) - _wrap360_wp(currentDeg);
  if (e > 180.0f) e -= 360.0f;
  if (e < -180.0f) e += 360.0f;
  return e;
}

// Active waypoint state (untuk LCD/debug)
static bool _wpActive = false;
static int _wpIdx = -1;
static float _wpX = 0.0f;
static float _wpY = 0.0f;
static float _wpAng = 0.0f;

bool hasActiveWaypoint() { return _wpActive; }
int getActiveWaypointIndex() { return _wpIdx; }
float getActiveWaypointX() { return _wpX; }
float getActiveWaypointY() { return _wpY; }
float getActiveWaypointAngleDeg() { return _wpAng; }

// targetX/targetY default mm (kalau WAYPOINT_UNITS_MM=1), otherwise meter
bool executeWaypoint(float targetX, float targetY, float targetAngle,
                     float threshold, float angleThreshold,
                     int targetpwm, int targetyawpwm) {
  _wpActive = true;
  _wpIdx = -1;
  _wpX = targetX;
  _wpY = targetY;
  _wpAng = targetAngle;

  // Posisi sekarang dari odom (meter)
  float x_m = getOdomX();
  float y_m = getOdomY();

#if WAYPOINT_UNITS_CM
  // Convert target(cm) -> meter
  float tx_m = targetX / 100.0f;
  float ty_m = targetY / 100.0f;
  float dx_m = tx_m - x_m;
  float dy_m = ty_m - y_m;
  float dist_units = sqrtf(dx_m * dx_m + dy_m * dy_m) * 100.0f; // meter -> cm
  float thr_units = threshold;
#elif WAYPOINT_UNITS_MM
  // Convert target(mm) -> meter
  float tx_m = targetX / 1000.0f;
  float ty_m = targetY / 1000.0f;
  float dx_m = tx_m - x_m;
  float dy_m = ty_m - y_m;
  float dist_units = sqrtf(dx_m * dx_m + dy_m * dy_m) * 1000.0f; // meter -> mm
  float thr_units = threshold;
#else
  float tx_m = targetX;
  float ty_m = targetY;
  float dx_m = tx_m - x_m;
  float dy_m = ty_m - y_m;
  float dist_units = sqrtf(dx_m * dx_m + dy_m * dy_m); // meter
  float thr_units = threshold;
#endif

  float dAngle = _angleErrorDeg_wp(targetAngle, yaw1);

  // Jalankan kontrol (posisi -> command RPM) pakai pidKinematik kita
  // batasXY/batasYaw dipakai sebagai limit command, bukan threshold.
  pidKinematik(tx_m, ty_m, targetAngle, (float)targetpwm, (float)targetyawpwm);

  // Syarat tercapai
  if (dist_units < thr_units && fabsf(dAngle) < angleThreshold) {
    // Stop motor saat sudah sampai
    rpmMotor(0, 0, 0);
    return true;
  }
  return false;
}

bool executeWaypoint(const Waypoint &wp) {
  return executeWaypoint(wp.x, wp.y, wp.angleDeg, wp.threshold, wp.angleThreshold, wp.batasXY, wp.batasYaw);
}

// ====== Waypoint list contoh (sesuai format yang kamu kirim) ======
// Catatan: x/y/threshold di sini dalam mm (karena WAYPOINT_UNITS_MM=1).
const Waypoint waypointTarget[] = {
  // Sumbu odom: X kanan (+), Y maju (+)
  // Unit: mm (WAYPOINT_UNITS_MM=1). "10" aku anggap 10 cm => 100 mm.
  {   0, 1000,   0, 10, 10, 200, 30 },  // 0: maju 10 cm, hadap 0
  { 1000, 1000, 0, 10, 10, 200, 30 },  // 1: kanan 10 cm (kumulatif), hadap 180
  { 1000,   0, 0, 10, 10, 200, 30 },  // 2: belakang 10 cm (kumulatif), hadap 270
  {   0,   0,   0, 10, 10, 200, 30 },  // 3: kiri 10 cm (kumulatif), hadap 0 (balik start)
};

const int numWaypoints = (int)(sizeof(waypointTarget) / sizeof(waypointTarget[0]));

// Runner sederhana:
// - panggil ini terus di loop()
// - akan eksekusi waypoint 0..N-1
// - return true kalau SEMUA waypoint selesai
bool runWaypoints() {
  static int idx = 0;
  if (idx >= numWaypoints) {
    _wpActive = false;
    _wpIdx = -1;
    rpmMotor(0, 0, 0);
    return true;
  }
  _wpActive = true;
  _wpIdx = idx;
  _wpX = waypointTarget[idx].x;
  _wpY = waypointTarget[idx].y;
  _wpAng = waypointTarget[idx].angleDeg;

  if (executeWaypoint(waypointTarget[idx])) {
    idx++;
#if WAYPOINT_RELATIVE
    // Mode RELATIF: waypoint berikutnya dihitung dari posisi sekarang
    resetOdometry();
#endif
  }
  return false;
}


