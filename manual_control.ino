int jarX, jarY;

void manual_control() {
  jarX = x_cm;
  jarY = y_cm;
  int jarX1, jarY1;
  //   if(Share){setMode = 0;}
  // if(Options){setMode = 1;}

  // if (up) { yL = 127; }
  // if (dw) { yL = -127; }
  // if (rg) { xL = 127; }
  // if (lf) { xL = -127; }
  if (R1 == 1) {
    modeSpeed = 2;
  } else if (L1 == 1) {
    modeSpeed = 0;
  } else {
    modeSpeed = 1;
  }

  yL = constrain(yL, -126, 126);
  xL = constrain(xL, -126, 126);
  yR = constrain(yR, -126, 126);
  xR = constrain(xR, -126, 126);

  // Quick angle set dengan L1 + D-Pad
  if (L2 == 1 && up == 1) {
    rpmW = 0;    // L2 + UP = 0°
  } else if (L2 == 1 && rg == 1) {
    rpmW = 90;   // L2 + RIGHT = 90°
  } else if (L2 == 1 && dw == 1) {
    rpmW = 180;  // L2 + DOWN = 180°
  } else if (L2 == 1 && lf == 1) {
    rpmW = 270;  // L2 + LEFT = 270°
  }

  // if (yawps4 >= 315 || yawps4 <= 45) {
  //   yawps41 = 0;
  // } else if (yawps4 >= 45 && yawps4 <= 135) {
  //   yawps41 = 90;
  // } else if (yawps4 >= 135 && yawps4 <= 225) {
  //   yawps41 = 180;
  // } else if (yawps4 >= 225 && yawps4 <= 315) {
  //   yawps41 = 270;
  // }

  //xL1 = (cos(radians(yawps41)) * xL + sin(radians(yawps41)) * yL);
  //yL1 = ((-sin(radians(yawps41))) * xL + cos(radians(yawps41)) * yL);

  SPEED = EE.SPEED[modeSpeed];
  SPEEDR = EE.SPEEDR[modeSpeed];

  //rpmY = map(yL, -127, 127, -SPEED, SPEED);
  //rpmX = map(xL, -127, 127, -SPEED, SPEED);
  //rpmW = map(xR, -127, 127, -SPEEDR, SPEEDR);
  //if (xL > 0 || xL < 0) { jarX1 = jarX; }
  //if (yL > 0 || yL < 0) { jarY1 = jarY; }
  if (up && L2 == 0) {
    yL = 127;
  } else if (dw && L2 == 0) {
    yL = -127;
  } else if (lf && L2 == 0) {
    xL = -127;
  } else if (rg && L2 == 0) {
    xL = 127;
  } else if (up && rg && L2 == 0) {
    yL = 65;
    xL = 65;
  } else if (up && lf && L2 == 0) {
    yL = 65;
    xL = -65;
  } else if (dw && rg && L2 == 0) {
    yL = -65;
    xL = 65;
  } else if (dw && lf && L2 == 0) {
    yL = -65;
    xL = -65;
  }

  //  if (xL > 0 || xL < 0) {
  //   int XL;
  //   if (xL<0){XL=-1;}else{XL=1;}
  //   rpmX=SPEED * XL;
  //   // rpmX = jarX + xL * EE.KP[modeSpeed];
  // }else if(xL=0)rpmX=0;
  // if (yL > 0 || yL < 0) {
  //   int YL;
  //   if (yL<0){YL=-1;}else{YL=1;}
  //   rpmY=SPEED * YL;
  //   // rpmY = jarY + yL * EE.KP[modeSpeed];
  // } else if(yL=0)rpmY=0;
  // //rpmX = jarX + xL1 * EE.KP[modeSpeed];
  // //rpmY = jarY + yL1 * EE.KP[modeSpeed];
  if (xL > 0 && yL > 0 ){
    xL = 63;
    yL = 63;
  } else if (xL < 0 && yL < 0 ){
    xL = -63;
    yL = -63;
  } else if (xL > 0 && yL < 0 ){
    xL = 63;
    yL = -63;
  }else if (xL < 0 && yL > 0 ){
    xL = -63;
    yL = 63;
  }
  if (xL > 0 || xL < 0) {
    rpmX = jarX + xL * EE.KP[modeSpeed];
  } 
  if (yL > 0 || yL < 0) {
    rpmY = jarY + yL * EE.KP[modeSpeed];
  }
  //rpmX = jarX + xL1 * EE.KP[modeSpeed];
  //rpmY = jarY + yL1 * EE.KP[modeSpeed];

  if (xR > 0 || xR < 0) {
    rpmW += xR * EE.KA[modeSpeed];
  }

  rpmW = normalizeAngle(rpmW, 0, 360);
  
  pidKinematik(rpmX * 0.01, rpmY * 0.01, rpmW, SPEED, SPEEDR);
}

// Manual control versi 2:
// - Translasi pakai GLOBAL frame (field-centric) via moveRobotRpmGlobal()
// - Kanan  (+) = xRightGlobal, Maju (+) = yForwardGlobal
// - Rotasi pakai stick kanan X (xR) sebagai rate (bukan target sudut)
//
// Catatan:
// - Fungsi ini TIDAK memakai pidKinematik (tidak ngejar posisi), jadi feel-nya seperti joystick drive biasa.
// - Kalau arah global terasa kebalik saat yaw berubah, cek `WORLD_YAW_INVERT` di `config.h`.
static inline float _mapf(float x, float inMin, float inMax, float outMin, float outMax) {
  if (inMax - inMin == 0.0f) return outMin;
  float t = (x - inMin) / (inMax - inMin);
  if (t < 0.0f) t = 0.0f;
  if (t > 1.0f) t = 1.0f;
  return outMin + t * (outMax - outMin);
}

void manual_control2() {
  static bool yawHoldInit = false;
  static float yawHoldTarget = 0.0f;

  // Mode speed sama seperti manual_control()
  if (R1 == 1) {
    modeSpeed = 2;
  } else if (L1 == 1) {
    modeSpeed = 0;
  } else {
    modeSpeed = 1;
  }

  yL = constrain(yL, -126, 126);
  xL = constrain(xL, -126, 126);
  yR = constrain(yR, -126, 126);
  xR = constrain(xR, -126, 126);

  // D-Pad drive (tanpa L2) - sama seperti versi lama
  if (up && L2 == 0) {
    yL = 127;
  } else if (dw && L2 == 0) {
    yL = -127;
  } else if (lf && L2 == 0) {
    xL = -127;
  } else if (rg && L2 == 0) {
    xL = 127;
  } else if (up && rg && L2 == 0) {
    yL = 65;
    xL = 65;
  } else if (up && lf && L2 == 0) {
    yL = 65;
    xL = -65;
  } else if (dw && rg && L2 == 0) {
    yL = -65;
    xL = 65;
  } else if (dw && lf && L2 == 0) {
    yL = -65;
    xL = -65;
  }

  // Diagonal clamp biar feel konsisten (meniru versi lama)
  if (xL > 0 && yL > 0) {
    xL = 63; yL = 63;
  } else if (xL < 0 && yL < 0) {
    xL = -63; yL = -63;
  } else if (xL > 0 && yL < 0) {
    xL = 63; yL = -63;
  } else if (xL < 0 && yL > 0) {
    xL = -63; yL = 63;
  }

  // Pakai parameter "manual global" yang sudah ada di config.h (lebih responsif dan konsisten)
  float xyLimit = (float)MANUAL_XY_MED;
  float rpmLimit = (float)MANUAL_RPM_LIMIT_MED;
  float yawLimit = (float)MANUAL_YAW_LIMIT;
  if (modeSpeed == 0) {
    xyLimit = (float)MANUAL_XY_SLOW;
    rpmLimit = (float)MANUAL_RPM_LIMIT_SLOW;
  } else if (modeSpeed == 2) {
    xyLimit = (float)MANUAL_XY_FAST;
    rpmLimit = (float)MANUAL_RPM_LIMIT_FAST;
  }
  SPEED = (int)xyLimit;   // biar LCD/debug lama masih masuk akal
  SPEEDR = (int)yawLimit; // batas koreksi yaw (dipakai yaw-hold)

  // Deadzone kecil supaya tidak drift
  const int deadXY = 8;
  float xCmd = (abs(xL) <= deadXY) ? 0.0f : _mapf((float)xL, -127.0f, 127.0f, -xyLimit, xyLimit);
  float yCmd = (abs(yL) <= deadXY) ? 0.0f : _mapf((float)yL, -127.0f, 127.0f, -xyLimit, xyLimit);

  // Rotasi dari stick kanan X (rate). Kalau tidak ada input rotasi -> yaw-hold aktif.
  bool rotActive = (abs(xR) > (int)MANUAL_RX_DEADZONE);
  float wCmd = 0.0f;
  if (rotActive) {
    // Rate command (skala RPM-mix), user sudah membalik tanda di panggilan moveRobotRpmGlobal()
    wCmd = _mapf((float)xR, -127.0f, 127.0f, -yawLimit, yawLimit);
    // Saat user sedang muter, target yaw-hold mengikuti heading sekarang agar pas dilepas langsung "hold"
    yawHoldTarget = yaw1;
    yawHoldInit = true;
  } else {
    // Kalau baru masuk mode hold, kunci target yaw ke heading saat ini
    if (!yawHoldInit) {
      yawHoldTarget = yaw1;
      yawHoldInit = true;
    }
  }

  // Simpan buat LCD/debug (kalau dipakai)
  rpmX = (int)xCmd;
  rpmY = (int)yCmd;
  rpmW = (int)wCmd;
  moveRobotRpmYawHoldGlobal(xCmd, yCmd, rpmW, rpmLimit, yawLimit);
//   // Stop benar-benar diam kalau translasi & rotasi nol (hindari "drift" karena minPWM / noise)
//   if (!rotActive && (fabsf(xCmd) < (float)MANUAL_STOP_DEADBAND_RPM) && (fabsf(yCmd) < (float)MANUAL_STOP_DEADBAND_RPM)) {
// #if MANUAL_HOLD_YAW_WHEN_STOP
//     // Tetap hold yaw saat stop (biasanya enak buat aiming)
//     moveRobotRpmYawHoldGlobal(0.0f, 0.0f, yawHoldTarget, rpmLimit, (float)MANUAL_STOP_YAW_LIMIT);
//     #else
//     moveRobotRpmYawHoldGlobal(0.0f, 0.0f, yawHoldTarget, rpmLimit, (float)MANUAL_STOP_YAW_LIMIT);
//     // moveRobotRpmGlobal(0.0f, 0.0f, 0.0f, rpmLimit);
//     yawHoldInit = false;
// #endif
//     return;
//   }

//   if (rotActive) {
//     // Manual rotasi langsung
//     moveRobotRpmYawHoldGlobal(xCmd, yCmd, yawHoldTarget, rpmLimit, yawLimit);
//     // moveRobotRpmGlobal(xCmd, yCmd, -wCmd, rpmLimit);
//   } else {
//     // Yaw-hold otomatis saat translasi (ini yang mencegah strafe bikin sudut berubah)
//     moveRobotRpmYawHoldGlobal(xCmd, yCmd, yawHoldTarget, rpmLimit, yawLimit);
//   }
}