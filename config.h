// Prevent multiple inclusion (Arduino menggabungkan banyak .ino jadi 1 TU)
#pragma once

// ====== PIN CONFIG ======
// Modul L298N 1
#define EN_MOTOR_1 23
#define EN_MOTOR_2 5 //5
#define MOTOR1_A   1 // 1
#define MOTOR1_B   3
#define MOTOR2_A   0 //19
#define MOTOR2_B   2 //18

// Modul L298N 2
#define EN_MOTOR_3 17
#define EN_MOTOR_4 5 //15
#define MOTOR3_A   16
#define MOTOR3_B   4
#define MOTOR4_A   19 //0
#define MOTOR4_B   18 //2

// ====== PWM CONFIG ======
#define MAX_PWM 1023
#define PWM_FREQ 15000   // 15 kHz - optimal untuk L298N (lebih halus & efisien)
#define PWM_RES  10      // 10-bit resolution: 0..1023

// Channel PWM untuk tiap EN (bebas 0..15)
#define M1_PWM_CHANNEL 0
#define M2_PWM_CHANNEL 1
#define M3_PWM_CHANNEL 2
#define M4_PWM_CHANNEL 3

// Encoder
#define ENC_MOTOR1_A 34
#define ENC_MOTOR1_B 35
#define ENC_MOTOR2_A 25//36
#define ENC_MOTOR2_B 26//39
#define ENC_MOTOR3_A 27
#define ENC_MOTOR3_B 14
#define ENC_MOTOR4_A 33
#define ENC_MOTOR4_B 32

// Deklarasi variabel encoder (definisi ada di encoder.ino)
// Counter dengan arah (positif/negatif)
extern volatile long encoder1Count;
extern volatile long encoder2Count;
extern volatile long encoder3Count;
extern volatile long encoder4Count;

// Counter absolut untuk RPM (selalu positif)
extern volatile unsigned long encoder1RPM;
extern volatile unsigned long encoder2RPM;
extern volatile unsigned long encoder3RPM;
extern volatile unsigned long encoder4RPM;

// ====== RPM CONFIG ======
#define PPR 210              // Pulse Per Rotation (sesuaikan dengan encoder kalian)
#define INTERVAL_RPM 100    // Interval pembacaan RPM dalam ms (100ms = 0.1 detik)

// Deklarasi fungsi RPM (definisi ada di encoder.ino)
float getRPM_Motor1();
float getRPM_Motor2();
float getRPM_Motor3();
float getRPM_Motor4();

// Kontrol RPM untuk omni 3 roda (definisi ada di pid.ino)
void rpmMotor(float rpm_w1, float rpm_w2, float rpm_w3);

// PID helper (definisi ada di pid.ino)
double computePID(int index, double setpoint, double input,
                  double Kp, double Ki, double Kd,
                  double Minintegral, double Maxintegral);
void resetPidChannel(int idx);
void primePidChannel(int idx, float error);

// Channel PID yang dipakai untuk yaw hold (jangan bentrok dengan RPM wheel PID 0..2)
#ifndef YAW_PID_CHANNEL
  #define YAW_PID_CHANNEL 3
#endif

// ====== ODOMETRY (X kanan +, Y maju +) dari encoderCount ======
void setupOdometry();
void updateOdometry();
void resetOdometry();
float getOdomX(); // meter (kanan +)
float getOdomY(); // meter (maju  +)
long getOdomDeltaM3(); // delta count terakhir (M3)
long getOdomDeltaM4(); // delta count terakhir (M4)

// Interval update odometry (ms)
#ifndef ODOM_INTERVAL_MS
  #define ODOM_INTERVAL_MS 20
#endif

// Invert tanda koordinat odom kalau arah kebalik
// - Y: maju (+)  => kalau maju jadi minus, set 1
// - X: kanan (+) => kalau kanan jadi minus, set 1
#ifndef ODOM_INVERT_Y
  #define ODOM_INVERT_Y 1
#endif
#ifndef ODOM_INVERT_X
  #define ODOM_INVERT_X 0
#endif

// Jika mau odom world-frame pakai yaw1:
#define ODOM_USE_IMU_YAW 1

// ====== WORLD/GLOBAL FRAME YAW SIGN ======
// Beberapa setup MPU menghasilkan yaw yang arahnya kebalik dari konvensi matematika (CCW +).
// Jika perintah GLOBAL terasa kebalik (mis. saat yaw bertambah, gerak global malah "muter salah"),
// set ini ke 1 untuk membalik tanda yaw yang dipakai di transform global/odometry.
#ifndef WORLD_YAW_INVERT
  #define WORLD_YAW_INVERT o
#endif

// ====== PID KINEMATIK (posisi X/Y + yaw) ======
// pidKinematik(targetX, targetY, targetYawDeg, batasXY, batasYaw)
// - targetX/targetY: dalam meter (sesuai getOdomX/Y)
// - batasXY/batasYaw: batas output command (satuan sama seperti moveRobotRpm, yaitu "RPM scale")

// Variabel global command PID Kinematik (untuk debug/LCD)
extern float cmdX;  // Command kanan (+)
extern float cmdY;  // Command maju (+)
extern float cmdW;  // Command rotasi CCW (+)

void pidKinematik(float targetX_m, float targetY_m, float targetYawDeg,
                  float batasXY, float batasYaw);
// Versi dengan rpmLimit runtime (biar manual mode bisa ikut R1/L1)
void pidKinematikLimit(float targetX_m, float targetY_m, float targetYawDeg,
                       float batasXY, float batasYaw, float rpmLimit);

// ====== GLOBAL FRAME COMMANDS ======
// Global/world frame memakai yaw1 sebagai orientasi robot.
// Konvensi koordinat GLOBAL (konsisten dengan odometry):
// - xRight (+)  : ke kanan
// - yForward (+): ke depan/maju
//
// Catatan: fungsi global ini akan mengubah (xRight,yForward) global -> (left,forward) lokal
// lalu memanggil moveRobotRpm / yaw hold.
void moveRobotRpmGlobal(float xRightGlobal, float yForwardGlobal, float w, float rpmLimit);
void moveRobotRpmYawHoldGlobal(float xRightGlobal, float yForwardGlobal, float targetYawDeg, float rpmLimit);
void moveRobotRpmYawHoldGlobal(float xRightGlobal, float yForwardGlobal, float rpmLimit);
// Versi dengan limit koreksi yaw runtime (batasYaw)
void moveRobotRpmYawHoldGlobal(float xRightGlobal, float yForwardGlobal, float targetYawDeg, float rpmLimit, float batasYaw);

// Jika 1, pidKinematik akan mengeluarkan command dalam GLOBAL frame (X kanan+, Y maju+)
// dan menjalankan robot via moveRobotRpmGlobal(). Supaya target posisi juga GLOBAL,
// aktifkan juga ODOM_USE_IMU_YAW.
#ifndef PIDKIN_USE_GLOBAL
  #define PIDKIN_USE_GLOBAL 1
#endif

#ifndef PIDKIN_ALPHA
  #define PIDKIN_ALPHA 0.9f
#endif
#ifndef PIDKIN_XY_KP
  #define PIDKIN_XY_KP 100.0f  // Naikkan dari 80 -> lebih responsif
#endif
#ifndef PIDKIN_XY_KI
  #define PIDKIN_XY_KI 0.0f
#endif
#ifndef PIDKIN_XY_KD
  #define PIDKIN_XY_KD 2.0f  // Tambah damping supaya tidak overshoot
#endif
#ifndef PIDKIN_XY_DEADBAND_M
  #define PIDKIN_XY_DEADBAND_M 0.01f   // 1 cm
#endif

#ifndef PIDKIN_YAW_DEADBAND_DEG
  #define PIDKIN_YAW_DEADBAND_DEG 5.0f
#endif

// PID yaw khusus untuk pidKinematik (satuan derajat -> output w "RPM scale")
#ifndef PIDKIN_YAW_KP
  #define PIDKIN_YAW_KP 0.5f  // Naikkan untuk error kecil, dikurangi otomatis untuk error besar
#endif
#ifndef PIDKIN_YAW_KI
  #define PIDKIN_YAW_KI 0.0f
#endif
#ifndef PIDKIN_YAW_KD
  #define PIDKIN_YAW_KD 0.0f  // Naikkan damping
#endif

// Channel PID untuk yaw di pidKinematik (jangan bentrok dengan wheel 0..2, yaw hold 3, XY 4..5)
#ifndef PIDKIN_YAW_CHANNEL
  #define PIDKIN_YAW_CHANNEL 6
#endif

// Limit RPM roda untuk moveRobotRpm di pidKinematik
#ifndef PIDKIN_RPM_LIMIT
  #define PIDKIN_RPM_LIMIT 400.0f  // Naikkan dari 250 -> gerak lebih cepat
#endif

// ====== WAYPOINT EXECUTOR ======
struct Waypoint {
  float x;              // default: mm (lihat WAYPOINT_UNITS_MM)
  float y;              // default: mm
  float angleDeg;       // derajat (0..360)
  float threshold;      // mm (atau meter jika WAYPOINT_UNITS_MM = 0)
  float angleThreshold; // derajat
  int batasXY;          // limit command XY (skala RPM)
  int batasYaw;         // limit command yaw (skala RPM)
};

// ====== WAYPOINT UNITS ======
// Pilih salah satu:
// - WAYPOINT_UNITS_MM = 1  => x/y/threshold dalam mm
// - WAYPOINT_UNITS_CM = 1  => x/y/threshold dalam cm
//
// Default: mm (biar cocok dengan contoh kamu: 500, 954, dst)
#ifndef WAYPOINT_UNITS_MM
  #define WAYPOINT_UNITS_MM 0
#endif
#ifndef WAYPOINT_UNITS_CM
  #define WAYPOINT_UNITS_CM 1
#endif
#if WAYPOINT_UNITS_MM && WAYPOINT_UNITS_CM
  #error "Pilih salah satu: WAYPOINT_UNITS_MM atau WAYPOINT_UNITS_CM"
#endif

// Jika 1, maka setiap waypoint dianggap RELATIF dari posisi saat ini.
// Implementasi: setelah waypoint tercapai, odometry di-reset (resetOdometry()).
#ifndef WAYPOINT_RELATIVE
  #define WAYPOINT_RELATIVE 0
#endif

bool executeWaypoint(float targetX, float targetY, float targetAngle,
                     float threshold, float angleThreshold,
                     int targetpwm, int targetyawpwm);

bool executeWaypoint(const Waypoint &wp);

// Eksekusi semua waypoint di waypointTarget[] (definisi di waypoint.ino)
bool runWaypoints();

// Current/active waypoint info (untuk ditampilkan di LCD)
bool hasActiveWaypoint();
int getActiveWaypointIndex();      // -1 kalau tidak diketahui
float getActiveWaypointX();        // mm kalau WAYPOINT_UNITS_MM=1, selain itu meter
float getActiveWaypointY();        // mm kalau WAYPOINT_UNITS_MM=1, selain itu meter
float getActiveWaypointAngleDeg(); // derajat

// ====== PS3 CONTROLLER ======
void setupPS3();

int prcPs3LX(); // -128..127 (kanan +)
int prcPs3LY(); // -128..127 (atas biasanya -)
int prcPs3RX();
int prcPs3RY();
int prcPs3R2(); // 0..255
int prcPs3L2(); // 0..255
int prcPs3R1(); // 0..255 (analog)
int prcPs3L1(); // 0..255 (analog)
bool prcPs3R1Btn(); // digital
bool prcPs3L1Btn(); // digital
bool prcPs3Up();
bool prcPs3Down();
bool prcPs3Left();
bool prcPs3Right();
bool prcPs3Cross();
bool prcPs3Circle();
bool prcPs3Triangle();
bool prcPs3Square();
bool prcPs3Start();
bool prcPs3Select();
bool prcPs3PS();

// ====== MANUAL CONTROL (PS3) ======
void manualControlPS3();

// Manual yaw target (untuk LCD/debug)
bool hasManualYawTarget();
float getManualYawTargetDeg();
float getManualTargetX(); // meter (X kanan +)
float getManualTargetY(); // meter (Y maju  +)

// Tuning manual control (unit: RPM command global)
#ifndef MANUAL_XY_SLOW
  #define MANUAL_XY_SLOW 400.0f
#endif
#ifndef MANUAL_XY_MED
  #define MANUAL_XY_MED 600.0f
#endif
#ifndef MANUAL_XY_FAST
  #define MANUAL_XY_FAST 900.0f
#endif
#ifndef MANUAL_YAW_RATE_DPS
  #define MANUAL_YAW_RATE_DPS 10.0f  // deg/s saat RX full
#endif
    #ifndef MANUAL_RPM_LIMIT_SLOW
  #define MANUAL_RPM_LIMIT_SLOW 200.0f
#endif
#ifndef MANUAL_RPM_LIMIT_MED
  #define MANUAL_RPM_LIMIT_MED 300.0f
#endif
#ifndef MANUAL_RPM_LIMIT_FAST
  #define MANUAL_RPM_LIMIT_FAST 500.0f
#endif
#ifndef MANUAL_YAW_LIMIT
  #define MANUAL_YAW_LIMIT 40.0f
#endif

// Saat translasi stop, apakah yaw target tetap di-hold?
#ifndef MANUAL_HOLD_YAW_WHEN_STOP
  #define MANUAL_HOLD_YAW_WHEN_STOP 0
#endif

// Kalau |yawTarget - yaw1| sudah kecil, motor dimatikan supaya tidak jitter
#ifndef MANUAL_STOP_YAW_DEADBAND_DEG
  #define MANUAL_STOP_YAW_DEADBAND_DEG 2.0f
#endif

// Limit koreksi yaw khusus saat translasi=0 (lebih kecil biar halus)
#ifndef MANUAL_STOP_YAW_LIMIT
  #define MANUAL_STOP_YAW_LIMIT 40.0f
#endif

// Offset target posisi (meter) untuk mode manual berbasis PID posisi:
// target = odom + stick*offset
#ifndef MANUAL_POS_OFFSET_M_SLOW
  #define MANUAL_POS_OFFSET_M_SLOW 0.20f
#endif
#ifndef MANUAL_POS_OFFSET_M_MED
  #define MANUAL_POS_OFFSET_M_MED 0.30f
#endif
#ifndef MANUAL_POS_OFFSET_M_FAST
  #define MANUAL_POS_OFFSET_M_FAST 0.50f
#endif

// Deadband stop supaya saat stick netral robot benar-benar diam (tidak jitter karena yaw PID)
#ifndef MANUAL_STOP_DEADBAND_RPM
  #define MANUAL_STOP_DEADBAND_RPM 8.0f
#endif

// Deadband RX (rotasi) supaya noise kecil tidak bikin yawTarget berubah
#ifndef MANUAL_RX_DEADZONE
  #define MANUAL_RX_DEADZONE 12
#endif

// Debug print PS3 ke Serial bisa bikin loop lambat (respons jadi terasa "nggak nendang")
#ifndef PS3_DEBUG_PRINTS
  #define PS3_DEBUG_PRINTS 0
#endif

// Jika diaktifkan, moveRobot() akan memakai rpmMotor() (closed-loop RPM),
// dan parameter speedLimit di moveRobot(...) diartikan sebagai batas RPM (bukan PWM).
#define OMNI_USE_RPM_CONTROL 1

// Mode untuk moveRobotRpm(): 0 = rpmMotor (closed-loop), 1 = pwmMotor (open-loop)
#ifndef USE_PWM_IN_MOVE_ROBOT_RPM
  #define USE_PWM_IN_MOVE_ROBOT_RPM 0
#endif

// Scaling factor untuk PWM mode (w1, w2, w3 dikali berapa sebelum ke pwmMotor)
#ifndef PWM_SCALING_FACTOR
  #define PWM_SCALING_FACTOR 2.0f
#endif

// ====== MPU CONFIG ======
// Deklarasi variabel global yaw, pitch, roll (definisi ada di mpu.ino)
extern float yaw;
extern float pitch;
extern float roll;
extern float yaw0;   // Offset yaw
extern float yaw1;   // Yaw setelah dikurangi offset
// Untuk LCD/debug odometry (di-set di `odometry.ino`)
extern float x_cm;
extern float y_cm;
// ====== OMNI 3 RODA (opsional) ======
// Jika kamu ingin kinematik pakai satuan fisik:
// - OMNI_WHEEL_RADIUS: jari-jari roda (meter)
// - OMNI_ROBOT_RADIUS: jarak dari pusat robot ke titik kontak roda (meter)
// Kalau tidak didefinisikan, default = 1 (artinya "mixing PWM" saja).
#define OMNI_WHEEL_RADIUS 0.03f
// #define OMNI_ROBOT_RADIUS 0.12f

// Invert arah motor (opsional) kalau ada motor yang arah majunya kebalik / tidak mau mundur.
// #define MOTOR1_INVERT 0
// #define MOTOR3_INVERT 0
// #define MOTOR4_INVERT 0

// Minimal PWM supaya motor mulai bergerak (deadband). 0 = off.
// Contoh: kalau motor baru gerak mulai 600, set:
#define OMNI_MIN_PWM 500.0f

// ====== YAW HEADING HOLD (PID) ======
// Yaw yang dipakai: yaw1 (derajat 0..360) dari `mpu.ino`.
// Output PID adalah komponen rotasi `w` untuk `moveRobot(vx, vy, w, limit)`.
//
// Default nilai di bawah aman sebagai awal, tapi tetap perlu tuning sesuai robot.
#ifndef YAW_PID_KP
  #define YAW_PID_KP 0.5f
#endif
#ifndef YAW_PID_KI
  #define YAW_PID_KI 0.0f
#endif
#ifndef YAW_PID_KD
  #define YAW_PID_KD 0.0f
#endif

// Batas integral (anti windup) dalam satuan derajat*detik
#ifndef YAW_PID_I_LIMIT
  #define YAW_PID_I_LIMIT 200.0f
#endif

// Batas output w (PWM) agar rotasi tidak terlalu agresif
#ifndef YAW_PID_W_LIMIT
  #define YAW_PID_W_LIMIT 500.0f
#endif

// Kalau arah putar yaw-hold kebalik (target 90 dari 0 malah muter kiri),
// set ini jadi 1 untuk membalik arah koreksi w.
#ifndef YAW_PID_INVERT
  // Pada setup kamu: saat target 0°, yaw aktual +10° justru makin menjauh.
  // Itu berarti arah koreksi w kebalik -> matikan invert.
  #define YAW_PID_INVERT 1
#endif

// ====== RPM PID DEADZONE COMPENSATION ======
// Saat OMNI_USE_RPM_CONTROL = 1, robot memakai rpmMotor() (closed-loop).
// Untuk target kecil (mis. koreksi yaw 5-10 derajat), output PID kadang kecil
// dan tidak cukup untuk mengalahkan gesekan/deadband motor -> terasa "lama".
// Set nilai minimal PWM nonzero supaya motor langsung "nendang" saat targetRpm != 0.
// Tuning cepat:
// - Naikkan sampai roda mulai muter konsisten di kecepatan rendah
// - Kalau terlalu agresif/jitter dekat target, turunkan sedikit
#ifndef RPM_PID_MIN_PWM
  #define RPM_PID_MIN_PWM 100
#endif

// minPWM hanya aktif jika |targetRpm| >= threshold ini.
// Tujuan: untuk target kecil (mis. 5..30 rpm dari yaw/posisi), jangan dipaksa minPWM karena bikin nyentak/jitter.
// Untuk kasus kamu (300, -150, -150), set threshold <= 150 supaya roda 150 ikut "nendang" bareng.
#ifndef RPM_PID_MIN_RPM_FOR_MINPWM
  // Jika roda 2/3 sering "delay" saat strafe (karena setengah dari roda 1),
  // turunkan threshold ini supaya target kecil juga dapat minPWM 600.
  // Tradeoff: makin kecil -> makin berpotensi jitter pada koreksi kecil (yaw/posisi).
  #define RPM_PID_MIN_RPM_FOR_MINPWM 20.0f
#endif

// ====== RPM PER-WHEEL CALIBRATION (untuk jalan lurus) ======
// Kalau saat moveRobotRpm/moveRobotRpmGlobal gerak miring karena 1-2 roda selalu "telat",
// kamu bisa kalibrasi per-roda di sini tanpa ubah kinematik.
//
// Mapping (sesuai rpmMotor di pid.ino):
// - w1 -> Motor 1
// - w2 -> Motor 3
// - w3 -> Motor 4
//
// Cara pakai (contoh):
// - Jika saat moveRobotRpmGlobal(200,0,0,200) miring karena M3 & M4 telat:
//   set RPM_W2_SCALE dan RPM_W3_SCALE jadi 1.05..1.20
#ifndef RPM_W1_SCALE
  #define RPM_W1_SCALE 1.0f
#endif
#ifndef RPM_W2_SCALE
  #define RPM_W2_SCALE 1.0f
#endif
#ifndef RPM_W3_SCALE
  #define RPM_W3_SCALE 1.0f
#endif

// Minimal PWM per-roda (override RPM_PID_MIN_PWM kalau perlu)
#ifndef RPM_PID_MIN_PWM_W1
  #define RPM_PID_MIN_PWM_W1 RPM_PID_MIN_PWM
#endif
#ifndef RPM_PID_MIN_PWM_W2
  #define RPM_PID_MIN_PWM_W2 RPM_PID_MIN_PWM
#endif
#ifndef RPM_PID_MIN_PWM_W3
  #define RPM_PID_MIN_PWM_W3 RPM_PID_MIN_PWM
#endif

// ====== RPM PID PER-WHEEL GAINS ======
// Ini untuk PID RPM roda (pid.ino -> rpmMotor). Kamu bisa set beda tiap roda.
// Mapping:
// - W1 -> Motor 1
// - W2 -> Motor 3
// - W3 -> Motor 4
//
// Contoh (kalau M3 telat): naikin Kp M3:
//   #define RPM_W2_KP 2.5
//
// Default: kalau tidak di-define, akan pakai default di pid.ino (RPM_PID_KP/KI/KD).
#ifndef RPM_W1_KP
  #define RPM_W1_KP RPM_PID_KP
#endif
#ifndef RPM_W1_KI
  #define RPM_W1_KI RPM_PID_KI
#endif
#ifndef RPM_W1_KD
  #define RPM_W1_KD RPM_PID_KD
#endif

#ifndef RPM_W2_KP
  #define RPM_W2_KP RPM_PID_KP 
#endif
#ifndef RPM_W2_KI
  #define RPM_W2_KI RPM_PID_KI
#endif
#ifndef RPM_W2_KD
  #define RPM_W2_KD RPM_PID_KD
#endif

#ifndef RPM_W3_KP
  #define RPM_W3_KP RPM_PID_KP
#endif
#ifndef RPM_W3_KI
  #define RPM_W3_KI RPM_PID_KI
#endif
#ifndef RPM_W3_KD
  #define RPM_W3_KD RPM_PID_KD
#endif

// ====== PID KINEMATIK - YAW TUNING HELPERS ======
// Minimum output yaw (cmdW) agar respon error kecil tidak kalah deadband.
#ifndef PIDKIN_YAW_MIN_OUT
  #define PIDKIN_YAW_MIN_OUT 8.0f
#endif
// Rentang error (deg) dimana minimum output yaw diaktifkan secara halus.
// Tujuan: mengalahkan deadband motor untuk error kecil tanpa bikin "sedat" dekat 0/360.
#ifndef PIDKIN_YAW_MIN_OUT_MAX_ERR_DEG
  #define PIDKIN_YAW_MIN_OUT_MAX_ERR_DEG 12.0f
#endif
// Gain scheduling: mulai kurangi Kp setelah error melewati batas (deg).
#ifndef PIDKIN_YAW_KP_SCALE_START_DEG
  #define PIDKIN_YAW_KP_SCALE_START_DEG 15.0f
#endif
// Error (deg) dimana scaling mencapai minimum.
#ifndef PIDKIN_YAW_KP_SCALE_END_DEG
  #define PIDKIN_YAW_KP_SCALE_END_DEG 60.0f
#endif
// Kp minimum = Kp * minScale
#ifndef PIDKIN_YAW_KP_MIN_SCALE
  #define PIDKIN_YAW_KP_MIN_SCALE 0.5f
#endif