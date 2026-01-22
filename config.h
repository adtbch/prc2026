/**
 * ============================================================
 * CONFIGURATION FILE - Robot Omni 3 Roda PRC 2026
 * ============================================================
 * 
 * File ini berisi HANYA deklarasi konstanta dan parameter global.
 * Sesuai instructions: config.h hanya deklarasi variabel, NO IMPLEMENTATION.
 * Function prototypes TIDAK perlu ditulis (global functions tidak perlu deklarasi ulang).
 * 
 * SECTIONS:
 * 1. Hardware Pin Definitions
 * 2. Hardware Parameters (PWM, Encoder, IMU)
 * 3. Robot Physical Parameters
 * 4. PID Tuning Parameters
 * 5. Control Parameters
 * 
 * ============================================================
 */

#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>
#include <Wire.h>
#include <Ps3Controller.h>
#include <LiquidCrystal_I2C.h>
#include <MPU6050_6Axis_MotionApps20.h>

//══════════════════════════════════════════════════════════
// 1. HARDWARE PIN DEFINITIONS
//══════════════════════════════════════════════════════════

namespace Pin {
  // Motor L298N Module 1 - Control untuk Motor 1 & 2
  constexpr uint8_t MOTOR1_EN = 23;    // Enable PWM untuk motor 1
  constexpr uint8_t MOTOR1_A  = 1;     // Direction pin A motor 1
  constexpr uint8_t MOTOR1_B  = 3;     // Direction pin B motor 1
  
  constexpr uint8_t MOTOR2_EN = 5;     // Enable PWM untuk motor 2
  constexpr uint8_t MOTOR2_A  = 19;    // Direction pin A motor 2
  constexpr uint8_t MOTOR2_B  = 18;    // Direction pin B motor 2
  
  // Motor L298N Module 2 - Control untuk Motor 3 & 4
  constexpr uint8_t MOTOR3_EN = 17;    // Enable PWM untuk motor 3
  constexpr uint8_t MOTOR3_A  = 16;    // Direction pin A motor 3
  constexpr uint8_t MOTOR3_B  = 4;     // Direction pin B motor 3
  
  constexpr uint8_t MOTOR4_EN = 5;     // Enable PWM untuk motor 4 (lift/unused)
  constexpr uint8_t MOTOR4_A  = 0;     // Direction pin A motor 4
  constexpr uint8_t MOTOR4_B  = 2;     // Direction pin B motor 4
  
  // Encoder pins (GPIO yang support interrupt di ESP32)
  constexpr uint8_t ENCODER1_A = 34;   // Encoder motor 1 channel A
  constexpr uint8_t ENCODER1_B = 35;   // Encoder motor 1 channel B
  
  constexpr uint8_t ENCODER2_A = 32;   // Encoder motor 2 channel A
  constexpr uint8_t ENCODER2_B = 33;   // Encoder motor 2 channel B
  
  constexpr uint8_t ENCODER3_A = 14;   // Encoder motor 3 channel A
  constexpr uint8_t ENCODER3_B = 27;   // Encoder motor 3 channel B
  
  constexpr uint8_t ENCODER4_A = 26;   // Encoder motor 4 channel A
  constexpr uint8_t ENCODER4_B = 25;   // Encoder motor 4 channel B
  
  // I2C pins untuk MPU6050 dan LCD (default ESP32)
  // SDA = GPIO 21 (default, tidak perlu define)
  // SCL = GPIO 22 (default, tidak perlu define)
}

//══════════════════════════════════════════════════════════
// 2. HARDWARE PARAMETERS
//══════════════════════════════════════════════════════════

namespace Pwm {
  constexpr uint16_t MAX_VALUE = 4095;         // PWM 12-bit resolution
  constexpr uint16_t FREQUENCY_HZ = 5000;     // 5 kHz - optimal untuk L298N
  constexpr uint8_t RESOLUTION_BITS = 12;      // 12-bit: 0-4095
  
  // PWM channel assignments (ESP32 punya 16 PWM channels: 0-15)
  constexpr uint8_t MOTOR1_CHANNEL = 0;
  constexpr uint8_t MOTOR2_CHANNEL = 1;
  constexpr uint8_t MOTOR3_CHANNEL = 2;
  constexpr uint8_t MOTOR4_CHANNEL = 3;
  
}

namespace Encoder {
  constexpr int PULSES_PER_REVOLUTION = 210;   // PPR sesuai datasheet encoder
  constexpr int RPM_UPDATE_INTERVAL_MS = 100;  // Update RPM setiap 100ms
}

namespace Imu {
  constexpr uint8_t I2C_ADDRESS = 0x68;        // Default MPU6050 I2C address
  constexpr uint32_t I2C_CLOCK_HZ = 400000;    // 400kHz I2C fast mode
  
  // Gyro offsets (kalibrasi manual - sesuaikan dengan MPU kalian)
  constexpr int16_t GYRO_OFFSET_X = 220;
  constexpr int16_t GYRO_OFFSET_Y = 76;
  constexpr int16_t GYRO_OFFSET_Z = -85;
  constexpr int16_t ACCEL_OFFSET_Z = 1788;
}

namespace Lcd {
  constexpr uint8_t I2C_ADDRESS = 0x27;        // LCD I2C address (cek dengan i2c scanner)
  constexpr uint8_t COLUMNS = 16;              // LCD 16x2
  constexpr uint8_t ROWS = 2;
}

//══════════════════════════════════════════════════════════
// 3. ROBOT PHYSICAL PARAMETERS
//══════════════════════════════════════════════════════════

namespace Robot {
  // Dimensi fisik robot (dalam meter)
  constexpr float WHEEL_RADIUS_M = 0.03f;      // 30mm radius roda omni
  constexpr float ROBOT_RADIUS_M = 0.12f;      // 120mm dari center robot ke center roda
  constexpr float WHEEL_BASE_M = 0.20f;        // 200mm jarak antar roda (untuk odometry)
  
  // Konfigurasi roda omni 3-wheel (120 degree spacing)
  // Wheel 1 -> Motor 1 (depan, 0 derajat dari sumbu Y+)
  // Wheel 2 -> Motor 3 (kiri belakang, 120 derajat)
  // Wheel 3 -> Motor 4 (kanan belakang, 240 derajat)
  constexpr float WHEEL1_ANGLE_DEG = 0.0f;
  constexpr float WHEEL2_ANGLE_DEG = 120.0f;
  constexpr float WHEEL3_ANGLE_DEG = 240.0f;
  
  // Motor inversion flags (set 1 jika arah motor terbalik)
  constexpr bool MOTOR1_INVERTED = false;
  constexpr bool MOTOR3_INVERTED = false;
  constexpr bool MOTOR4_INVERTED = false;
}

//══════════════════════════════════════════════════════════
// 4. PID TUNING PARAMETERS
//══════════════════════════════════════════════════════════

/**
 * Struktur untuk menyimpan parameter PID
 * Memudahkan grouping dan maintenance tuning values
 */
struct PidGains {
  float kp;              // Proportional gain
  float ki;              // Integral gain
  float kd;              // Derivative gain
  float outputLimit;     // Maximum output value
  float deadband;        // Error deadband (ignore error < deadband)
};

namespace Tuning {
  // PID untuk closed-loop RPM control (wheel speed)
  constexpr PidGains RPM_WHEEL = {
    .kp = 2.0f,
    .ki = 0.08f,
    .kd = 0.0f,
    .outputLimit = 4095.0f,    // 12-bit PWM max
    .deadband = 0.0f
  };
  
  // PID untuk yaw hold (heading control saat translasi)
  constexpr PidGains YAW_HOLD = {
    .kp = 0.5f,
    .ki = 0.0f,
    .kd = 0.0f,
    .outputLimit = 500.0f,
    .deadband = 3.0f       // Ignore error < 3 derajat
  };
  
  // PID untuk position control X dan Y (navigation)
  constexpr PidGains POSITION_XY = {
    .kp = 100.0f,
    .ki = 0.0f,
    .kd = 2.0f,
    .outputLimit = 400.0f,
    .deadband = 0.01f      // 1cm deadband
  };
  
  // PID untuk position control Yaw (heading navigation)
  constexpr PidGains POSITION_YAW = {
    .kp = 0.5f,
    .ki = 0.0f,
    .kd = 0.0f,
    .outputLimit = 40.0f,
    .deadband = 5.0f       // Ignore error < 5 derajat
  };
  
  // Per-wheel RPM scaling (kalibrasi jika roda tidak sinkron)
  constexpr float RPM_WHEEL1_SCALE = 1.0f;
  constexpr float RPM_WHEEL2_SCALE = 1.0f;
  constexpr float RPM_WHEEL3_SCALE = 1.0f;

}

//══════════════════════════════════════════════════════════
// 5. CONTROL PARAMETERS
//══════════════════════════════════════════════════════════

namespace Control {
  // Manual control speed presets (RPM scale)
  constexpr float MANUAL_SPEED_SLOW = 200.0f;  // Mode lambat (L1)
  constexpr float MANUAL_SPEED_MEDIUM = 300.0f;// Mode sedang (default)
  constexpr float MANUAL_SPEED_FAST = 500.0f;  // Mode cepat (R1)
  
  // Joystick deadzone (ignore input < deadzone)
  constexpr int JOYSTICK_DEADZONE = 12;        // -12 to +12 dianggap 0
  
  // Diagonal joystick compensation
  // Saat diagonal, magnitude jadi √2 x single axis
  // Kurangi ke ~50% untuk feel yang konsisten
  constexpr int DIAGONAL_STICK_LIMIT = 63;     // 127 / 2 ≈ 63
  
  // Yaw rotation rate (derajat per detik saat stick full)
  constexpr float YAW_RATE_DEG_PER_SEC = 10.0f;
}

namespace Config {
  // Update rates (milliseconds)
  constexpr uint32_t ODOMETRY_UPDATE_MS = 20;   // 50Hz odometry update
  constexpr uint32_t DISPLAY_UPDATE_MS = 200;   // 5Hz LCD refresh
  
  // Odometry configuration
  constexpr bool USE_IMU_FOR_ODOMETRY = true;   // Fuse IMU yaw dengan wheel odometry
  constexpr bool INVERT_ODOM_X = false;         // Flip X axis jika arah kebalik
  constexpr bool INVERT_ODOM_Y = true;          // Flip Y axis jika arah kebalik
  
  // World frame yaw inversion (jika arah rotasi IMU kebalik)
  constexpr bool INVERT_WORLD_YAW = false;
}

//══════════════════════════════════════════════════════════
// 6. GLOBAL VARIABLES (Runtime State)
//══════════════════════════════════════════════════════════

// Encoder pulse counts (signed - bisa negatif untuk reverse)
volatile long encoderCount[4] = {0, 0, 0, 0};

// Pulse count absolut untuk RPM calculation (selalu positif)
static volatile unsigned long _encoderPulseCount[4] = {0, 0, 0, 0};

// Current RPM values (updated setiap INTERVAL_MS)
float encoderRpm[4] = {0.0f, 0.0f, 0.0f, 0.0f};

// Last state untuk quadrature decoding
static uint8_t _lastStateEncoder[4] = {0, 0, 0, 0};

// Timing untuk RPM calculation
static unsigned long _lastRpmUpdateTime = 0;



// ══════════════════════════════════════════════════════════
// GLOBAL VARIABLES
// ══════════════════════════════════════════════════════════

// Joystick values (-128 to 127, 0 = center)
int ps3StickLeftX = 0;
int ps3StickLeftY = 0;
int ps3StickRightX = 0;
int ps3StickRightY = 0;

// Connection status
bool ps3ControllerConnected = false;

// Button states (true = pressed)
bool ps3ButtonX = false;
bool ps3ButtonCircle = false;
bool ps3ButtonTriangle = false;
bool ps3ButtonSquare = false;
bool ps3ButtonL1 = false;
bool ps3ButtonL2 = false;
bool ps3ButtonL3 = false;
bool ps3ButtonR1 = false;
bool ps3ButtonR2 = false;
bool ps3ButtonR3 = false;
bool ps3DpadUp = false;
bool ps3DpadDown = false;
bool ps3DpadLeft = false;
bool ps3DpadRight = false;
bool ps3ButtonSelect = false;
bool ps3ButtonStart = false;


// ══════════════════════════════════════════════════════════
// GLOBAL VARIABLES
// ══════════════════════════════════════════════════════════

// MPU object
static MPU6050 _mpu;

// MPU control/status
static bool _dmpReady = false;          // DMP initialization status
static uint8_t _devStatus = 0;          // Device status setelah init
static uint16_t _packetSize = 0;        // Expected DMP packet size
static uint8_t _fifoBuffer[64];         // FIFO buffer

// Orientation data structures
static Quaternion _quaternion;          // Quaternion dari DMP
static VectorFloat _gravity;            // Gravity vector
static float _ypr[3];                   // Yaw, Pitch, Roll array

// IMU angles (declared in config.h)
float imuYaw = 0.0f;                    // Raw yaw (0-360 degrees)
float imuPitch = 0.0f;                  // Pitch angle
float imuRoll = 0.0f;                   // Roll angle
float imuYawOffset = 0.0f;              // Yaw calibration offset
float imuYawCalibrated = 0.0f;          // Yaw setelah offset (0-360)

#endif // CONFIG_H
