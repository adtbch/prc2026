/**
 * ============================================================
 * FILE: mpu.ino  
 * LAYER: Hardware Abstraction Layer
 * ============================================================
 * 
 * DESCRIPTION:
 * MPU6050 6-axis IMU dengan Digital Motion Processor (DMP).
 * Menyediakan stable yaw/pitch/roll untuk navigation dan odometry.
 * 
 * FEATURES:
 * - DMP untuk sensor fusion (gyro + accel)
 * - Quaternion-based orientation (no gimbal lock)
 * - Yaw calibration support
 * 
 * HARDWARE:
 * - MPU6050 via I2C (address 0x68)
 * - DMP firmware loaded to MPU
 * 
 * PUBLIC FUNCTIONS:
 * - hardware_initializeImu()
 * - hardware_updateImu()
 * - hardware_calibrateImuYaw()
 * - hardware_getImuYaw()
 * 
 * ============================================================
 */

// ══════════════════════════════════════════════════════════
// PRIVATE FUNCTIONS
// ══════════════════════════════════════════════════════════

/**
 * @brief Normalize angle ke range 0-360 degrees
 * 
 * @param angle Angle in degrees (bisa di luar range 0-360)
 * @return Normalized angle (0-360)
 */
static inline float _imu_normalizeAngle360(float angle) {
  while (angle < 0.0f) angle += 360.0f;
  while (angle >= 360.0f) angle -= 360.0f;
  return angle;
}

// ══════════════════════════════════════════════════════════
// PUBLIC FUNCTIONS
// ══════════════════════════════════════════════════════════

/**
 * @brief Initialize MPU6050 dan load DMP firmware
 * 
 * Steps:
 * 1. Initialize I2C communication
 * 2. Reset MPU dan load DMP firmware
 * 3. Set gyro/accel offsets (kalibrasi)
 * 4. Enable DMP
 * 
 * @return true jika berhasil, false jika gagal
 */
bool hardware_initializeImu() {
  // Initialize I2C
  Wire.begin();
  Wire.setClock(Imu::I2C_CLOCK_HZ);
  
  // Initialize MPU6050
  Serial.println("Initializing MPU6050...");
  _mpu.initialize();
  
  // Verify connection
  if (!_mpu.testConnection()) {
    Serial.println("MPU6050 connection failed!");
    return false;
  }
  Serial.println("MPU6050 connection successful");
  
  // Load DMP firmware
  Serial.println("Loading DMP firmware...");
  _devStatus = _mpu.dmpInitialize();
  
  // Set gyro/accel offsets (dari kalibrasi)
  _mpu.setXGyroOffset(Imu::GYRO_OFFSET_X);
  _mpu.setYGyroOffset(Imu::GYRO_OFFSET_Y);
  _mpu.setZGyroOffset(Imu::GYRO_OFFSET_Z);
  _mpu.setZAccelOffset(Imu::ACCEL_OFFSET_Z);
  
  // Check DMP init status
  if (_devStatus == 0) {
    // DMP ready - calibrate dan enable
    Serial.println("Calibrating DMP...");
    _mpu.CalibrateAccel(6);
    _mpu.CalibrateGyro(6);
    _mpu.PrintActiveOffsets();
    
    Serial.println("Enabling DMP...");
    _mpu.setDMPEnabled(true);
    
    _packetSize = _mpu.dmpGetFIFOPacketSize();
    _dmpReady = true;
    
    Serial.println("DMP ready!");
    return true;
    
  } else {
    // DMP init failed
    Serial.print("DMP init failed! Error code: ");
    Serial.println(_devStatus);
    return false;
  }
}

/**
 * @brief Update IMU readings dari DMP
 * 
 * Membaca data dari FIFO, extract quaternion, dan convert ke Euler angles.
 * Dipanggil setiap cycle di loop() untuk real-time orientation tracking.
 * 
 * Formula konversi: Quaternion -> Yaw/Pitch/Roll
 */
void hardware_updateImu() {
  // Skip jika DMP tidak ready
  if (!_dmpReady) {
    // Debug warning (print sekali saja)
    static bool warningPrinted = false;
    if (!warningPrinted) {
      Serial.println("[IMU] WARNING: DMP not ready - Yaw will remain 0");
      warningPrinted = true;
    }
    return;
  }
  
  // Read packet dari FIFO
  if (_mpu.dmpGetCurrentFIFOPacket(_fifoBuffer)) {
    // Extract quaternion
    _mpu.dmpGetQuaternion(&_quaternion, _fifoBuffer);
    
    // Extract gravity vector
    _mpu.dmpGetGravity(&_gravity, &_quaternion);
    
    // Convert quaternion -> Euler angles (yaw, pitch, roll)
    _mpu.dmpGetYawPitchRoll(_ypr, &_quaternion, &_gravity);
    
    // Convert radians ke degrees
    imuYaw = _ypr[0] * 180.0f / PI;
    imuPitch = _ypr[1] * 180.0f / PI;
    imuRoll = _ypr[2] * 180.0f / PI;
    
    // Normalize yaw ke 0-360
    imuYaw = _imu_normalizeAngle360(imuYaw);
    
    // Apply calibration offset
    imuYawCalibrated = imuYaw - imuYawOffset;
    imuYawCalibrated = _imu_normalizeAngle360(imuYawCalibrated);
  }
}

/**
 * @brief Calibrate yaw offset (set current yaw sebagai 0)
 * 
 * Berguna untuk:
 * - Reset heading ke 0 di awal program
 * - Re-calibrate jika robot di-geser secara manual
 * 
 * Cara pakai:
 * 1. Posisikan robot menghadap ke arah yang diinginkan sebagai 0°
 * 2. Call fungsi ini
 * 3. Yaw akan ter-reset ke 0°
 */
void hardware_calibrateImuYaw() {
  imuYawOffset = imuYaw;
  imuYawCalibrated = 0.0f;
  Serial.print("IMU Yaw calibrated. Offset: ");
  Serial.println(imuYawOffset);
}

/**
 * @brief Get calibrated yaw angle
 * 
 * @return Yaw angle setelah offset (0-360 degrees)
 */
float hardware_getImuYaw() {
  return imuYawCalibrated;
}
