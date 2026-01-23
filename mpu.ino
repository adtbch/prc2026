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
  Serial.println("[MPU] Initializing MPU6050...");
  _mpu.initialize();
  
  // Load DMP firmware
  // Note: Tidak perlu testConnection() karena bisa false positive
  // DMP init status yang akan validate connection
  Serial.println("[MPU] Loading DMP firmware...");
  _devStatus = _mpu.dmpInitialize();
  
  // Set gyro/accel offsets (dari kalibrasi)
  _mpu.setXGyroOffset(Imu::GYRO_OFFSET_X);
  _mpu.setYGyroOffset(Imu::GYRO_OFFSET_Y);
  _mpu.setZGyroOffset(Imu::GYRO_OFFSET_Z);
  _mpu.setZAccelOffset(Imu::ACCEL_OFFSET_Z);
  
  // Check DMP init status
  if (_devStatus == 0) {
    // DMP ready - calibrate dan enable
    Serial.println("[MPU] DMP init successful! Calibrating...");
    _mpu.CalibrateAccel(6);
    _mpu.CalibrateGyro(6);
    _mpu.PrintActiveOffsets();
    
    Serial.println("[MPU] Enabling DMP...");
    _mpu.setDMPEnabled(true);
    
    _packetSize = _mpu.dmpGetFIFOPacketSize();
    _dmpReady = true;
    
    Serial.println("[MPU] DMP ready! IMU is operational.");
    return true;
    
  } else {
    // DMP init failed
    Serial.print("[MPU] ERROR: DMP initialization failed! Error code: ");
    Serial.println(_devStatus);
    Serial.println("[MPU] Possible causes:");
    Serial.println("[MPU]   - I2C wiring issue (check SDA/SCL)");
    Serial.println("[MPU]   - MPU6050 not powered");
    Serial.println("[MPU]   - Wrong I2C address (should be 0x68)");
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
    imuYaw = math_wrapAngle360(imuYaw);
    
    // Apply calibration offset
    imuYawCalibrated = imuYaw - imuYawOffset;
    imuYawCalibrated = math_wrapAngle360(imuYawCalibrated);
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
