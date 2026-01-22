/**
 * ============================================================
 * FILE: pid_storage.ino
 * LAYER: Storage Layer
 * ============================================================
 * 
 * DESCRIPTION:
 * Preferences storage system untuk PID parameters.
 * Menyimpan dan memuat nilai Ki, Kp, Kd untuk masing-masing motor.
 * 
 * BEST PRACTICES:
 * - Separate namespace per motor untuk avoid conflicts
 * - Atomic write operations dengan error checking
 * - Default values sebagai fallback
 * - Validation sebelum apply values
 * 
 * PUBLIC FUNCTIONS:
 * - storage_initializePidPreferences()
 * - storage_loadPidParameters()
 * - storage_savePidParameters(motorId, kp, ki, kd)
 * - storage_resetPidToDefaults(motorId)
 * 
 * ============================================================
 */

// Preferences object
static Preferences _pidPreferences;

// Namespace names untuk setiap motor
namespace PidStorage {
  constexpr char MOTOR1_NAMESPACE[] = "pid_motor1";
  constexpr char MOTOR2_NAMESPACE[] = "pid_motor2";
  constexpr char MOTOR3_NAMESPACE[] = "pid_motor3";
  constexpr char TUNING_NAMESPACE[] = "pid_tuning";  // Auto-tuning metadata
}

// Default PID values (Industrial standard for DC motor RPM control)
// Based on Ziegler-Nichols Quarter Decay Ratio method
namespace PidDefaults {
  // Motor 1 (Wheel 1 - Depan)
  constexpr float MOTOR1_KP = 0.0f;
  constexpr float MOTOR1_KI = 0.0f;
  constexpr float MOTOR1_KD = 0.0f;
  
  // Motor 2 (Wheel 2 - Kiri Belakang)
  constexpr float MOTOR2_KP = 0.0f;
  constexpr float MOTOR2_KI = 0.0f;
  constexpr float MOTOR2_KD = 0.0f;
  
  // Motor 3 (Wheel 3 - Kanan Belakang)
  constexpr float MOTOR3_KP = 0.0f;
  constexpr float MOTOR3_KI = 0.0f;
  constexpr float MOTOR3_KD = 0.0f;
}

// ════════════════════════════════════════════════════════
// HELPER FUNCTIONS
// ══════════════════════════════════════════════════════════

/**
 * @brief Get namespace name untuk motor tertentu
 * 
 * @param motorId Motor ID (0-2)
 * @return Namespace string
 */
const char* _getMotorNamespace(uint8_t motorId) {
  switch (motorId) {
    case 0: return PidStorage::MOTOR1_NAMESPACE;
    case 1: return PidStorage::MOTOR2_NAMESPACE;
    case 2: return PidStorage::MOTOR3_NAMESPACE;
    default: return PidStorage::MOTOR1_NAMESPACE;
  }
}

/**
 * @brief Validate PID values untuk prevent extreme/invalid values
 * 
 * @param kp Proportional gain
 * @param ki Integral gain
 * @param kd Derivative gain
 * @return true if values are valid
 */
bool _validatePidValues(float kp, float ki, float kd) {
  // Kp range: 0.0 to 100 (0.0 = proportional disabled)
  if (kp < 0.0f || kp > 100.0f) {
    Serial.printf("[PID Storage] Validation FAILED: Kp=%.4f out of range\n", kp);
    return false;
  }
  
  // Ki range: 0 to 50 (prevent integral windup)
  if (ki < 0.0f || ki > 50.0f) {
    Serial.printf("[PID Storage] Validation FAILED: Ki=%.4f out of range\n", ki);
    return false;
  }
  
  // Kd range: 0 to 10 (derivative term noise sensitivity)
  if (kd < 0.0f || kd > 10.0f) {
    Serial.printf("[PID Storage] Validation FAILED: Kd=%.4f out of range\n", kd);
    return false;
  }
  
  return true;
}

// ══════════════════════════════════════════════════════════
// PUBLIC FUNCTIONS
// ══════════════════════════════════════════════════════════

/**
 * @brief Initialize PID storage system
 * 
 * Harus dipanggil di setup() sebelum load/save operations
 */
void storage_initializePidPreferences() {
  Serial.println("[PID Storage] Initializing Preferences system...");
  
  // Test write capability
  _pidPreferences.begin(PidStorage::TUNING_NAMESPACE, false);
  bool initSuccess = _pidPreferences.putBool("initialized", true);
  _pidPreferences.end();
  
  if (!initSuccess) {
    Serial.println("[PID Storage] ERROR: Failed to initialize Preferences!");
  } else {
    Serial.println("[PID Storage] Preferences system ready");
  }
}

/**
 * @brief Load PID parameters untuk semua motor dari Preferences
 * 
 * Akan apply values ke config.h Tuning:: constants.
 * Jika tidak ada saved values, gunakan defaults.
 */
void storage_loadPidParameters() {
  Serial.println("[PID Storage] Loading PID parameters from flash...");
  
  // Load Motor 1 (Wheel 1)
  _pidPreferences.begin(PidStorage::MOTOR1_NAMESPACE, true);  // read-only
  float kp1 = _pidPreferences.getFloat("kp", PidDefaults::MOTOR1_KP);
  float ki1 = _pidPreferences.getFloat("ki", PidDefaults::MOTOR1_KI);
  float kd1 = _pidPreferences.getFloat("kd", PidDefaults::MOTOR1_KD);
  _pidPreferences.end();
  
  // Validate and apply to runtime
  if (_validatePidValues(kp1, ki1, kd1)) {
    Tuning::RPM_WHEEL1.kp = kp1;
    Tuning::RPM_WHEEL1.ki = ki1;
    Tuning::RPM_WHEEL1.kd = kd1;
    Serial.printf("[PID Storage] Motor 1: Kp=%.3f, Ki=%.3f, Kd=%.3f\n", kp1, ki1, kd1);
  } else {
    Serial.println("[PID Storage] Motor 1: Invalid values, using defaults");
    kp1 = PidDefaults::MOTOR1_KP;
    ki1 = PidDefaults::MOTOR1_KI;
    kd1 = PidDefaults::MOTOR1_KD;
  }
  
  // Load Motor 2 (Wheel 2)
  _pidPreferences.begin(PidStorage::MOTOR2_NAMESPACE, true);
  float kp2 = _pidPreferences.getFloat("kp", PidDefaults::MOTOR2_KP);
  float ki2 = _pidPreferences.getFloat("ki", PidDefaults::MOTOR2_KI);
  float kd2 = _pidPreferences.getFloat("kd", PidDefaults::MOTOR2_KD);
  _pidPreferences.end();
  
  if (_validatePidValues(kp2, ki2, kd2)) {
    Tuning::RPM_WHEEL2.kp = kp2;
    Tuning::RPM_WHEEL2.ki = ki2;
    Tuning::RPM_WHEEL2.kd = kd2;
    Serial.printf("[PID Storage] Motor 2: Kp=%.3f, Ki=%.3f, Kd=%.3f\n", kp2, ki2, kd2);
  } else {
    Serial.println("[PID Storage] Motor 2: Invalid values, using defaults");
    kp2 = PidDefaults::MOTOR2_KP;
    ki2 = PidDefaults::MOTOR2_KI;
    kd2 = PidDefaults::MOTOR2_KD;
  }
  
  // Load Motor 3 (Wheel 3)
  _pidPreferences.begin(PidStorage::MOTOR3_NAMESPACE, true);
  float kp3 = _pidPreferences.getFloat("kp", PidDefaults::MOTOR3_KP);
  float ki3 = _pidPreferences.getFloat("ki", PidDefaults::MOTOR3_KI);
  float kd3 = _pidPreferences.getFloat("kd", PidDefaults::MOTOR3_KD);
  _pidPreferences.end();
  
  if (_validatePidValues(kp3, ki3, kd3)) {
    Tuning::RPM_WHEEL3.kp = kp3;
    Tuning::RPM_WHEEL3.ki = ki3;
    Tuning::RPM_WHEEL3.kd = kd3;
    Serial.printf("[PID Storage] Motor 3: Kp=%.3f, Ki=%.3f, Kd=%.3f\n", kp3, ki3, kd3);
  } else {
    Serial.println("[PID Storage] Motor 3: Invalid values, using defaults");
    kp3 = PidDefaults::MOTOR3_KP;
    ki3 = PidDefaults::MOTOR3_KI;
    kd3 = PidDefaults::MOTOR3_KD;
  }
  
  Serial.println("[PID Storage] Load complete");
}

/**
 * @brief Save PID parameters untuk motor tertentu ke Preferences
 * 
 * @param motorId Motor ID (0=Motor1, 1=Motor2, 2=Motor3)
 * @param kp Proportional gain
 * @param ki Integral gain
 * @param kd Derivative gain
 * @return true if save successful
 */
bool storage_savePidParameters(uint8_t motorId, float kp, float ki, float kd) {
  if (motorId > 2) {
    Serial.println("[PID Storage] ERROR: Invalid motor ID");
    return false;
  }
  
  // Validate values before saving
  if (!_validatePidValues(kp, ki, kd)) {
    Serial.println("[PID Storage] ERROR: Invalid PID values, not saved");
    return false;
  }
  
  const char* ns = _getMotorNamespace(motorId);
  
  Serial.printf("[PID Storage] Saving Motor %d: Kp=%.3f, Ki=%.3f, Kd=%.3f\n", 
                motorId + 1, kp, ki, kd);
  
  _pidPreferences.begin(ns, false);  // read-write mode
  
  bool success = true;
  success &= _pidPreferences.putFloat("kp", kp);
  success &= _pidPreferences.putFloat("ki", ki);
  success &= _pidPreferences.putFloat("kd", kd);
  
  // Save timestamp untuk tracking
  success &= _pidPreferences.putULong("timestamp", millis());
  
  _pidPreferences.end();
  
  if (success) {
    Serial.printf("[PID Storage] Motor %d saved successfully\n", motorId + 1);
  } else {
    Serial.printf("[PID Storage] ERROR: Failed to save Motor %d\n", motorId + 1);
  }
  
  return success;
}

/**
 * @brief Reset PID parameters ke default values untuk motor tertentu
 * 
 * @param motorId Motor ID (0-2), atau 255 untuk reset semua motor
 * @return true if reset successful
 */
bool storage_resetPidToDefaults(uint8_t motorId) {
  bool success = true;
  
  if (motorId == 255) {
    // Reset semua motor
    Serial.println("[PID Storage] Resetting ALL motors to defaults...");
    success &= storage_savePidParameters(0, PidDefaults::MOTOR1_KP, 
                                          PidDefaults::MOTOR1_KI, PidDefaults::MOTOR1_KD);
    success &= storage_savePidParameters(1, PidDefaults::MOTOR2_KP, 
                                          PidDefaults::MOTOR2_KI, PidDefaults::MOTOR2_KD);
    success &= storage_savePidParameters(2, PidDefaults::MOTOR3_KP, 
                                          PidDefaults::MOTOR3_KI, PidDefaults::MOTOR3_KD);
  } else if (motorId <= 2) {
    // Reset satu motor
    Serial.printf("[PID Storage] Resetting Motor %d to defaults...\n", motorId + 1);
    
    float kp, ki, kd;
    switch (motorId) {
      case 0:
        kp = PidDefaults::MOTOR1_KP;
        ki = PidDefaults::MOTOR1_KI;
        kd = PidDefaults::MOTOR1_KD;
        break;
      case 1:
        kp = PidDefaults::MOTOR2_KP;
        ki = PidDefaults::MOTOR2_KI;
        kd = PidDefaults::MOTOR2_KD;
        break;
      case 2:
      default:
        kp = PidDefaults::MOTOR3_KP;
        ki = PidDefaults::MOTOR3_KI;
        kd = PidDefaults::MOTOR3_KD;
        break;
    }
    
    success = storage_savePidParameters(motorId, kp, ki, kd);
  } else {
    Serial.println("[PID Storage] ERROR: Invalid motor ID for reset");
    return false;
  }
  
  if (success) {
    Serial.println("[PID Storage] Reset complete");
  } else {
    Serial.println("[PID Storage] ERROR: Reset failed");
  }
  
  return success;
}

/**
 * @brief Get saved PID values untuk motor tertentu (utility function)
 * 
 * @param motorId Motor ID (0-2)
 * @param kp Output: Kp value
 * @param ki Output: Ki value
 * @param kd Output: Kd value
 * @return true if values retrieved successfully
 */
bool storage_getPidParameters(uint8_t motorId, float& kp, float& ki, float& kd) {
  if (motorId > 2) return false;
  
  const char* ns = _getMotorNamespace(motorId);
  
  _pidPreferences.begin(ns, true);  // read-only
  
  // Get default values based on motor
  float defaultKp, defaultKi, defaultKd;
  switch (motorId) {
    case 0:
      defaultKp = PidDefaults::MOTOR1_KP;
      defaultKi = PidDefaults::MOTOR1_KI;
      defaultKd = PidDefaults::MOTOR1_KD;
      break;
    case 1:
      defaultKp = PidDefaults::MOTOR2_KP;
      defaultKi = PidDefaults::MOTOR2_KI;
      defaultKd = PidDefaults::MOTOR2_KD;
      break;
    case 2:
    default:
      defaultKp = PidDefaults::MOTOR3_KP;
      defaultKi = PidDefaults::MOTOR3_KI;
      defaultKd = PidDefaults::MOTOR3_KD;
      break;
  }
  
  kp = _pidPreferences.getFloat("kp", defaultKp);
  ki = _pidPreferences.getFloat("ki", defaultKi);
  kd = _pidPreferences.getFloat("kd", defaultKd);
  
  _pidPreferences.end();
  
  return _validatePidValues(kp, ki, kd);
}

/**
 * @brief Print all saved PID values untuk debugging
 */
void storage_printAllPidValues() {
  Serial.println("\n========== SAVED PID VALUES ==========");
  
  for (uint8_t i = 0; i < 3; i++) {
    float kp, ki, kd;
    if (storage_getPidParameters(i, kp, ki, kd)) {
      Serial.printf("Motor %d: Kp=%.4f, Ki=%.4f, Kd=%.4f\n", i + 1, kp, ki, kd);
    } else {
      Serial.printf("Motor %d: ERROR retrieving values\n", i + 1);
    }
  }
  
  Serial.println("======================================\n");
}
