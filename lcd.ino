/**
 * ============================================================
 * FILE: lcd.ino
 * LAYER: Hardware Abstraction Layer
 * ============================================================
 * 
 * DESCRIPTION:
 * LCD 16x2 I2C untuk display status robot.
 * Menampilkan odometry, RPM, mode, dan debug info.
 * 
 * HARDWARE:
 * - LCD 16x2 with I2C backpack (PCF8574)
 * - I2C Address: 0x27 (default)
 * 
 * PUBLIC FUNCTIONS:
 * - hardware_initializeLcd()
 * - lcd_clear()
 * - lcd_showMessage(line1, line2)
 * - lcd_printAt(col, row, text)
 * - lcd_printFloatAt(col, row, value, decimals)
 * - lcd_displayRobotStatus()
 * 
 * ============================================================
 */

// ══════════════════════════════════════════════════════════
// GLOBAL VARIABLES
// ══════════════════════════════════════════════════════════

// LCD object (address, columns, rows)
static LiquidCrystal_I2C _lcd(Lcd::I2C_ADDRESS, Lcd::COLUMNS, Lcd::ROWS);

// ══════════════════════════════════════════════════════════
// PUBLIC FUNCTIONS
// ══════════════════════════════════════════════════════════

/**
 * @brief Initialize LCD display
 * 
 * Setup I2C communication dan tampilkan splash screen
 */
void hardware_initializeLcd() {
  _lcd.init();
  _lcd.backlight();
  
  // Splash screen
  _lcd.setCursor(0, 0);
  _lcd.print("PRC2026 Robot");
  _lcd.setCursor(0, 1);
  _lcd.print("Initializing...");
}

/**
 * @brief Clear LCD screen
 */
void lcd_clear() {
  _lcd.clear();
}

/**
 * @brief Tampilkan 2 baris text (centered untuk LCD 16x2)
 * 
 * @param line1 Text untuk baris pertama
 * @param line2 Text untuk baris kedua
 */
void lcd_showMessage(const char* line1, const char* line2) {
  _lcd.clear();
  _lcd.setCursor(0, 0);
  _lcd.print(line1);
  _lcd.setCursor(0, 1);
  _lcd.print(line2);
}

/**
 * @brief Print text di posisi tertentu
 * 
 * @param col Column (0-15 untuk LCD 16x2)
 * @param row Row (0-1 untuk LCD 16x2)
 * @param text Text string
 */
void lcd_printAt(uint8_t col, uint8_t row, const char* text) {
  _lcd.setCursor(col, row);
  _lcd.print(text);
}

/**
 * @brief Print float value di posisi tertentu
 * 
 * @param col Column
 * @param row Row
 * @param value Float value
 * @param decimals Jumlah desimal (0-3)
 */
void lcd_printFloatAt(uint8_t col, uint8_t row, float value, uint8_t decimals) {
  _lcd.setCursor(col, row);
  _lcd.print(value, decimals);
}

/**
 * @brief Display robot status (dipanggil setiap 200ms)
 * 
 * Format LCD 16x2:
 * Line 1: X, Y position (cm)
 * Line 2: Yaw angle, PS3 connection status
 */
void lcd_displayRobotStatus() {
  // Line 1: Encoder RPM
  _lcd.setCursor(0, 0);
  _lcd.print("M1:");
  _lcd.print((int)encoderRpm[0]);
  _lcd.print(" M2:");
  _lcd.print((int)encoderRpm[1]);
  _lcd.print("    ");  // Clear sisa karakter
  
  // Line 2: IMU Yaw dan PS3 connection status
  _lcd.setCursor(0, 1);
  
  // Check apakah IMU ready (extern dari mpu.ino)
  extern bool _dmpReady;
  if (_dmpReady) {
    // IMU working - display yaw
    _lcd.print("Y:");
    
    // Tampilkan yaw dalam format signed (-180 to +180)
    float yawSigned = imuYawCalibrated;
    if (yawSigned > 180.0f) yawSigned -= 360.0f;
    _lcd.print((int)yawSigned);
    _lcd.print(" ");
  } else {
    // IMU not working - display error
    _lcd.print("Y:ERR ");
  }
  
  // PS3 connection indicator
  if (ps3ControllerConnected) {
    _lcd.print("[PS3]");
  } else {
    _lcd.print("     ");
  }
  _lcd.print("   ");  // Clear sisa
}

// ══════════════════════════════════════════════════════════
// DEBUG FUNCTIONS (Replacement untuk Serial debug)
// ══════════════════════════════════════════════════════════

/**
 * @brief Display debug message di LCD (2 lines max)
 * 
 * @param line1 Message untuk baris 1 (max 16 chars)
 * @param line2 Message untuk baris 2 (max 16 chars)
 * @param duration_ms Duration display (0 = permanent)
 */
void lcd_debugMessage(const char* line1, const char* line2, uint16_t duration_ms) {
  _lcd.clear();
  _lcd.setCursor(0, 0);
  _lcd.print(line1);
  
  if (line2 && strlen(line2) > 0) {
    _lcd.setCursor(0, 1);
    _lcd.print(line2);
  }
  
  // Non-blocking delay untuk temporary messages
  if (duration_ms > 0) {
    delay(duration_ms);
  }
}

/**
 * @brief Display formatted auto-tuning progress di LCD
 * 
 * @param motorId Motor ID (0-2)
 * @param cycle Current cycle
 * @param maxCycles Max cycles
 * @param progress Progress percentage (0-100)
 */
void lcd_debugTuningProgress(uint8_t motorId, uint8_t cycle, uint8_t maxCycles, uint8_t progress) {
  char line1[17];
  char line2[17];
  
  snprintf(line1, sizeof(line1), "Tune M%d:%d/%d", motorId + 1, cycle, maxCycles);
  snprintf(line2, sizeof(line2), "Progress: %d%%", progress);
  
  _lcd.clear();
  _lcd.setCursor(0, 0);
  _lcd.print(line1);
  _lcd.setCursor(0, 1);
  _lcd.print(line2);
}

/**
 * @brief Display PID values di LCD
 * 
 * @param motorId Motor ID (0-2)
 * @param kp Kp value
 * @param ki Ki value
 * @param kd Kd value
 */
void lcd_debugPidValues(uint8_t motorId, float kp, float ki, float kd) {
  char line1[17];
  char line2[17];
  
  snprintf(line1, sizeof(line1), "M%d Kp:%.2f", motorId + 1, kp);
  snprintf(line2, sizeof(line2), "Ki:%.2f Kd:%.2f", ki, kd);
  
  lcd_debugMessage(line1, line2, 3000);
}

/**
 * @brief Display metrics di LCD
 * 
 * @param overshoot Overshoot percentage
 * @param riseTime Rise time in ms
 * @param score Quality score
 */
void lcd_debugMetrics(float overshoot, unsigned long riseTime, float score) {
  char line1[17];
  char line2[17];
  
  snprintf(line1, sizeof(line1), "OS:%.1f%% RT:%lu", overshoot, riseTime);
  snprintf(line2, sizeof(line2), "Score: %.2f", score);
  
  lcd_debugMessage(line1, line2, 2000);
}

/**
 * @brief Display error message di LCD
 * 
 * @param errorCode Short error code/message
 */
void lcd_debugError(const char* errorCode) {
  char line1[17];
  snprintf(line1, sizeof(line1), "ERROR: %s", errorCode);
  lcd_debugMessage(line1, "", 3000);
}

/**
 * @brief Display initialization status
 * 
 * @param component Component name (max 10 chars)
 * @param success true if OK, false if failed
 */
void lcd_debugInit(const char* component, bool success) {
  char line1[17];
  snprintf(line1, sizeof(line1), "Init: %s", component);
  lcd_debugMessage(line1, success ? "OK" : "FAILED", 1000);
}
