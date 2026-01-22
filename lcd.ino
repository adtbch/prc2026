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
  _lcd.print(" M3:");
  _lcd.print((int)encoderRpm[2]);
  _lcd.print("    ");  // Clear sisa karakter
  
  // Line 2: IMU Yaw dan PS3 connection status
  _lcd.setCursor(0, 1);
  _lcd.print("Y:");
  
  // Tampilkan yaw dalam format signed (-180 to +180)
  float yawSigned = imuYawCalibrated;
  if (yawSigned > 180.0f) yawSigned -= 360.0f;
  _lcd.print((int)yawSigned);
  _lcd.print(" ");
  
  // PS3 connection indicator
  if (ps3ControllerConnected) {
    _lcd.print("[PS3]");
  } else {
    _lcd.print("     ");
  }
  _lcd.print("   ");  // Clear sisa
}
