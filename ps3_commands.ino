/**
 * ============================================================
 * FILE: ps3_commands.ino
 * LAYER: User Interface Layer
 * ============================================================
 * 
 * DESCRIPTION:
 * PS3 controller command handler untuk auto-tuning dan debugging.
 * Menggunakan button combination untuk trigger commands.
 * 
 * BUTTON COMBINATIONS:
 * - L1 + Square    = Tune Motor 1
 * - L1 + Cross     = Tune Motor 2
 * - L1 + Circle    = Tune Motor 3
 * - L1 + Triangle  = Tune All Motors
 * - L2 + Start     = Print PID values
 * - L2 + Select    = Reset PID to defaults
 * - L1 + L2        = Cancel tuning
 * 
 * PUBLIC FUNCTIONS:
 * - handlePs3Commands() - dipanggil di loop()
 * 
 * ============================================================
 */

// Previous button states untuk edge detection (detect press, bukan hold)
static bool _prevL1 = false;
static bool _prevL2 = false;
static bool _prevSquare = false;
static bool _prevCross = false;
static bool _prevCircle = false;
static bool _prevTriangle = false;
static bool _prevStart = false;
static bool _prevSelect = false;

/**
 * @brief Detect button press (rising edge)
 * 
 * @param currentState Current button state
 * @param prevState Previous button state (will be updated)
 * @return true if button just pressed (0->1 transition)
 */
bool _isButtonPressed(bool currentState, bool &prevState) {
  bool pressed = currentState && !prevState;
  prevState = currentState;
  return pressed;
}

/**
 * @brief Handle PS3 controller commands untuk auto-tuning
 * 
 * Dipanggil di loop() untuk process button combinations.
 * Non-blocking, hanya execute jika PS3 connected.
 */
void handlePs3Commands() {
  // Skip jika PS3 tidak connected
  if (!ps3ControllerConnected) {
    return;
  }
  
  // Show PS3 status on LCD (non-tuning mode)
  static uint32_t lastStatusUpdate = 0;
  if (!autotuning_isActive() && (millis() - lastStatusUpdate >= 5000)) {
    lcd_debugMessage("PS3 Ready", "Press L1+Btn", 0);
    lastStatusUpdate = millis();
  }
  
  // Read current button states
  bool l1 = ps3ButtonL1;
  bool l2 = ps3ButtonL2;
  bool square = ps3ButtonSquare;
  bool cross = ps3ButtonX;
  bool circle = ps3ButtonCircle;
  bool triangle = ps3ButtonTriangle;
  bool start = ps3ButtonStart;
  bool select = ps3ButtonSelect;
  
  // ══════════════════════════════════════════════════════════
  // AUTO-TUNING COMMANDS (L1 + Face Buttons)
  // ══════════════════════════════════════════════════════════
  
  // L1 + Square = Tune Motor 1
  if (l1 && _isButtonPressed(square, _prevSquare)) {
    lcd_debugMessage("[L1+Square]", "Start Tune M1");
    delay(1000);
    autotuning_startMotor(0);
  }
  
  // L1 + Cross = Tune Motor 2
  else if (l1 && _isButtonPressed(cross, _prevCross)) {
    lcd_debugMessage("[L1+Cross]", "Start Tune M2");
    delay(1000);
    autotuning_startMotor(1);
  }
  
  // L1 + Circle = Tune Motor 3
  else if (l1 && _isButtonPressed(circle, _prevCircle)) {
    lcd_debugMessage("[L1+Circle]", "Start Tune M3");
    delay(1000);
    autotuning_startMotor(2);
  }
  
  // L1 + Triangle = Tune ALL motors (sequential)
  else if (l1 && _isButtonPressed(triangle, _prevTriangle)) {
    lcd_debugMessage("[L1+Triangle]", "Tune ALL Motors");
    delay(1000);
    autotuning_startAllMotors();  // Sequential tuning semua motor
  }
  
  // ══════════════════════════════════════════════════════════
  // UTILITY COMMANDS (L2 + Start/Select)
  // ══════════════════════════════════════════════════════════
  
  // L2 + Start = Show current PID values on LCD
  else if (l2 && _isButtonPressed(start, _prevStart)) {
    lcd_debugMessage("[L2+Start]", "Show PID");
    delay(1000);
    
    // Display each motor's PID sequentially
    for (uint8_t i = 0; i < 3; i++) {
      float kp, ki, kd;
      if (storage_getPidParameters(i, kp, ki, kd)) {
        lcd_debugPidValues(i, kp, ki, kd);
      }
    }
  }
  
  // L2 + Select = Reset ALL to defaults
  else if (l2 && _isButtonPressed(select, _prevSelect)) {
    lcd_debugMessage("[L2+Select]", "Reset to Def");
    delay(1000);
    
    // Reset all motors ke default values
    if (storage_resetPidToDefaults(255)) {
      // Apply defaults to runtime
      storage_loadPidParameters();
      
      lcd_debugMessage("Reset SUCCESS", "Kp=0.0 Ki=0.08");
      delay(2000);
      
      // Show confirmation untuk setiap motor
      for (uint8_t i = 0; i < 3; i++) {
        char line1[17];
        snprintf(line1, sizeof(line1), "M%d: RESET OK", i + 1);
        lcd_debugMessage(line1, "Kp=0.0 Ki=0.08");
        delay(1500);
      }
    } else {
      lcd_debugError("RESET_FAIL");
      delay(2000);
    }
  }
  
  // ══════════════════════════════════════════════════════════
  // CANCEL COMMAND (L1 + L2 together)
  // ══════════════════════════════════════════════════════════
  
  // L1 + L2 = Cancel tuning
  else if (l1 && l2 && (_isButtonPressed(l1, _prevL1) || _isButtonPressed(l2, _prevL2))) {
    lcd_debugMessage("[L1+L2]", "CANCEL Tuning");
    autotuning_cancel();
    delay(1500);
  }
  
  // Update previous states (untuk buttons yang tidak di-check di atas)
  _prevL1 = l1;
  _prevL2 = l2;
  
  // Note: Square, Cross, Circle, Triangle, Start, Select sudah di-update di _isButtonPressed()
}
