/**
 * ============================================================
 * FILE: ps3_interface.ino
 * LAYER: User Interface Layer
 * ============================================================
 * 
 * DESCRIPTION:
 * Complete PS3 controller interface untuk manual control dan commands.
 * Menggabungkan joystick control dengan button combinations.
 * 
 * MANUAL CONTROL (Continuous - Every Loop):
 * - Left Stick X/Y  : Translational movement (strafe + forward/backward)
 * - Right Stick X   : Rotational movement (yaw)
 * - R2 Trigger      : Turbo mode (2x speed)
 * - L2 (hold alone) : Slow mode (0.3x speed, precision)
 * 
 * BUTTON COMMANDS (Discrete - On Press):
 * - L1 + Square    : Tune Motor 1
 * - L1 + Cross     : Tune Motor 2
 * - L1 + Circle    : Tune Motor 3
 * - L1 + Triangle  : Tune All Motors
 * - L2 + Start     : Show PID values
 * - L2 + Select    : Reset PID to defaults
 * - L1 + L2        : Cancel tuning
 * 
 * FEATURES:
 * - Deadzone untuk eliminate stick drift
 * - Exponential response curve untuk smooth control
 * - Field-centric control (global frame movement)
 * - Edge detection untuk button presses
 * - Command mode overrides manual control
 * 
 * PUBLIC FUNCTIONS:
 * - ps3Interface_updateManualControl() - Joystick control
 * - ps3Interface_handleCommands() - Button combinations
 * - ps3Interface_stop() - Emergency stop
 * - ps3Interface_getMode() - Get current mode string
 * 
 * ============================================================
 */

// ══════════════════════════════════════════════════════════
// CONFIGURATION CONSTANTS
// ══════════════════════════════════════════════════════════

namespace PS3Config {
  // Manual Control Parameters
  constexpr float STICK_DEADZONE = 15.0f;      // Range 0-128, ignore < 15
  constexpr float MAX_LINEAR_SPEED = 0.3f;     // m/s (default speed)
  constexpr float MAX_ANGULAR_SPEED = 1.0f;    // rad/s (default rotation)
  constexpr float TURBO_MULTIPLIER = 2.0f;     // R2: 2x speed
  constexpr float SLOW_MULTIPLIER = 0.3f;      // L2 alone: 0.3x precision
  constexpr float RESPONSE_CURVE = 1.5f;       // Exponential smoothing
  constexpr bool FIELD_CENTRIC = true;         // Global frame movement
}

// ══════════════════════════════════════════════════════════
// PRIVATE STATE VARIABLES
// ══════════════════════════════════════════════════════════

// Button edge detection untuk commands
static bool _prevL1 = false;
static bool _prevL2 = false;
static bool _prevSquare = false;
static bool _prevCross = false;
static bool _prevCircle = false;
static bool _prevTriangle = false;
static bool _prevStart = false;
static bool _prevSelect = false;

// ══════════════════════════════════════════════════════════
// PRIVATE HELPER FUNCTIONS - Manual Control
// ══════════════════════════════════════════════════════════

/**
 * @brief Apply deadzone ke joystick value
 * 
 * Ignore values dalam deadzone dan remap range ke 0-128.
 * 
 * @param value Raw stick value (-128 to 128)
 * @return Processed value dengan deadzone applied (-128 to 128)
 */
static float _applyDeadzone(int8_t value) {
  float val = static_cast<float>(value);
  
  // Check deadzone
  if (fabsf(val) < PS3Config::STICK_DEADZONE) {
    return 0.0f;
  }
  
  // Remap range: deadzone to 128 → 0 to 128
  if (val > 0.0f) {
    return (val - PS3Config::STICK_DEADZONE) / (128.0f - PS3Config::STICK_DEADZONE) * 128.0f;
  } else {
    return (val + PS3Config::STICK_DEADZONE) / (128.0f - PS3Config::STICK_DEADZONE) * 128.0f;
  }
}

/**
 * @brief Apply exponential response curve untuk smooth control
 * 
 * Membuat stick lebih halus di center dan lebih responsive di edges.
 * 
 * @param value Normalized value (-1.0 to 1.0)
 * @param exponent Response curve exponent (1.0 = linear, 2.0 = quadratic)
 * @return Curved value (-1.0 to 1.0)
 */
static float _applyResponseCurve(float value, float exponent) {
  float sign = (value >= 0.0f) ? 1.0f : -1.0f;
  float magnitude = fabsf(value);
  
  // Apply exponential curve
  float curved = powf(magnitude, exponent);
  
  return sign * curved;
}

/**
 * @brief Convert joystick value ke velocity command
 * 
 * Normalize dari -128...128 ke velocity dalam m/s atau rad/s.
 * 
 * @param stickValue Processed stick value (-128 to 128)
 * @param maxVelocity Maximum velocity untuk scaling
 * @param speedMultiplier Mode multiplier (turbo/slow/normal)
 * @return Velocity command (m/s atau rad/s)
 */
static float _stickToVelocity(float stickValue, float maxVelocity, float speedMultiplier) {
  // Normalize ke -1.0 to 1.0
  float normalized = stickValue / 128.0f;
  
  // Apply response curve
  float curved = _applyResponseCurve(normalized, PS3Config::RESPONSE_CURVE);
  
  // Scale ke velocity dengan multiplier
  return curved * maxVelocity * speedMultiplier;
}

// ══════════════════════════════════════════════════════════
// PRIVATE HELPER FUNCTIONS - Commands
// ══════════════════════════════════════════════════════════

/**
 * @brief Detect button press (rising edge)
 * 
 * @param currentState Current button state
 * @param prevState Previous button state (will be updated)
 * @return true if button just pressed (0->1 transition)
 */
static bool _isButtonPressed(bool currentState, bool &prevState) {
  bool pressed = currentState && !prevState;
  prevState = currentState;
  return pressed;
}

/**
 * @brief Check apakah ada command combination yang aktif
 * 
 * Digunakan untuk prioritize command mode over manual control.
 * 
 * @return true jika L1 atau L2 pressed (command mode)
 */
static bool _isCommandModeActive() {
  return ps3ButtonL1 || (ps3ButtonL2 && (ps3ButtonStart || ps3ButtonSelect));
}

// ══════════════════════════════════════════════════════════
// PUBLIC FUNCTIONS - Manual Control
// ══════════════════════════════════════════════════════════

/**
 * @brief Update manual control loop (joystick)
 * 
 * Membaca PS3 joystick input dan mengkonversi menjadi wheel velocities.
 * Dipanggil setiap loop cycle dari main loop.
 * 
 * CONTROL MAPPING:
 * - Left Stick X  → Vy (strafe left/right)
 * - Left Stick Y  → Vx (forward/backward)
 * - Right Stick X → Omega (rotate left/right)
 * - R2 Button     → Turbo mode (2x speed)
 * - L2 Alone      → Slow mode (0.3x speed, precision)
 * 
 * NOTE: Manual control disabled saat command mode aktif (L1/L2 combinations)
 */
void ps3Interface_updateManualControl() {
  // Skip jika PS3 tidak connected
  if (!ps3ControllerConnected) {
    return;
  }
  
  // Skip jika auto-tuning sedang aktif (safety)
  if (autotuning_isActive()) {
    return;
  }
  
  // Skip jika command mode aktif (L1 atau L2+Start/Select pressed)
  if (_isCommandModeActive()) {
    return;
  }
  
  // ════════════════════════════════════════════════════════
  // Step 1: Read joystick values dengan deadzone
  // ════════════════════════════════════════════════════════
  
  float stick_lx = _applyDeadzone(ps3StickLeftX);   // Strafe (left/right)
  float stick_ly = _applyDeadzone(ps3StickLeftY);   // Forward/backward
  float stick_rx = _applyDeadzone(ps3StickRightX);  // Rotation
  
  // ════════════════════════════════════════════════════════
  // Step 2: Determine speed mode (normal/turbo/slow)
  // ════════════════════════════════════════════════════════
  
  float speedMultiplier = 1.0f;  // Normal mode
  
  if (ps3ButtonR2) {
    // Turbo mode - high speed
    speedMultiplier = PS3Config::TURBO_MULTIPLIER;
  } else if (ps3ButtonL2) {
    // Slow mode - precision control (L2 alone, not with other buttons)
    speedMultiplier = PS3Config::SLOW_MULTIPLIER;
  }
  
  // ════════════════════════════════════════════════════════
  // Step 3: Convert stick values ke velocity commands
  // ════════════════════════════════════════════════════════
  
  float vx = _stickToVelocity(stick_ly, PS3Config::MAX_LINEAR_SPEED, speedMultiplier);
  float vy = _stickToVelocity(stick_lx, PS3Config::MAX_LINEAR_SPEED, speedMultiplier);
  float omega = _stickToVelocity(stick_rx, PS3Config::MAX_ANGULAR_SPEED, speedMultiplier);
  
  // ════════════════════════════════════════════════════════
  // Step 4: Convert velocity ke wheel speeds (inverse kinematics)
  // ════════════════════════════════════════════════════════
  
  float wheel_vel[3];  // rad/s
  kinematics_inverseKinematics(vx, vy, omega, wheel_vel, PS3Config::FIELD_CENTRIC);
  
  // ════════════════════════════════════════════════════════
  // Step 5: Convert rad/s ke RPM dan set target ke PID controller
  // ════════════════════════════════════════════════════════
  
  constexpr float RAD_S_TO_RPM = 9.5493f;  // 60/(2*PI)
  
  control_setRpmAllWheels(
    wheel_vel[0] * RAD_S_TO_RPM,  // Motor 1
    wheel_vel[1] * RAD_S_TO_RPM,  // Motor 2
    wheel_vel[2] * RAD_S_TO_RPM   // Motor 3
  );
}

// ══════════════════════════════════════════════════════════
// PUBLIC FUNCTIONS - Commands
// ══════════════════════════════════════════════════════════

/**
 * @brief Handle PS3 controller button commands
 * 
 * Process button combinations untuk auto-tuning dan utility functions.
 * Dipanggil di loop() untuk detect button presses.
 * Non-blocking, hanya execute jika PS3 connected.
 * 
 * COMMAND PRIORITY: Commands override manual control saat L1/L2 pressed.
 */
void ps3Interface_handleCommands() {
  // Skip jika PS3 tidak connected
  if (!ps3ControllerConnected) {
    return;
  }
  
  // Show PS3 status on LCD (non-tuning, non-control mode)
  static uint32_t lastStatusUpdate = 0;
  if (!autotuning_isActive() && !_isCommandModeActive() && 
      (millis() - lastStatusUpdate >= 5000)) {
    lcd_debugMessage("PS3 Ready", "Use Joystick", 0);
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
    autotuning_startAllMotors();
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
}

// ══════════════════════════════════════════════════════════
// PUBLIC FUNCTIONS - Utilities
// ══════════════════════════════════════════════════════════

/**
 * @brief Stop robot (emergency stop)
 * 
 * Set semua motor ke 0 RPM.
 * Dipanggil saat disconnect atau mode switch.
 */
void ps3Interface_stop() {
  control_setRpmAllWheels(0.0f, 0.0f, 0.0f);
}

/**
 * @brief Get current control mode status
 * 
 * @return String: "TURBO", "SLOW", "COMMAND", atau "NORMAL"
 */
const char* ps3Interface_getMode() {
  if (_isCommandModeActive()) return "COMMAND";
  if (ps3ButtonR2) return "TURBO";
  if (ps3ButtonL2) return "SLOW";
  return "NORMAL";
}
