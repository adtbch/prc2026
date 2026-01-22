/**
 * ============================================================
 * FILE: ps3controller.ino
 * LAYER: Hardware Abstraction Layer
 * ============================================================
 * 
 * DESCRIPTION:
 * PS3 DualShock controller interface via Bluetooth.
 * Menyediakan joystick dan button inputs untuk manual control.
 * 
 * HARDWARE:
 * - PS3 DualShock 3 controller
 * - ESP32 Bluetooth
 * 
 * PUBLIC FUNCTIONS:
 * - hardware_initializePs3()
 * - hardware_isPs3Connected()
 * - ps3_handleConnection() - Auto-reconnect logic
 * 
 * GLOBALS:
 * - ps3StickLeftX, ps3StickLeftY (joystick values)
 * - ps3Button* (button states)
 * 
 * ============================================================
 */

// ══════════════════════════════════════════════════════════
// CONNECTION STATE TRACKING
// ══════════════════════════════════════════════════════════

// Track koneksi PS3 untuk reconnection logic
static bool ps3WasConnectedBefore = false;
static uint32_t lastReconnectAttempt = 0;
static uint8_t reconnectAttemptCount = 0;
static uint32_t lastReminderMsg = 0;

// ══════════════════════════════════════════════════════════
// CALLBACK FUNCTIONS (Called by PS3 library)
// ══════════════════════════════════════════════════════════

/**
 * @brief Callback saat PS3 controller connected
 */
static void _ps3_onConnect() {
  ps3ControllerConnected = true;
  Ps3.setPlayer(1);  // Set LED indicator ke player 1
  Serial.println("PS3 Controller Connected!");
}

/**
 * @brief Callback saat PS3 controller disconnected
 * 
 * Reset semua input ke neutral/0 untuk safety
 */
static void _ps3_onDisconnect() {
  ps3ControllerConnected = false;
  
  // Reset joysticks
  ps3StickLeftX = 0;
  ps3StickLeftY = 0;
  ps3StickRightX = 0;
  ps3StickRightY = 0;
  
  // Reset buttons
  ps3ButtonX = false;
  ps3ButtonCircle = false;
  ps3ButtonTriangle = false;
  ps3ButtonSquare = false;
  ps3ButtonL1 = false;
  ps3ButtonL2 = false;
  ps3ButtonL3 = false;
  ps3ButtonR1 = false;
  ps3ButtonR2 = false;
  ps3ButtonR3 = false;
  ps3DpadUp = false;
  ps3DpadDown = false;
  ps3DpadLeft = false;
  ps3DpadRight = false;
  ps3ButtonSelect = false;
  ps3ButtonStart = false;
  
  Serial.println("PS3 Controller Disconnected");
}

/**
 * @brief Callback untuk update data dari controller
 * 
 * Dipanggil oleh library saat ada perubahan input.
 * Copy semua data ke global variables.
 */
static void _ps3_onNotify() {
  // Joystick (note: Y axis inverted untuk match robot convention)
  ps3StickLeftX = Ps3.data.analog.stick.lx;
  ps3StickLeftY = -Ps3.data.analog.stick.ly;  // Invert Y (up = positive)
  ps3StickRightX = Ps3.data.analog.stick.rx;
  ps3StickRightY = -Ps3.data.analog.stick.ry;
  
  // Face buttons
  ps3ButtonX = Ps3.data.button.cross;
  ps3ButtonCircle = Ps3.data.button.circle;
  ps3ButtonTriangle = Ps3.data.button.triangle;
  ps3ButtonSquare = Ps3.data.button.square;
  
  // Shoulder buttons
  ps3ButtonL1 = Ps3.data.button.l1;
  ps3ButtonL2 = Ps3.data.button.l2;
  ps3ButtonL3 = Ps3.data.button.l3;
  ps3ButtonR1 = Ps3.data.button.r1;
  ps3ButtonR2 = Ps3.data.button.r2;
  ps3ButtonR3 = Ps3.data.button.r3;
  
  // D-pad
  ps3DpadUp = Ps3.data.button.up;
  ps3DpadDown = Ps3.data.button.down;
  ps3DpadLeft = Ps3.data.button.left;
  ps3DpadRight = Ps3.data.button.right;
  
  // System buttons
  ps3ButtonSelect = Ps3.data.button.select;
  ps3ButtonStart = Ps3.data.button.start;
}

// ══════════════════════════════════════════════════════════
// PUBLIC FUNCTIONS
// ══════════════════════════════════════════════════════════

/**
 * @brief Initialize PS3 controller Bluetooth connection
 * 
 * Note: ESP32 MAC address harus di-pair dengan PS3 controller dulu
 * menggunakan SixaxisPairTool (Windows) atau sixpair (Linux)
 */
void hardware_initializePs3() {
  // Attach callbacks
  Ps3.attach(_ps3_onNotify);
  Ps3.attachOnConnect(_ps3_onConnect);
  Ps3.attachOnDisconnect(_ps3_onDisconnect);
  
  // Begin PS3 Bluetooth
  Ps3.begin();
  
  Serial.println("PS3 Controller initialized. Waiting for connection...");
}

/**
 * @brief Re-initialize PS3 untuk reconnection attempt
 * 
 * Dipanggil saat PS3 disconnect dan ingin reconnect.
 * Soft reset Bluetooth stack tanpa restart ESP32.
 */
void hardware_reinitializePs3() {
  // PS3 library tidak punya built-in reconnect
  // Kita harus end() dan begin() ulang
  
  Serial.println("[PS3] Re-initializing PS3 controller (soft reset)...");
  
  // Re-attach callbacks (ensure callbacks masih active)
  Ps3.attach(_ps3_onNotify);
  Ps3.attachOnConnect(_ps3_onConnect);
  Ps3.attachOnDisconnect(_ps3_onDisconnect);
  
  // Begin lagi (ini akan trigger re-scan)
  Ps3.begin();
  
  Serial.println("[PS3] Soft reset complete - Press PS button to reconnect");
}

/**
 * Full Bluetooth reset (hard reset untuk reconnection failures)
 * Dipanggil setelah multiple soft reset failures
 * 
 * Strategy: Completely restart Bluetooth stack dari scratch
 * WARNING: Lebih agresif dari soft reset, bisa disconnect devices lain
 */
void hardware_fullBluetoothReset() {
  Serial.println("[PS3] ========================================");
  Serial.println("[PS3] FULL BLUETOOTH RESET - Hard restart");
  Serial.println("[PS3] ========================================");
  
  // Step 1: Stop Bluetooth completely
  Serial.println("[PS3] Step 1: Stopping Bluetooth stack...");
  btStop();  // Stop ESP32 Bluetooth stack
  delay(1000);  // Wait for full shutdown
  
  // Step 2: Restart Bluetooth
  Serial.println("[PS3] Step 2: Restarting Bluetooth stack...");
  btStart();  // Restart ESP32 Bluetooth stack
  delay(1000);  // Wait for initialization
  
  // Step 3: Re-initialize PS3 controller
  Serial.println("[PS3] Step 3: Re-initializing PS3 controller...");
  Ps3.attach(_ps3_onNotify);
  Ps3.attachOnConnect(_ps3_onConnect);
  Ps3.attachOnDisconnect(_ps3_onDisconnect);
  
  Ps3.begin();  // Start PS3 library
  delay(500);
  
  Serial.println("[PS3] ========================================");
  Serial.println("[PS3] Full reset complete - Press PS button");
  Serial.println("[PS3] ========================================");
}

// ══════════════════════════════════════════════════════════
// CONNECTION MANAGEMENT
// ══════════════════════════════════════════════════════════

/**
 * Handle PS3 connection state dan reconnection logic
 * Dipanggil setiap loop untuk maintain connection
 * 
 * FEATURES:
 * - Auto-reconnect saat disconnect
 * - Multi-stage reconnect strategy (soft → hard reset)
 * - Periodic status display
 */
void ps3_handleConnection() {
  if (Ps3.isConnected()) {
    // Controller connected - reset state
    ps3WasConnectedBefore = true;
    reconnectAttemptCount = 0;  // Reset attempt counter when connected
    yield();  // Yield to Bluetooth stack
    
  } else {
    // Controller disconnected - attempt reconnection
    _ps3_reconnectLogic();
  }
}

/**
 * Internal: Reconnection logic saat PS3 disconnect
 * 
 * STRATEGY:
 * 1. Soft reset (3x attempts) - re-initialize PS3
 * 2. Hard reset (after 3 fails) - full BT restart
 */
void _ps3_reconnectLogic() {
  // Hanya reconnect jika pernah connected sebelumnya
  if (!ps3WasConnectedBefore) {
    return;
  }
  
  // Check interval untuk reconnect attempt (5 detik)
  if (millis() - lastReconnectAttempt >= 5000) {
    reconnectAttemptCount++;
    
    Serial.printf("[PS3] Disconnected - Reconnect attempt #%d\n", reconnectAttemptCount);
    lcd_debugMessage("PS3 Disconn", "Reconnecting...");
    
    // Strategy 1: Soft reset (first 3 attempts)
    if (reconnectAttemptCount <= 3) {
      hardware_reinitializePs3();
      Serial.println("[PS3] Soft reset - Press PS button on controller");
    }
    // Strategy 2: Hard reset (after 3 failed attempts)
    else if (reconnectAttemptCount == 4) {
      Serial.println("[PS3] Multiple failures - Full BT restart");
      lcd_debugMessage("PS3: Full Reset", "Wait 5s...");
      hardware_fullBluetoothReset();
      reconnectAttemptCount = 0;  // Reset counter after hard reset
    }
    
    lastReconnectAttempt = millis();
  }
  
  // Display periodic reminder (setiap 10 detik)
  _ps3_displayReminder();
}

/**
 * Internal: Display periodic reminder untuk press PS button
 */
void _ps3_displayReminder() {
  if (millis() - lastReminderMsg >= 10000) {  // Every 10 seconds
    lcd_debugMessage("PS3 Offline", "Press PS Btn");
    lastReminderMsg = millis();
  }
}

/**
 * Reset PS3 connection state
 * Berguna untuk force reconnection atau clear state
 */
void ps3_resetConnectionState() {
  ps3WasConnectedBefore = false;
  reconnectAttemptCount = 0;
  lastReconnectAttempt = 0;
  lastReminderMsg = 0;
  Serial.println("[PS3] Connection state reset");
}

/**
 * Get connection attempt count
 * @return Number of reconnection attempts
 */
uint8_t ps3_getReconnectAttempts() {
  return reconnectAttemptCount;
}

/**
 * @brief Check apakah PS3 controller connected
 * 
 * @return true jika connected, false jika tidak
 */
bool hardware_isPs3Connected() {
  return ps3ControllerConnected;
}
