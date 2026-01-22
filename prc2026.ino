/**
 * ============================================================
 * MAIN PROGRAM - Robot Omni 3 Roda PRC 2026
 * ============================================================
 * 
 * DESCRIPTION:
 * Program utama untuk robot omni 3 roda dengan kontrol:
 * - Manual control via PS3 controller (field-centric)
 * - Autonomous waypoint navigation
 * - Closed-loop RPM control dengan odometry
 * 
 * AUTHOR: PRC Team 2026
 * DATE: January 2026
 * 
 * ============================================================
 */

#include "config.h"

void setup() {
  delay(100);
  
  // Initialize LCD first untuk debug display
  hardware_initializeLcd();
  // lcd_debugMessage("PRC2026", "Starting...");
  delay(500);
  
  // Initialize hardware layer
  // lcd_debugInit("Motors", true);
  hardware_initializeMotors();
  
  // lcd_debugInit("Encoders", true);
  hardware_initializeEncoders();
  
  // CRITICAL: Initialize MPU6050 untuk yaw angle
  lcd_debugMessage("Init IMU...", "Please wait");
  bool imuOk = hardware_initializeImu();
  if (imuOk) {
    lcd_debugMessage("IMU OK", "DMP Ready");
    Serial.println("[SETUP] MPU6050 initialized successfully");
  } else {
    lcd_debugMessage("IMU FAILED", "No Yaw data");
    Serial.println("[SETUP] WARNING: MPU6050 init FAILED - Yaw will be 0");
  }
  delay(1500);
  
  // lcd_debugInit("PS3", true);
  hardware_initializePs3();
  
  // Initialize control layer
  // lcd_debugInit("PID Ctrl", true);
  control_initializePid();
  
  // Initialize PID storage system
  // lcd_debugInit("PID Store", true);
  storage_initializePidPreferences();
  
  // Load saved PID parameters dari flash
  lcd_debugMessage("Loading PID", "from flash...");
  storage_loadPidParameters();
  delay(1000);
  
  // Initialize auto-tuning system
  // lcd_debugInit("AutoTune", true);
  autotuning_initialize();
  
  // Display ready status di LCD
  lcd_showMessage("PRC2026", "Ready!");
  delay(2000);
  lcd_clear();
}

void loop() {
  // ══════════════════════════════════════════════════════════
  // SENSOR UPDATES (High Frequency - setiap cycle)
  // ══════════════════════════════════════════════════════════
  
  // Update IMU untuk yaw angle (diperlukan untuk global frame control)
  hardware_updateImu();
  
  // CRITICAL: Periodic IMU health check (every 10 seconds)
  // Re-initialize jika IMU failure detected
  static uint32_t lastImuHealthCheck = 0;
  if (millis() - lastImuHealthCheck >= 10000) {
    extern bool _dmpReady;
    if (!_dmpReady) {
      Serial.println("[HEALTH] IMU not ready - Attempting re-initialization...");
      lcd_debugMessage("IMU Reinit", "Wait...");
      
      if (hardware_initializeImu()) {
        Serial.println("[HEALTH] IMU re-initialized successfully!");
        lcd_debugMessage("IMU OK", "Recovered!");
        delay(1000);
      } else {
        Serial.println("[HEALTH] IMU re-init FAILED - Check wiring!");
      }
    }
    lastImuHealthCheck = millis();
  }
  
  // Update RPM calculation dari encoder pulses
  hardware_updateEncoderRpm();
  // ══════════════════════════════════════════════════════════
  // AUTO-TUNING (If active)
  // ══════════════════════════════════════════════════════════
  
  // Update auto-tuning state machine jika sedang running
  autotuning_update();
  
  // CRITICAL: PS3 event loop untuk maintain connection
  // Tanpa ini, PS3 akan disconnect setelah beberapa detik
  static bool ps3WasConnectedBefore = false;  // Track connection history
  
  if (Ps3.isConnected()) {
    // Event sudah di-handle via callbacks, cukup check connection
    ps3WasConnectedBefore = true;  // Mark that we've been connected
    yield();  // Yield to Bluetooth stack
    
  } else {
    // Disconnected state
    
    // Jika pernah connected sebelumnya, attempt reconnection
    if (ps3WasConnectedBefore) {
      static uint32_t lastReconnectAttempt = 0;
      static uint8_t reconnectAttemptCount = 0;
      
      if (millis() - lastReconnectAttempt >= 5000) {  // Coba setiap 5 detik
        reconnectAttemptCount++;
        
        Serial.printf("[PS3] Disconnected - Reconnect attempt #%d\n", reconnectAttemptCount);
        lcd_debugMessage("PS3 Disconn", "Reconnecting...");
        
        // Strategy 1: Re-initialize PS3 (first 3 attempts)
        if (reconnectAttemptCount <= 3) {
          hardware_reinitializePs3();
          Serial.println("[PS3] Soft reset - Press PS button on controller");
        }
        // Strategy 2: Full Bluetooth restart (after 3 failed attempts)
        else if (reconnectAttemptCount == 4) {
          Serial.println("[PS3] Multiple failures - Full BT restart");
          lcd_debugMessage("PS3: Full Reset", "Wait 5s...");
          hardware_fullBluetoothReset();
          reconnectAttemptCount = 0;  // Reset counter
        }
        
        lastReconnectAttempt = millis();
      }
      
      // Display periodic reminder
      static uint32_t lastReminderMsg = 0;
      if (millis() - lastReminderMsg >= 10000) {  // Every 10 seconds
        lcd_debugMessage("PS3 Offline", "Press PS Btn");
        lastReminderMsg = millis();
      }
    }
  }
  
  // Handle PS3 controller commands untuk tuning (non-blocking)
  handlePs3Commands();
  
  // Handle serial commands jika UART available (non-blocking)
  // handleSerialCommands();  // DISABLED - UART pins dipakai untuk keperluan lain
  
  // ══════════════════════════════════════════════════════════
  // CONTROL LOOP (Every Cycle)
  // ══════════════════════════════════════════════════════════
  
  // PID controller update (CRITICAL: Must run every loop!)
  // Process current RPM targets dengan encoder feedback
  control_updateRpmControl();
  
  // TODO: Implement manual control dan navigation
  // navigation_updateManualControl();
  // navigation_runWaypointSequence();
  
  // ══════════════════════════════════════════════════════════
  // DISPLAY UPDATE (Low Frequency - 5Hz / 200ms)
  // ══════════════════════════════════════════════════════════
  static uint32_t lastDisplayUpdate = 0;
  if (millis() - lastDisplayUpdate >= Config::DISPLAY_UPDATE_MS) {
    lcd_displayRobotStatus();
    lastDisplayUpdate = millis();
  }
  
  // Debug print ke Serial (optional - comment jika tidak perlu)
  #ifdef DEBUG_PRINT_ENABLED
  static uint32_t lastDebugPrint = 0;
  if (millis() - lastDebugPrint >= 500) {
    debug_printEncoderRpm();
    lastDebugPrint = millis();
  }
  #endif
}
