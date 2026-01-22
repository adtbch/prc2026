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
  
  // Initialize kinematics system untuk field-centric control
  lcd_debugMessage("Init Kinematic", "Kalman filter");
  kinematics_initialize();
  delay(500);
  
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
  
  // Update RPM calculation dari encoder pulses
  hardware_updateEncoderRpm();
  
  // Update odometry dengan Kalman filter fusion (encoder + IMU)
  // Konversi RPM ke rad/s dan hitung dt
  static uint32_t lastOdoUpdate = 0;
  uint32_t now = millis();
  float dt = (now - lastOdoUpdate) / 1000.0f;  // Convert ms to seconds
  
  if (dt > 0.001f) {  // Update hanya jika dt > 1ms
    // Get wheel velocities dalam rad/s dari encoder RPM
    float wheel_vel[3];
    wheel_vel[0] = hardware_getEncoderRpm(0) * 0.104719755f;  // RPM to rad/s (2π/60)
    wheel_vel[1] = hardware_getEncoderRpm(1) * 0.104719755f;
    wheel_vel[2] = hardware_getEncoderRpm(2) * 0.104719755f;
    
    kinematics_updateOdometry(wheel_vel, dt);
    lastOdoUpdate = now;
  }
  // ══════════════════════════════════════════════════════════
  // AUTO-TUNING (If active)
  // ══════════════════════════════════════════════════════════
  
  // Update auto-tuning state machine jika sedang running
  autotuning_update();
  
  // ══════════════════════════════════════════════════════════
  // PS3 CONNECTION MANAGEMENT
  // ══════════════════════════════════════════════════════════
  
  // Handle PS3 connection state dan auto-reconnect
  ps3_handleConnection();
  
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
