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
  // Initialize serial communication untuk debugging
  Serial.begin(115200);
  delay(100);
  Serial.println("\n\n=== PRC2026 Robot Starting ===");
  
  // Initialize hardware layer
  Serial.print("Initializing motors... ");
  hardware_initializeMotors();
  Serial.println("OK");
  
  Serial.print("Initializing encoders... ");
  hardware_initializeEncoders();
  Serial.println("OK");
  
  Serial.print("Initializing IMU... ");
  if (hardware_initializeImu()) {
    Serial.println("OK");
  } else {
    Serial.println("FAILED! Robot will run without IMU");
  }
  
  Serial.print("Initializing LCD... ");
  hardware_initializeLcd();
  Serial.println("OK");
  
  Serial.print("Initializing PS3... ");
  hardware_initializePs3();
  Serial.println("OK");
  
  // Initialize control layer
  Serial.print("Initializing PID controllers... ");
  control_initializePid();
  Serial.println("OK");
  
  Serial.println("\n=== Robot Ready! ===\n");
  
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
  
  // ══════════════════════════════════════════════════════════
  // CONTROL LOOP (Every Cycle)
  // ══════════════════════════════════════════════════════════
  
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
