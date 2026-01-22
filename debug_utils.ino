/**
 * ============================================================
 * FILE: debug_utils.ino
 * LAYER: Utility Functions
 * ============================================================
 * 
 * DESCRIPTION:
 * Helper functions untuk debugging via Serial dan LCD.
 * 
 * FEATURES:
 * - Formatted Serial.print untuk debugging
 * - Vector printing
 * - Odometry monitoring
 * 
 * ============================================================
 */


/**
 * @brief Print label dan float value ke Serial
 * 
 * Format: "label: value"
 * 
 * @param label Label text
 * @param value Float value
 */
void debug_printValue(const char* label, float value) {
  Serial.print(label);
  Serial.print(": ");
  Serial.println(value);
}

/**
 * @brief Print 3D vector ke Serial
 * 
 * Format: "label: (x, y, z)"
 * 
 * @param label Label text
 * @param x X component
 * @param y Y component
 * @param z Z component
 */
void debug_printVector3(const char* label, float x, float y, float z) {
  Serial.print(label);
  Serial.print(": (");
  Serial.print(x);
  Serial.print(", ");
  Serial.print(y);
  Serial.print(", ");
  Serial.print(z);
  Serial.println(")");
}

/**
 * @brief Print encoder RPM ke Serial
 * 
 * Menampilkan RPM semua wheels
 */
void debug_printEncoderRpm() {
  Serial.print("RPM - M1:");
  Serial.print(encoderRpm[0], 1);
  Serial.print(" M3:");
  Serial.print(encoderRpm[2], 1);
  Serial.print(" M4:");
  Serial.println(encoderRpm[3], 1);
}
