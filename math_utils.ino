/**
 * ============================================================
 * FILE: math_utils.ino
 * LAYER: Utility Functions
 * ============================================================
 * 
 * DESCRIPTION:
 * Helper functions untuk perhitungan matematik yang umum dipakai.
 * 
 * FUNCTIONS:
 * - Angle wrapping dan normalisasi
 * - Value clamping
 * - Vector normalization
 * 
 * ============================================================
 */

/**
 * @brief Wrap angle ke range 0-360 degrees
 * 
 * @param angleDeg Angle dalam degrees (bisa di luar range)
 * @return Wrapped angle (0-360)
 */
float math_wrapAngle360(float angleDeg) {
  while (angleDeg < 0.0f) angleDeg += 360.0f;
  while (angleDeg >= 360.0f) angleDeg -= 360.0f;
  return angleDeg;
}

/**
 * @brief Calculate shortest angle error (signed)
 * 
 * Return nilai [-180, 180] yang merepresentasikan
 * arah rotasi terpendek dari current ke target.
 * 
 * Contoh:
 * - current=10°, target=350° → error=-20° (putar kiri 20°)
 * - current=350°, target=10° → error=+20° (putar kanan 20°)
 * 
 * @param targetDeg Target angle (0-360)
 * @param currentDeg Current angle (0-360)
 * @return Shortest error (-180 to +180)
 */
float math_calculateAngleError(float targetDeg, float currentDeg) {
  float error = math_wrapAngle360(targetDeg) - math_wrapAngle360(currentDeg);
  
  // Wrap ke range [-180, 180]
  if (error > 180.0f) error -= 360.0f;
  if (error < -180.0f) error += 360.0f;
  
  return error;
}

/**
 * @brief Normalize angle ke range -PI to PI (radian)
 * 
 * @param angleRad Angle dalam radian
 * @return Normalized angle (-PI to PI)
 */
float math_normalizeAngleRad(float angleRad) {
  while (angleRad > PI) angleRad -= 2.0f * PI;
  while (angleRad < -PI) angleRad += 2.0f * PI;
  return angleRad;
}

/**
 * @brief Clamp float value ke range [min, max]
 * 
 * @param value Value to clamp
 * @param minVal Minimum value
 * @param maxVal Maximum value
 * @return Clamped value
 */
float math_clampValue(float value, float minVal, float maxVal) {
  if (value < minVal) return minVal;
  if (value > maxVal) return maxVal;
  return value;
}

/**
 * @brief Clamp integer value ke range [min, max]
 * 
 * @param value Value to clamp
 * @param minVal Minimum value
 * @param maxVal Maximum value
 * @return Clamped value
 */
int math_clampInt(int value, int minVal, int maxVal) {
  if (value < minVal) return minVal;
  if (value > maxVal) return maxVal;
  return value;
}

/**
 * @brief Calculate 2D vector magnitude
 * 
 * Menghitung panjang vektor sqrt(x^2 + y^2).
 * Optimized untuk menghindari duplicate sqrt pattern.
 * 
 * @param x X component
 * @param y Y component
 * @return Magnitude of vector
 */
float math_vectorMagnitude(float x, float y) {
  return sqrtf(x * x + y * y);
}

/**
 * @brief Normalize 3 values agar magnitude max tidak exceed limit
 * 
 * Berguna untuk wheel speed normalization saat total command
 * melebihi batas hardware.
 * 
 * Contoh:
 * Input: a=800, b=900, c=700, limit=600
 * Max = 900, scale = 600/900 = 0.667
 * Output: a=533, b=600, c=467
 * 
 * @param a First value (modified in-place)
 * @param b Second value (modified in-place)
 * @param c Third value (modified in-place)
 * @param maxMagnitude Maximum allowed magnitude
 */
void math_normalizeThreeValues(float& a, float& b, float& c, float maxMagnitude) {
  // Find maximum absolute value
  float maxAbs = fabsf(a);
  float temp = fabsf(b);
  if (temp > maxAbs) maxAbs = temp;
  temp = fabsf(c);
  if (temp > maxAbs) maxAbs = temp;
  
  // Scale down jika exceed limit
  if (maxAbs > maxMagnitude && maxAbs > 0.0001f) {
    float scale = maxMagnitude / maxAbs;
    a *= scale;
    b *= scale;
    c *= scale;
  }
}
