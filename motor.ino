/**
 * ============================================================
 * FILE: motor.ino
 * LAYER: Hardware Abstraction Layer
 * ============================================================
 * 
 * DESCRIPTION:
 * Mengontrol 4 motor DC menggunakan 2 driver L298N.
 * Menyediakan PWM control dengan deadband compensation.
 * 
 * MOTOR MAPPING:
 * - Motor 1, 3, 4: Untuk gerak robot (omni 3-wheel)
 * - Motor 2: Untuk mechanism lift atau unused
 * 
 * HARDWARE:
 * - L298N Module 1: Motor 1 & 2
 * - L298N Module 2: Motor 3 & 4
 * - ESP32 PWM: 10-bit (0-1023), 15kHz frequency
 * 
 * PUBLIC FUNCTIONS:
 * - hardware_initializeMotors()
 * - hardware_setMotorPwm(motorId, pwm)
 * - hardware_setAllMotorsPwm(m1, m2, m3, m4)
 * - hardware_stopAllMotors()
 * 
 * ============================================================
 */


/**
 * @brief Clamp PWM value ke range yang valid
 * 
 * Memastikan PWM tidak exceed batas hardware (0-1023)
 * 
 * @param pwm PWM value (bisa negatif untuk reverse)
 * @return Clamped PWM value
 */
static inline int _motor_clampPwm(int pwm) {
  if (pwm > Pwm::MAX_VALUE) return Pwm::MAX_VALUE;
  if (pwm < -Pwm::MAX_VALUE) return -Pwm::MAX_VALUE;
  return pwm;
}

/**
 * @brief Setup satu PWM channel untuk motor
 * 
 * Configure LEDC peripheral ESP32 untuk generate PWM signal
 * 
 * @param channel PWM channel number (0-15)
 * @param pinEnable GPIO pin untuk enable/PWM
 */
static void _motor_setupPwmChannel(uint8_t channel, uint8_t pinEnable) {
  ledcAttach(pinEnable, Pwm::FREQUENCY_HZ, Pwm::RESOLUTION_BITS);
  ledcWrite(pinEnable, 0);  // Start dengan PWM = 0 (motor stop)
}

/**
 * @brief Kontrol satu motor L298N dengan PWM dan direction
 * 
 * L298N control:
 * - PWM > 0: IN1=HIGH, IN2=LOW (forward)
 * - PWM < 0: IN1=LOW, IN2=HIGH (backward)  
 * - PWM = 0: IN1=LOW, IN2=LOW (coast stop)
 * 
 * @param pwmChannel PWM channel untuk enable pin
 * @param pinIn1 GPIO untuk direction input 1
 * @param pinIn2 GPIO untuk direction input 2
 * @param pwm PWM value (-1023 to +1023)
 */
static void _motor_setL298nPwm(uint8_t pwmChannel, uint8_t pinEnable, uint8_t pinIn1, uint8_t pinIn2, int pwm) {
  int pwmClamped = _motor_clampPwm(pwm);
  
  if (pwmClamped > 0) {
    // Forward direction
    digitalWrite(pinIn1, HIGH);
    digitalWrite(pinIn2, LOW);
    ledcWrite(pinEnable, pwmClamped);
    
  } else if (pwmClamped < 0) {
    // Backward direction
    digitalWrite(pinIn1, LOW);
    digitalWrite(pinIn2, HIGH);
    ledcWrite(pinEnable, -pwmClamped);  // Absolute value
    
  } else {
    // Stop (coast mode - motor bebas berputar)
    digitalWrite(pinIn1, LOW);
    digitalWrite(pinIn2, LOW);
    ledcWrite(pinEnable, 0);
    
    // Alternative brake mode (motor di-short, resist rotation):
    // digitalWrite(pinIn1, HIGH);
    // digitalWrite(pinIn2, HIGH);
    // ledcWrite(pwmChannel, 0);
  }
}

/**
 * @brief Initialize semua motor dan PWM channels
 * 
 * Setup pin modes dan PWM channels untuk 4 motor.
 * Setelah init, semua motor dalam keadaan stop.
 */
void hardware_initializeMotors() {
  // Setup pin mode untuk direction control (IN1, IN2)
  pinMode(Pin::MOTOR1_A, OUTPUT);
  pinMode(Pin::MOTOR1_B, OUTPUT);
  pinMode(Pin::MOTOR2_A, OUTPUT);
  pinMode(Pin::MOTOR2_B, OUTPUT);
  pinMode(Pin::MOTOR3_A, OUTPUT);
  pinMode(Pin::MOTOR3_B, OUTPUT);
  pinMode(Pin::MOTOR4_A, OUTPUT);
  pinMode(Pin::MOTOR4_B, OUTPUT);
  
  // Setup PWM channels untuk enable pins
  _motor_setupPwmChannel(Pwm::MOTOR1_CHANNEL, Pin::MOTOR1_EN);
  _motor_setupPwmChannel(Pwm::MOTOR2_CHANNEL, Pin::MOTOR2_EN);
  _motor_setupPwmChannel(Pwm::MOTOR3_CHANNEL, Pin::MOTOR3_EN);
  _motor_setupPwmChannel(Pwm::MOTOR4_CHANNEL, Pin::MOTOR4_EN);
  
  // Stop semua motor sebagai safety
  hardware_stopAllMotors();
}

/**
 * @brief Set PWM untuk satu motor
 * 
 * @param motorId Motor ID (0-3 untuk motor 1-4)
 * @param pwm PWM value (-1023 to +1023)
 *            Positive = forward, Negative = backward
 */
void hardware_setMotorPwm(uint8_t motorId, int16_t pwm) {
  switch (motorId) {
    case 0:  // Motor 1
      _motor_setL298nPwm(Pwm::MOTOR1_CHANNEL, Pin::MOTOR1_EN, Pin::MOTOR1_A, Pin::MOTOR1_B, pwm);
      break;
      
    case 1:  // Motor 2
      _motor_setL298nPwm(Pwm::MOTOR2_CHANNEL, Pin::MOTOR2_EN, Pin::MOTOR2_A, Pin::MOTOR2_B, pwm);
      break;
      
    case 2:  // Motor 3
      _motor_setL298nPwm(Pwm::MOTOR3_CHANNEL, Pin::MOTOR3_EN, Pin::MOTOR3_A, Pin::MOTOR3_B, pwm);
      break;
      
    case 3:  // Motor 4
      _motor_setL298nPwm(Pwm::MOTOR4_CHANNEL, Pin::MOTOR4_EN, Pin::MOTOR4_A, Pin::MOTOR4_B, pwm);
      break;
      
    default:
      // Invalid motor ID - safety: do nothing
      break;
  }
}

/**
 * @brief Set PWM untuk semua motor sekaligus
 * 
 * Lebih efisien daripada call hardware_setMotorPwm() 4 kali
 * 
 * @param m1 PWM motor 1 (omni wheel depan)
 * @param m2 PWM motor 2 (lift/unused)
 * @param m3 PWM motor 3 (omni wheel kiri belakang)
 * @param m4 PWM motor 4 (omni wheel kanan belakang)
 */
void hardware_setAllMotorsPwm(int16_t m1, int16_t m2, int16_t m3, int16_t m4) {
  _motor_setL298nPwm(Pwm::MOTOR1_CHANNEL, Pin::MOTOR1_EN, Pin::MOTOR1_A, Pin::MOTOR1_B, m1);
  _motor_setL298nPwm(Pwm::MOTOR2_CHANNEL, Pin::MOTOR2_EN, Pin::MOTOR2_A, Pin::MOTOR2_B, m2);
  _motor_setL298nPwm(Pwm::MOTOR3_CHANNEL, Pin::MOTOR3_EN, Pin::MOTOR3_A, Pin::MOTOR3_B, m3);
  _motor_setL298nPwm(Pwm::MOTOR4_CHANNEL, Pin::MOTOR4_EN, Pin::MOTOR4_A, Pin::MOTOR4_B, m4);
}

/**
 * @brief Emergency stop - matikan semua motor
 * 
 * Gunakan untuk safety stop atau saat error
 */
void hardware_stopAllMotors() {
  hardware_setAllMotorsPwm(0, 0, 0, 0);
}
