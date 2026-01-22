/**
 * ============================================================
 * FILE: pid.ino
 * LAYER: Control Algorithm Layer
 * ============================================================
 * 
 * DESCRIPTION:
 * Generic PID controller dengan multiple channels.
 * Digunakan untuk:
 * - RPM control (wheel speed, closed-loop)
 * - Yaw hold (heading control)
 * - Position control (X, Y, Yaw navigation)
 * 
 * FEATURES:
 * - Multi-channel PID (8 channels)
 * - Configurable gains per channel
 * - Anti-windup (integral clamping)
 * - Deadband support
 * 
 * PUBLIC FUNCTIONS:
 * - control_initializePid()
 * - control_computePid(channel, setpoint, input, gains)
 * - control_resetPidChannel(channel)
 * - control_setRpmAllWheels(rpm1, rpm2, rpm3)
 * 
 * ============================================================
 */

// ══════════════════════════════════════════════════════════
// PID CHANNEL DEFINITIONS
// ══════════════════════════════════════════════════════════

namespace PidChannel {
  constexpr uint8_t WHEEL1_RPM = 0;        // Motor 1 RPM control
  constexpr uint8_t WHEEL2_RPM = 1;        // Motor 3 RPM control  
  constexpr uint8_t WHEEL3_RPM = 2;        // Motor 4 RPM control
  constexpr uint8_t YAW_HOLD = 3;          // Yaw heading hold
  constexpr uint8_t POSITION_X = 4;        // Position control X
  constexpr uint8_t POSITION_Y = 5;        // Position control Y
  constexpr uint8_t POSITION_YAW = 6;      // Position control Yaw
  constexpr uint8_t RESERVED = 7;          // Reserved for future use
}

// ══════════════════════════════════════════════════════════
// PID DATA STRUCTURES
// ══════════════════════════════════════════════════════════

/**
 * Struktur untuk menyimpan state PID per channel
 */
struct PidState {
  float error;              // Current error
  float integral;           // Accumulated integral
  float derivative;         // Rate of change
  float previousError;      // Previous error (untuk derivative)
};

// Total 8 PID channels
constexpr uint8_t PID_CHANNEL_COUNT = 8;
static PidState _pidState[PID_CHANNEL_COUNT];

// ══════════════════════════════════════════════════════════
// PUBLIC FUNCTIONS
// ══════════════════════════════════════════════════════════

/**
 * @brief Initialize semua PID channels
 * 
 * Reset semua state ke 0
 */
void control_initializePid() {
  for (uint8_t i = 0; i < PID_CHANNEL_COUNT; i++) {
    control_resetPidChannel(i);
  }
}

/**
 * @brief Reset satu PID channel ke initial state
 * 
 * @param channel Channel ID (0-7)
 */
void control_resetPidChannel(uint8_t channel) {
  if (channel >= PID_CHANNEL_COUNT) return;
  
  _pidState[channel].error = 0.0f;
  _pidState[channel].integral = 0.0f;
  _pidState[channel].derivative = 0.0f;
  _pidState[channel].previousError = 0.0f;
}

/**
 * @brief Compute PID output untuk satu channel
 * 
 * Formula:
 * output = Kp*error + Ki*integral + Kd*derivative
 * 
 * Features:
 * - Deadband: ignore error < deadband
 * - Anti-windup: clamp integral ke ±(outputLimit/Ki)
 * - Output limit: clamp output ke ±outputLimit
 * 
 * @param channel Channel ID (0-7)
 * @param setpoint Target value (desired state)
 * @param input Current value (measured state)
 * @param gains PID gains (Kp, Ki, Kd, outputLimit, deadband)
 * @return PID output (clamped ke ±outputLimit)
 */
float control_computePid(uint8_t channel, float setpoint, float input, const PidGains& gains) {
  if (channel >= PID_CHANNEL_COUNT) return 0.0f;
  
  PidState& state = _pidState[channel];
  
  // Calculate error
  state.error = setpoint - input;
  
  // Apply deadband (ignore small errors)
  if (fabsf(state.error) < gains.deadband) {
    state.error = 0.0f;
    // Note: Tidak reset integral saat deadband, biar smooth transition
  }
  
  // Update integral dengan anti-windup
  state.integral += state.error;
  
  // Clamp integral untuk prevent windup
  if (gains.ki > 0.0001f) {  // Avoid divide by zero
    float maxIntegral = gains.outputLimit / gains.ki;
    state.integral = math_clampValue(state.integral, -maxIntegral, maxIntegral);
  }
  
  // Calculate derivative
  state.derivative = state.error - state.previousError;
  state.previousError = state.error;
  
  // Compute PID output
  float output = gains.kp * state.error
               + gains.ki * state.integral
               + gains.kd * state.derivative;
  
  // Clamp output
  output = math_clampValue(output, -gains.outputLimit, gains.outputLimit);
  
  return output;
}

// ══════════════════════════════════════════════════════════
// RPM CONTROL (Closed-Loop Speed Control)
// ══════════════════════════════════════════════════════════

/**
 * @brief Control 3 wheels dengan target RPM (closed-loop)
 * 
 * Menggunakan PID untuk match target RPM dengan feedback dari encoder.
 * Output: PWM command ke motor.
 * 
 * Features:
 * - Per-wheel RPM scaling (kalibrasi)
 * - Minimum PWM compensation (deadband motor)
 * - Direction handling (positive/negative RPM)
 * 
 * @param rpmW1 Target RPM untuk wheel 1 (Motor 1)
 * @param rpmW2 Target RPM untuk wheel 2 (Motor 3)
 * @param rpmW3 Target RPM untuk wheel 3 (Motor 4)
 */
void control_setRpmAllWheels(float rpmW1, float rpmW2, float rpmW3) {
  // Apply per-wheel scaling (kalibrasi untuk straight line)
  rpmW1 *= Tuning::RPM_WHEEL1_SCALE;
  rpmW2 *= Tuning::RPM_WHEEL2_SCALE;
  rpmW3 *= Tuning::RPM_WHEEL3_SCALE;
  
  // Array untuk simplify loop processing
  float targetRpm[3] = {rpmW1, rpmW2, rpmW3};
  uint8_t motorIds[3] = {0, 2, 3};  // Motor 1, 3, 4 (skip motor 2)
  uint8_t pidChannels[3] = {
    PidChannel::WHEEL1_RPM,
    PidChannel::WHEEL2_RPM,
    PidChannel::WHEEL3_RPM
  };
  uint16_t minPwm[3] = {
    Tuning::MIN_PWM_WHEEL1,
    Tuning::MIN_PWM_WHEEL2,
    Tuning::MIN_PWM_WHEEL3
  };
  
  // Process each wheel
  for (uint8_t i = 0; i < 3; i++) {
    float target = targetRpm[i];
    
    // Jika target = 0, stop motor dan reset PID
    if (fabsf(target) < 0.1f) {
      hardware_setMotorPwm(motorIds[i], 0);
      control_resetPidChannel(pidChannels[i]);
      continue;
    }
    
    // Get current RPM dari encoder (selalu positif)
    float currentRpm = hardware_getEncoderRpm(motorIds[i]);
    
    // PID control magnitude (absolute value)
    float targetMagnitude = fabsf(target);
    float outputMagnitude = control_computePid(
      pidChannels[i],
      targetMagnitude,
      currentRpm,
      Tuning::RPM_WHEEL
    );
    
    // Apply minimum PWM untuk overcome motor deadband
    // Note: Hanya apply jika target cukup besar (> 20 RPM)
    if (targetMagnitude > 20.0f && outputMagnitude > 0.0f && outputMagnitude < minPwm[i]) {
      outputMagnitude = minPwm[i];
    }
    
    // Apply direction sign
    int16_t pwm = (target > 0.0f) ? (int16_t)outputMagnitude : -(int16_t)outputMagnitude;
    
    // Send PWM ke motor
    hardware_setMotorPwm(motorIds[i], pwm);
  }
}
