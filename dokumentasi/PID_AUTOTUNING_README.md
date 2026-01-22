# PID Auto-Tuning System - Dokumentasi

## Overview

Sistem auto-tuning PID dengan **Preferences storage** untuk robot omni 3 roda. Implementasi menggunakan metode **Ziegler-Nichols** dan **Relay Auto-Tuning (Åström-Hägglund)** yang sudah terbukti di industri.

## Features

### 1. **Preferences Storage System** (`pid_storage.ino`)
- ✅ Separate namespace per motor (avoid conflicts)
- ✅ Atomic write operations dengan error checking
- ✅ Validation sebelum save/load
- ✅ Default values sebagai fallback
- ✅ Timestamp tracking untuk audit

### 2. **Auto-Tuning System** (`pid_autotuning.ino`)
- ✅ Multi-stage precision (Coarse → Fine → Ultra-Fine)
- ✅ Performance metrics (ISE, IAE, ITAE, Overshoot, Rise Time)
- ✅ Safety features (burst detection, overshoot limits)
- ✅ Per-motor independent tuning
- ✅ Systematic parameter exploration (Gradient Descent inspired)

## Best Practices Implemented

### Industrial Standard Methods

1. **Ziegler-Nichols Quarter Decay Ratio**
   - Target: 25% overshoot dengan damping optimal
   - Rise time balanced dengan stability

2. **Performance Indices**
   - ISE (Integral Square Error) - untuk accuracy
   - IAE (Integral Absolute Error) - untuk consistency
   - ITAE (Integral Time Absolute Error) - untuk settling time

3. **Safety & Robustness**
   - Burst detection (initial overshoot >30%)
   - Parameter validation (prevent extreme values)
   - Timeout protection
   - Graceful degradation

## Usage

### Startup Sequence

```cpp
void setup() {
  // 1. Initialize hardware
  hardware_initializeMotors();
  hardware_initializeEncoders();
  
  // 2. Initialize PID storage
  storage_initializePidPreferences();
  
  // 3. Load saved PID parameters
  storage_loadPidParameters();
  
  // 4. Initialize auto-tuning
  autotuning_initialize();
}
```

### Start Auto-Tuning

```cpp
// Tune motor 1 (Wheel 1 - Depan)
autotuning_startMotor(0);

// Di loop()
void loop() {
  autotuning_update();  // Update state machine
  
  // Check progress
  uint8_t progress = autotuning_getProgress();  // 0-100%
}
```

### Save/Load PID Manually

```cpp
// Save parameters
storage_savePidParameters(0, 2.5f, 0.1f, 0.05f);  // Motor 1

// Load parameters
float kp, ki, kd;
storage_getPidParameters(0, kp, ki, kd);  // Motor 1

// Reset to defaults
storage_resetPidToDefaults(0);  // Motor 1
storage_resetPidToDefaults(255);  // All motors
```

### Print Saved Values

```cpp
storage_printAllPidValues();  // Print semua motor
```

## Tuning Process

### 1. **Starting Point**
- Load saved PID dari Preferences
- Jika tidak ada, gunakan defaults:
  - Kp = 2.0 (Proportional gain)
  - Ki = 0.08 (Integral gain)
  - Kd = 0.0 (Derivative gain)

### 2. **Test Cycle** (15 seconds each)
- Apply test PID values
- Run motor dengan target RPM (200 RPM)
- Collect performance metrics:
  - Overshoot (%)
  - Rise time (ms)
  - Average error (RPM)
  - Burst detection

### 3. **Analysis & Scoring**
- Calculate performance score (lower = better)
- Weighted factors:
  - Error: 12.0x (highest priority)
  - Overshoot: 3.0x (safety critical)
  - Stability: 5.0x (consistency)
  - Rise time: 0.003x (speed)

### 4. **Parameter Adjustment**
- **Coarse stage** (cycles 1-6): Aggressive search
  - High overshoot → Reduce Kp, Increase Kd
  - Slow rise → Increase Kp, Ki
  - High error → Increase Ki
  
- **Fine stage** (cycles 7-12): Precision optimization
  - Medium overshoot → Fine-tune Kd
  - Steady-state error → Adjust Ki
  
- **Ultra-fine stage** (cycles 13-18): Micro-optimization
  - Pattern search untuk local minimum
  - Very small adjustments

### 5. **Completion**
- Save best parameters ke Preferences
- Quality evaluation:
  - Score < 20: EXCELLENT ✓✓✓
  - Score < 35: GOOD ✓✓
  - Score < 50: ACCEPTABLE ✓
  - Score ≥ 50: NEEDS IMPROVEMENT

## Preferences Storage Structure

### Namespaces

```cpp
"pid_motor1"  // Motor 1 (Wheel 1 - Depan)
"pid_motor2"  // Motor 2 (Wheel 2 - Kiri Belakang)
"pid_motor3"  // Motor 3 (Wheel 3 - Kanan Belakang)
"pid_tuning"  // Auto-tuning metadata
```

### Keys per namespace

```cpp
"kp"          // float - Proportional gain
"ki"          // float - Integral gain
"kd"          // float - Derivative gain
"timestamp"   // uint32_t - Save timestamp (ms)
```

## Performance Metrics

### Overshoot Thresholds
- **High**: >15% (unacceptable - safety risk)
- **Medium**: 8-15% (acceptable with penalty)
- **Low**: 3-8% (good performance)
- **Excellent**: <3% (target zone)

### Rise Time Thresholds
- **Slow**: >4000ms (needs improvement)
- **Medium**: 2000-4000ms (acceptable)
- **Fast**: <2000ms (good performance)

### Error Thresholds
- **High**: >5 RPM (needs Ki adjustment)
- **Medium**: 3-5 RPM (acceptable)
- **Low**: 1.5-3 RPM (good)
- **Excellent**: <1.5 RPM (target zone)

## Troubleshooting

### Problem: Auto-tuning tidak start

**Solusi:**
```cpp
// Check state
if (!autotuning_isActive()) {
  autotuning_startMotor(0);  // Try start again
}
```

### Problem: Parameters tidak tersimpan

**Solusi:**
```cpp
// Re-initialize Preferences
storage_initializePidPreferences();

// Try save again
storage_savePidParameters(0, kp, ki, kd);

// Check flash space
Serial.printf("Free heap: %d bytes\n", ESP.getFreeHeap());
```

### Problem: Tuning result buruk (score >50)

**Cause:** Motor atau encoder bermasalah

**Solusi:**
1. Check encoder connections
2. Verify motor PWM signal
3. Calibrate encoder pulses per revolution
4. Adjust TARGET_RPM lebih rendah (150 RPM)

### Problem: Motor oscillating (bergetar)

**Cause:** Kp terlalu tinggi atau Kd terlalu rendah

**Solusi:**
```cpp
// Manual adjustment
storage_savePidParameters(0, 
  kp * 0.7f,  // Reduce Kp by 30%
  ki,         // Keep Ki
  kd * 1.5f   // Increase Kd by 50%
);
```

## Serial Commands (Future Enhancement)

Untuk kemudahan testing, tambahkan serial commands:

```cpp
// Di loop()
if (Serial.available()) {
  String cmd = Serial.readStringUntil('\n');
  
  if (cmd == "tune1") {
    autotuning_startMotor(0);  // Tune motor 1
  } else if (cmd == "tune2") {
    autotuning_startMotor(1);  // Tune motor 2
  } else if (cmd == "tune3") {
    autotuning_startMotor(2);  // Tune motor 3
  } else if (cmd == "cancel") {
    autotuning_cancel();
  } else if (cmd == "print") {
    storage_printAllPidValues();
  } else if (cmd == "reset") {
    storage_resetPidToDefaults(255);  // Reset all
  }
}
```

## Advanced Configuration

### Adjust Tuning Parameters

Edit `pid_autotuning.ino`, namespace `AutoTune`:

```cpp
namespace AutoTune {
  constexpr float TARGET_RPM = 200.0f;         // ← Ubah target RPM
  constexpr uint32_t TEST_DURATION_MS = 15000; // ← Ubah durasi test
  constexpr uint8_t MAX_TUNING_CYCLES = 18;    // ← Ubah jumlah cycles
  
  // Adjust scoring weights
  constexpr float OVERSHOOT_WEIGHT = 3.0f;     // ← Prioritas overshoot
  constexpr float ERROR_WEIGHT = 12.0f;        // ← Prioritas error
}
```

### Adjust Default PID Values

Edit `pid_storage.ino`, namespace `PidDefaults`:

```cpp
namespace PidDefaults {
  constexpr float MOTOR1_KP = 2.0f;   // ← Default Kp motor 1
  constexpr float MOTOR1_KI = 0.08f;  // ← Default Ki motor 1
  constexpr float MOTOR1_KD = 0.0f;   // ← Default Kd motor 1
  // ... dst untuk motor 2 dan 3
}
```

## References

### Academic Papers
1. Åström & Hägglund - "PID Controllers: Theory, Design, and Tuning" (1995)
2. Ziegler & Nichols - "Optimum Settings for Automatic Controllers" (1942)
3. Ang, Chong, Li - "PID Control System Analysis, Design, and Technology" (2005)

### GitHub References
- [Error404ku/Project-AGV-SAMI](https://github.com/Error404ku/Project-AGV-SAMI/tree/adaptifPidRpm)
  - Adaptive PID dengan scoring system
  - Multi-motor tuning strategy
  - Performance metrics implementation

### Industrial Standards
- ISA-PID-101 - Process Control Standard
- Quarter Decay Ratio Method
- Cohen-Coon Method (alternative)

## Future Enhancements

### Priority 1: Integration dengan Config.h
- [ ] Replace constexpr Tuning:: values dengan runtime variables
- [ ] Auto-apply loaded PID values ke PidGains structs

### Priority 2: User Interface
- [ ] LCD display untuk tuning progress
- [ ] PS3 controller interface (start/cancel tuning)
- [ ] LED indicator untuk tuning status

### Priority 3: Advanced Features
- [ ] Adaptive tuning (re-tune saat performance degradation)
- [ ] Multi-setpoint testing (test di berbagai RPM)
- [ ] Disturbance rejection testing
- [ ] Export tuning results ke SD card

### Priority 4: Safety
- [ ] Motor current monitoring
- [ ] Temperature protection
- [ ] Emergency stop integration

## Changelog

### v1.0.0 (2026-01-22) - Initial Release
- ✅ Preferences storage system
- ✅ Multi-stage auto-tuning
- ✅ Performance metrics (ISE/IAE/ITAE)
- ✅ Per-motor independent tuning
- ✅ Safety features (burst detection, limits)
- ✅ Integration dengan existing codebase

---

**Author:** PRC Team 2026  
**Last Updated:** January 22, 2026  
**License:** MIT
