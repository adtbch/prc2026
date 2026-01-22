/**
 * ============================================================
 * FILE: pid_autotuning.ino
 * LAYER: Control Algorithm Layer (Auto-Tuning)
 * ============================================================
 * 
 * DESCRIPTION:
 * Adaptive PID auto-tuning untuk motor RPM control.
 * Menggunakan metode Relay Auto-Tuning (Åström-Hägglund) dengan
 * systematic parameter exploration berdasarkan performance metrics.
 * 
 * BEST PRACTICES IMPLEMENTED:
 * 1. **Ziegler-Nichols Inspired** - Quarter decay ratio target
 * 2. **Multi-Stage Precision** - Coarse → Fine → Ultra-Fine tuning
 * 3. **Performance Metrics** - ISE, IAE, ITAE, Overshoot, Rise Time
 * 4. **Safety Features** - Burst detection, overshoot limits, timeout
 * 5. **Per-Motor Tuning** - Independent tuning untuk setiap motor
 * 
 * TUNING WORKFLOW:
 * 1. Start dengan saved PID values atau defaults
 * 2. Test motor dengan target RPM
 * 3. Analyze response (overshoot, rise time, error)
 * 4. Adjust parameters systematically
 * 5. Save best parameters ke Preferences
 * 
 * PUBLIC FUNCTIONS:
 * - autotuning_initialize()
 * - autotuning_startMotor(motorId)
 * - autotuning_update() - dipanggil di loop()
 * - autotuning_cancel()
 * - autotuning_getProgress()
 * 
 * REFERENCES:
 * - Åström & Hägglund, "PID Controllers: Theory, Design, and Tuning"
 * - Ziegler-Nichols Method (Quarter Decay Ratio)
 * - ISE/IAE/ITAE Performance Indices
 * 
 * ============================================================
 */

// Forward declarations untuk storage functions (defined in pid_storage.ino)
bool storage_getPidParameters(uint8_t motorId, float& kp, float& ki, float& kd);
bool storage_savePidParameters(uint8_t motorId, float kp, float ki, float kd);

// ══════════════════════════════════════════════════════════
// CONSTANTS & CONFIGURATION
// ══════════════════════════════════════════════════════════

namespace AutoTune {
  // Test parameters
  constexpr float TARGET_RPM = 100.0f;              // Target RPM untuk pengujian
  constexpr uint32_t TEST_DURATION_MS = 15000;      // 15 detik per test
  constexpr uint32_t COOLDOWN_DURATION_MS = 3000;   // 3 detik cooldown
  constexpr uint8_t MAX_TUNING_CYCLES = 18;         // 18 cycles total
  
  // Performance thresholds (Industry Standard)
  constexpr float HIGH_OVERSHOOT_THRESHOLD = 10.0f;  // 10% max acceptable
  constexpr float MEDIUM_OVERSHOOT_THRESHOLD = 5.0f; // 5% acceptable
  constexpr float LOW_OVERSHOOT_THRESHOLD = 3.0f;    // 3% excellent
  constexpr uint32_t SLOW_RISE_TIME_MS = 4000;       // 4s slow
  constexpr uint32_t MEDIUM_RISE_TIME_MS = 2000;     // 2s medium
  
  // PID adjustment steps (multi-stage)
  // STRATEGY: Equal Kp/Ki Priority → Kd (Balanced error minimization)
  // Based on TUNER.txt - Balanced approach untuk minimize error dengan smooth response
  
  // Kp steps (EQUAL PRIORITY with Ki - Response speed)
  constexpr float KP_COARSE_STEP = 2.5f;       // Large exploration (dari TUNER.txt)
  constexpr float KP_FINE_STEP = 0.8f;         // Medium optimization (dari TUNER.txt)
  constexpr float KP_ULTRA_FINE_STEP = 0.25f;  // Fine tuning (dari TUNER.txt)
  
  // Ki steps (EQUAL PRIORITY with Kp - Steady-state error elimination)
  constexpr float KI_COARSE_STEP = 0.3f;       // ⬇️ Reduced - Ki sangat sensitif untuk motor DC
  constexpr float KI_FINE_STEP = 0.08f;        // ⬇️ Reduced - prevent integral windup
  constexpr float KI_ULTRA_FINE_STEP = 0.02f;  // ⬇️ Reduced - micro adjustment
  
  // Kd steps (SECONDARY - Damping untuk smooth response)
  constexpr float KD_COARSE_STEP = 0.12f;      // Moderate damping (dari TUNER.txt)
  constexpr float KD_FINE_STEP = 0.05f;        // Precision damping (dari TUNER.txt)
  constexpr float KD_ULTRA_FINE_STEP = 0.015f; // Ultra-fine damping (dari TUNER.txt)
  
  // Scoring weights (Performance Index)
  // STRATEGY: Balanced approach - Error minimization dengan controlled response
  // From TUNER.txt - Priority: Error > Overshoot > Stability > Rise Time
  constexpr float OVERSHOOT_WEIGHT = 3.0f;     // CRITICAL - Prevent mechanical stress
  constexpr float RISE_TIME_WEIGHT = 0.003f;   // LOW - Fast response desired tapi not critical
  constexpr float ERROR_WEIGHT = 12.0f;        // HIGHEST - Tracking accuracy most important
  constexpr float STABILITY_WEIGHT = 5.0f;     // MEDIUM - Allow some oscillation during tuning
  constexpr float BURST_PENALTY = 0.6f;        // REDUCED - Allow initial aggressive response
  
  // Score quality thresholds
  constexpr float EXCELLENT_SCORE = 20.0f;
  constexpr float GOOD_SCORE = 35.0f;
  constexpr float ACCEPTABLE_SCORE = 50.0f;
}

// ══════════════════════════════════════════════════════════
// STATE MACHINE & DATA STRUCTURES
// ══════════════════════════════════════════════════════════

enum AutoTuningState {
  AUTOTUNE_IDLE,
  AUTOTUNE_STARTING,
  AUTOTUNE_RAMP_UP,        // Testing phase (sesuai TUNER.txt)
  AUTOTUNE_ANALYZING,
  AUTOTUNE_COOLDOWN,
  AUTOTUNE_UPDATING,
  AUTOTUNE_SWITCH_MOTOR,   // Switch motor phase untuk tune all (sesuai TUNER.txt)
  AUTOTUNE_FINISHED
};

enum TuningPrecision {
  PRECISION_COARSE,
  PRECISION_FINE,
  PRECISION_ULTRA_FINE
};

struct AutoTuneMetrics {
  float maxOvershoot;               // Maximum overshoot (%)
  uint32_t riseTime;                // Rise time 10%-90% (ms)
  float totalError;                 // Accumulated error (IAE)
  uint32_t sampleCount;             // Number of samples
  float initialBurst;               // Initial burst magnitude
  bool burstDetected;               // Burst detection flag
  bool hasCrossed10pct;             // Rise time tracking
  bool hasCrossed90pct;             // Rise time tracking
  
  void reset() {
    maxOvershoot = 0.0f;
    riseTime = 0;
    totalError = 0.0f;
    sampleCount = 0;
    initialBurst = 0.0f;
    burstDetected = false;
    hasCrossed10pct = false;
    hasCrossed90pct = false;
  }
  
  float getAverageError() const {
    return (sampleCount > 0) ? (totalError / sampleCount) : 0.0f;
  }
};

/**
 * @brief History entry untuk setiap tuning cycle
 * 
 * Menyimpan hasil test PID values dengan performance score.
 * Digunakan untuk analisa dan transparansi tuning process.
 */
struct TuningHistoryEntry {
  uint8_t cycleNumber;          // Cycle number (1-18)
  float testKp;                 // PID values yang di-test
  float testKi;
  float testKd;
  float score;                  // Performance score (lower = better)
  float maxOvershoot;           // Metrics untuk analisa
  uint32_t riseTime;
  float avgError;
  uint8_t precision;            // Precision stage (0=COARSE, 1=FINE, 2=ULTRA_FINE)
  
  // Calculate accuracy percentage dari score
  float getAccuracyPercent() const {
    // Score mapping ke percentage (lower score = higher accuracy)
    // Score 0-20 = 100-90%
    // Score 20-40 = 90-80%
    // Score 40-60 = 80-70%
    // Score >60 = <70%
    if (score < 20.0f) {
      return 100.0f - (score / 20.0f * 10.0f);  // 100-90%
    } else if (score < 40.0f) {
      return 90.0f - ((score - 20.0f) / 20.0f * 10.0f);  // 90-80%
    } else if (score < 60.0f) {
      return 80.0f - ((score - 40.0f) / 20.0f * 10.0f);  // 80-70%
    } else {
      return max(0.0f, 70.0f - ((score - 60.0f) / 40.0f * 20.0f));  // <70%
    }
  }
};

// ══════════════════════════════════════════════════════════
// GLOBAL VARIABLES
// ══════════════════════════════════════════════════════════

static AutoTuningState _autotuneState = AUTOTUNE_IDLE;
static TuningPrecision _tunePrecision = PRECISION_COARSE;

static uint8_t _tuningMotorId = 0;           // Motor yang sedang di-tune (0-2)
static bool _tuneAllMotors = false;          // Flag untuk tune all motors mode
static uint8_t _tuneAllCurrentMotor = 0;     // Current motor dalam tune all mode (0-2)
static uint8_t _tuningCycle = 0;             // Current cycle number
static uint32_t _testStartTime = 0;          // Test start timestamp

static AutoTuneMetrics _metrics;

// Current PID values being tested
static float _testKp = 0.0f;
static float _testKi = 0.0f;
static float _testKd = 0.0f;

// Best PID values found (current motor)
static float _bestKp = 0.0f;
static float _bestKi = 0.0f;
static float _bestKd = 0.0f;
static float _bestScore = 999999.0f;

// Best PID values per motor (tune all mode)
static float _bestKpPerMotor[3] = {0.0f, 0.0f, 0.0f};
static float _bestKiPerMotor[3] = {0.0f, 0.0f, 0.0f};
static float _bestKdPerMotor[3] = {0.0f, 0.0f, 0.0f};
static float _bestScorePerMotor[3] = {999999.0f, 999999.0f, 999999.0f};

// Tuning control
static float _currentKpStep = AutoTune::KP_COARSE_STEP;
static float _currentKiStep = AutoTune::KI_COARSE_STEP;
static float _currentKdStep = AutoTune::KD_COARSE_STEP;
static uint8_t _cyclesWithoutImprovement = 0;
static uint8_t _precisionStageCount = 0;

// Tuning History Table (NEW FEATURE!)
static TuningHistoryEntry _historyTable[AutoTune::MAX_TUNING_CYCLES];
static uint8_t _historyCount = 0;  // Number of entries in history

// ══════════════════════════════════════════════════════════
// HELPER FUNCTIONS
// ══════════════════════════════════════════════════════════

/**
 * @brief Constrain PID values ke safe operating range
 * 
 * CRITICAL: Motor DC sangat sensitif terhadap Ki dan Kd
 * Range di-set lebih ketat untuk prevent wild oscillation
 */
void _constrainPidValues(float& kp, float& ki, float& kd) {
  // Kp range: 0.5 - 50.0 (response speed, tidak boleh terlalu kecil atau besar)
  kp = math_clampValue(kp, 0.5f, 50.0f);
  
  // Ki range: 0.01 - 5.0 (CRITICAL: Ki terlalu besar menyebabkan integral windup dan oscillation)
  ki = math_clampValue(ki, 0.01f, 5.0f);
  
  // Kd range: 0.0 - 2.0 (derivative sangat sensitif noise, jangan terlalu besar)
  kd = math_clampValue(kd, 0.0f, 2.0f);
  
  // Log jika ada clamping (warning untuk debug)
  static float prevKp = 0, prevKi = 0, prevKd = 0;
  if (kp != prevKp || ki != prevKi || kd != prevKd) {
    Serial.printf("[AutoTune] Constrained PID: Kp=%.3f, Ki=%.3f, Kd=%.3f\n", kp, ki, kd);
    prevKp = kp; prevKi = ki; prevKd = kd;
  }
}

/**
 * @brief Calculate performance score berdasarkan metrics
 * Lower score = better performance
 * 
 * Based on ISE (Integral Square Error) and ITAE (Integral Time Absolute Error)
 */
float _calculatePerformanceScore() {
  float avgError = _metrics.getAverageError();
  float overshoot = _metrics.maxOvershoot;
  uint32_t riseTime = _metrics.riseTime;
  
  float score = 0.0f;
  
  // 1. ERROR TERM (ISE approach) - Most important
  score += avgError * avgError * AutoTune::ERROR_WEIGHT;
  
  // 2. OVERSHOOT PENALTY (Quadratic for safety)
  if (overshoot > AutoTune::HIGH_OVERSHOOT_THRESHOLD) {
    score += (overshoot * overshoot) * AutoTune::OVERSHOOT_WEIGHT * 2.0f;
  } else if (overshoot > AutoTune::MEDIUM_OVERSHOOT_THRESHOLD) {
    score += (overshoot * overshoot) * AutoTune::OVERSHOOT_WEIGHT;
  } else {
    score += overshoot * AutoTune::OVERSHOOT_WEIGHT * 0.5f;
  }
  
  // 3. RISE TIME PENALTY (Linear)
  if (riseTime > AutoTune::SLOW_RISE_TIME_MS) {
    score += (float)(riseTime - AutoTune::SLOW_RISE_TIME_MS) * AutoTune::RISE_TIME_WEIGHT * 1.5f;
  } else if (riseTime > AutoTune::MEDIUM_RISE_TIME_MS) {
    score += (float)(riseTime - AutoTune::MEDIUM_RISE_TIME_MS) * AutoTune::RISE_TIME_WEIGHT;
  }
  
  // 4. STABILITY BONUS (Reward consistent performance)
  if (_metrics.sampleCount > 50) {
    if (avgError < 1.5f) {
      score -= 8.0f * AutoTune::STABILITY_WEIGHT;
    } else if (avgError < 3.0f) {
      score -= 4.0f * AutoTune::STABILITY_WEIGHT;
    }
  }
  
  // 5. QUALITY BONUS (Quarter Decay achievement)
  if (riseTime < AutoTune::MEDIUM_RISE_TIME_MS && 
      overshoot < AutoTune::MEDIUM_OVERSHOOT_THRESHOLD && 
      avgError < 2.5f) {
    score -= 15.0f;  // Excellent zone bonus
  }
  
  // 6. BURST PENALTY
  if (_metrics.burstDetected) {
    float burstOvershoot = ((_metrics.initialBurst - AutoTune::TARGET_RPM) / AutoTune::TARGET_RPM) * 100.0f;
    if (burstOvershoot > 30.0f) {
      score += burstOvershoot * AutoTune::BURST_PENALTY;
    }
  }
  
  // 7. PID SANITY CHECK (Prevent extreme values)
  if (_testKp < 1.0f || _testKp > 80.0f) score += 20.0f;
  if (_testKi < 0.1f || _testKi > 40.0f) score += 15.0f;
  if (_testKd < 0.0f || _testKd > 8.0f) score += 12.0f;
  
  return max(0.0f, score);
}

/**
 * @brief Check apakah perlu pindah ke precision stage berikutnya
 */
bool _shouldTransitionPrecision() {
  _precisionStageCount++;
  
  if (_tunePrecision == PRECISION_COARSE) {
    return (_precisionStageCount >= 6 || _cyclesWithoutImprovement >= 3 || 
            _bestScore < AutoTune::EXCELLENT_SCORE);
  } else if (_tunePrecision == PRECISION_FINE) {
    return (_precisionStageCount >= 5 || _cyclesWithoutImprovement >= 3);
  }
  
  return false;
}

/**
 * @brief Update precision stage
 */
void _updatePrecisionStage() {
  if (_tunePrecision == PRECISION_COARSE) {
    _tunePrecision = PRECISION_FINE;
    _currentKpStep = AutoTune::KP_FINE_STEP;
    _currentKiStep = AutoTune::KI_FINE_STEP;
    _currentKdStep = AutoTune::KD_FINE_STEP;
    Serial.println("[AutoTune] Switching to FINE precision");
  } else if (_tunePrecision == PRECISION_FINE) {
    _tunePrecision = PRECISION_ULTRA_FINE;
    _currentKpStep = AutoTune::KP_ULTRA_FINE_STEP;
    _currentKiStep = AutoTune::KI_ULTRA_FINE_STEP;
    _currentKdStep = AutoTune::KD_ULTRA_FINE_STEP;
    Serial.println("[AutoTune] Switching to ULTRA-FINE precision");
  }
  
  _precisionStageCount = 0;
  _cyclesWithoutImprovement = 0;
}

/**
 * @brief Systematic parameter adjustment based on metrics
 * Implements BALANCED strategy (Equal Kp/Ki priority → Kd)
 * Based on TUNER.txt - Focus: Error minimization dengan balanced response
 */
void _adjustParameters() {
  float overshoot = _metrics.maxOvershoot;
  float avgError = _metrics.getAverageError();
  uint32_t riseTime = _metrics.riseTime;
  
  Serial.printf("\n[AutoTune] Analyzing: Overshoot=%.1f%%, Rise=%lums, Error=%.2f\n",
                overshoot, riseTime, avgError);
  
  // ═══ BALANCED SEQUENTIAL TUNING - TUNER.txt Strategy ═══
  // Priority: Establish P → Eliminate SS Error with I → Add Damping with D
  
  if (_tunePrecision == PRECISION_COARSE) {
    // COARSE: Balanced exploration untuk find optimal Kp/Ki range
    
    if (overshoot > AutoTune::HIGH_OVERSHOOT_THRESHOLD) {
      // Severe penalty - reduce both Kp dan add Kd
      _testKp *= 0.75f;  // ZN: Reduce by 25% if overshoot (dari TUNER.txt)
      _testKd *= 1.5f;   // Moderate damping increase (dari TUNER.txt)
      Serial.println("[AutoTune] COARSE: HIGH OVERSHOOT → Reduce Kp, Increase Kd");
      
    } else if (riseTime > AutoTune::SLOW_RISE_TIME_MS) {
      // Slow rise - increase Kp AND Ki together (balanced approach)
      _testKp *= 1.25f;  // ⬇️ Moderate increase (prevent overshoot)
      _testKi *= 1.10f;  // ⬇️ Conservative Ki increase (prevent windup)
      Serial.println("[AutoTune] COARSE: SLOW RISE → Increase Kp+Ki (balanced)");
      
    } else if (avgError > 5.0f) {
      // High steady-state error - Ki priority
      _testKi *= 1.15f;  // ⬇️ Conservative increase (Ki sangat sensitif)
      Serial.println("[AutoTune] COARSE: HIGH ERROR → Increase Ki");
      
    } else if (overshoot > AutoTune::MEDIUM_OVERSHOOT_THRESHOLD && 
               riseTime < AutoTune::MEDIUM_RISE_TIME_MS) {
      // Medium overshoot with good rise - fine-tune Kd
      _testKd *= 1.15f;  // ⬇️ Conservative Kd (sensitif noise)
      _testKp *= 0.95f;  // Slight Kp reduction
      Serial.println("[AutoTune] COARSE: MEDIUM OVERSHOOT → Increase Kd, reduce Kp");
      
    } else if (_metrics.burstDetected) {
      // Oscillation detected - moderate damping (tidak terlalu agresif)
      _testKp *= 0.80f;  // ⬆️ Less aggressive reduction
      _testKd *= 1.30f;  // ⬇️ Moderate damping increase
      _testKi *= 0.85f;  // ⬆️ Less aggressive reduction
      Serial.println("[AutoTune] COARSE: BURST → Moderate damping");
      
    } else {
      // Directional search based on improvement
      if (_cyclesWithoutImprovement == 0) {
        // Keep direction, increase step (balanced Kp/Ki)
        _testKp += _currentKpStep * 1.2f;
        Serial.println("[AutoTune] COARSE: IMPROVING → Continue Kp direction");
      } else {
        // Change direction, try Ki
        _testKi += _currentKiStep * 1.0f;
        Serial.println("[AutoTune] COARSE: EXPLORING → Try Ki increase");
      }
    }
    
  } else if (_tunePrecision == PRECISION_FINE) {
    // FINE: Precision optimization dengan smaller adjustments
    
    if (overshoot > AutoTune::MEDIUM_OVERSHOOT_THRESHOLD) {
      _testKp -= _currentKpStep;
      _testKd += _currentKdStep * 1.5f;
      Serial.println("[AutoTune] FINE: Reduce overshoot → Kp-, Kd+");
      
    } else if (overshoot < AutoTune::LOW_OVERSHOOT_THRESHOLD && 
               avgError < 3.0f && riseTime < AutoTune::MEDIUM_RISE_TIME_MS) {
      // Already in good zone - micro-optimization
      _testKi += _currentKiStep * 0.8f;
      Serial.println("[AutoTune] FINE: Optimize SS error → Ki+");
      
    } else if (avgError > 3.0f) {
      _testKi += _currentKiStep * 1.2f;
      Serial.println("[AutoTune] FINE: Reduce SS error → Ki+");
      
    } else if (riseTime > AutoTune::MEDIUM_RISE_TIME_MS) {
      _testKp += _currentKpStep * 0.8f;
      Serial.println("[AutoTune] FINE: Improve response → Kp+");
      
    } else {
      // Balance optimization
      _testKd += _currentKdStep * 0.5f;
      Serial.println("[AutoTune] FINE: Balance optimization → Kd+");
    }
    
  } else {
    // ULTRA-FINE: Micro-optimization untuk final polish
    
    if (overshoot > 5.0f) {
      _testKp -= _currentKpStep * 0.6f;
      Serial.println("[AutoTune] ULTRA-FINE: Reduce overshoot → Kp-");
      
    } else if (avgError > 2.0f) {
      _testKi += _currentKiStep * 0.8f;
      Serial.println("[AutoTune] ULTRA-FINE: Reduce error → Ki+");
      
    } else if (riseTime > AutoTune::MEDIUM_RISE_TIME_MS + 500) {
      _testKp += _currentKpStep * 0.5f;
      Serial.println("[AutoTune] ULTRA-FINE: Improve response → Kp+");
      
    } else {
      // Local search dengan pattern sistematis (dari TUNER.txt)
      int pattern = _tuningCycle % 3;
      if (pattern == 0) {
        _testKp += _currentKpStep * 0.3f;
        Serial.println("[AutoTune] ULTRA-FINE: Pattern Kp+");
      } else if (pattern == 1) {
        _testKp -= _currentKpStep * 0.3f;
        Serial.println("[AutoTune] ULTRA-FINE: Pattern Kp-");
      } else {
        _testKi += _currentKiStep * 0.4f;
        Serial.println("[AutoTune] ULTRA-FINE: Pattern Ki+");
      }
    }
  }
  
  // Constrain values
  _constrainPidValues(_testKp, _testKi, _testKd);
  
  Serial.printf("[AutoTune] Next test: Kp=%.4f, Ki=%.4f, Kd=%.4f\n", 
                _testKp, _testKi, _testKd);
}

// ══════════════════════════════════════════════════════════
// PUBLIC FUNCTIONS
// ══════════════════════════════════════════════════════════

/**
 * @brief Initialize auto-tuning system
 * 
 * Harus dipanggil di setup()
 */
void autotuning_initialize() {
  Serial.println("[AutoTune] System initialized");
  _autotuneState = AUTOTUNE_IDLE;
}

/**
 * @brief Start auto-tuning untuk motor tertentu
 * 
 * @param motorId Motor ID (0-2)
 * @return true if tuning started successfully
 */
bool autotuning_startMotor(uint8_t motorId) {
  if (_autotuneState != AUTOTUNE_IDLE) {
    Serial.println("[AutoTune] ERROR: Tuning already running");
    return false;
  }
  
  if (motorId > 2) {
    Serial.println("[AutoTune] ERROR: Invalid motor ID");
    return false;
  }
  
  _tuneAllMotors = false;  // Single motor mode
  _tuningMotorId = motorId;
  _tuningCycle = 0;
  _tunePrecision = PRECISION_COARSE;
  _cyclesWithoutImprovement = 0;
  _precisionStageCount = 0;
  
  // Reset history table
  _historyCount = 0;
  
  // Load saved PID values sebagai starting point
  float savedKp, savedKi, savedKd;
  if (storage_getPidParameters(motorId, savedKp, savedKi, savedKd)) {
    _testKp = savedKp;
    _testKi = savedKi;
    _testKd = savedKd;
    Serial.printf("[AutoTune] Starting from saved: Kp=%.3f, Ki=%.3f, Kd=%.3f\n",
                  savedKp, savedKi, savedKd);
  } else {
    // Fallback ke defaults
    _testKp = 2.0f;
    _testKi = 0.08f;
    _testKd = 0.0f;
    Serial.println("[AutoTune] Starting from defaults");
  }
  
  // Initialize best values
  _bestKp = _testKp;
  _bestKi = _testKi;
  _bestKd = _testKd;
  _bestScore = 999999.0f;
  
  // Reset precision stage
  _currentKpStep = AutoTune::KP_COARSE_STEP;
  _currentKiStep = AutoTune::KI_COARSE_STEP;
  _currentKdStep = AutoTune::KD_COARSE_STEP;
  
  Serial.println("\n========== AUTO-TUNING START ==========");
  Serial.printf("Motor: %d\n", motorId + 1);
  Serial.printf("Target RPM: %.0f\n", AutoTune::TARGET_RPM);
  Serial.printf("Max Cycles: %d\n", AutoTune::MAX_TUNING_CYCLES);
  Serial.printf("Est. Duration: %.1f minutes\n",
                (AutoTune::MAX_TUNING_CYCLES * (AutoTune::TEST_DURATION_MS + AutoTune::COOLDOWN_DURATION_MS)) / 60000.0f);
  Serial.println("=======================================\n");
  
  _autotuneState = AUTOTUNE_STARTING;
  return true;
}

/**
 * @brief Start auto-tuning untuk SEMUA motor (sequential)
 * 
 * State machine: IDLE → STARTING → RAMP_UP → ANALYZING → COOLDOWN → 
 *                UPDATING → SWITCH_MOTOR → (repeat for next motor) → FINISHED
 * 
 * @return true if tuning started successfully
 */
bool autotuning_startAllMotors() {
  if (_autotuneState != AUTOTUNE_IDLE) {
    Serial.println("[AutoTune] ERROR: Tuning already running");
    return false;
  }
  
  _tuneAllMotors = true;          // Tune all mode
  _tuneAllCurrentMotor = 0;       // Start dengan motor 1
  _tuningMotorId = 0;
  _tuningCycle = 0;
  _tunePrecision = PRECISION_COARSE;
  _cyclesWithoutImprovement = 0;
  _precisionStageCount = 0;
  
  // Reset history table
  _historyCount = 0;
  
  // Initialize best scores untuk semua motor
  for (uint8_t i = 0; i < 3; i++) {
    _bestScorePerMotor[i] = 999999.0f;
    
    // Load saved PID sebagai starting point
    float savedKp, savedKi, savedKd;
    if (storage_getPidParameters(i, savedKp, savedKi, savedKd)) {
      _bestKpPerMotor[i] = savedKp;
      _bestKiPerMotor[i] = savedKi;
      _bestKdPerMotor[i] = savedKd;
    } else {
      _bestKpPerMotor[i] = 2.0f;
      _bestKiPerMotor[i] = 0.08f;
      _bestKdPerMotor[i] = 0.0f;
    }
  }
  
  // Start dengan motor pertama
  _testKp = _bestKpPerMotor[0];
  _testKi = _bestKiPerMotor[0];
  _testKd = _bestKdPerMotor[0];
  _bestKp = _testKp;
  _bestKi = _testKi;
  _bestKd = _testKd;
  _bestScore = 999999.0f;
  
  // Reset precision stage
  _currentKpStep = AutoTune::KP_COARSE_STEP;
  _currentKiStep = AutoTune::KI_COARSE_STEP;
  _currentKdStep = AutoTune::KD_COARSE_STEP;
  
  Serial.println("\n========== AUTO-TUNING ALL MOTORS ==========");
  Serial.printf("Target RPM: %.0f\n", AutoTune::TARGET_RPM);
  Serial.printf("Cycles per motor: %d\n", AutoTune::MAX_TUNING_CYCLES / 3);
  Serial.printf("Total cycles: %d\n", AutoTune::MAX_TUNING_CYCLES);
  Serial.printf("Est. Duration: %.1f minutes\n",
                (AutoTune::MAX_TUNING_CYCLES * (AutoTune::TEST_DURATION_MS + AutoTune::COOLDOWN_DURATION_MS)) / 60000.0f);
  Serial.println("===========================================\n");
  
  _autotuneState = AUTOTUNE_STARTING;
  return true;
}

/**
 * @brief Update auto-tuning state machine
 * 
 * Harus dipanggil di loop() saat tuning active
 */
void autotuning_update() {
  if (_autotuneState == AUTOTUNE_IDLE) return;
  
  uint32_t currentTime = millis();
  
  switch (_autotuneState) {
    case AUTOTUNE_STARTING:
    {
      // Display cycle info di LCD
      lcd_debugTuningProgress(_tuningMotorId, _tuningCycle + 1, AutoTune::MAX_TUNING_CYCLES, 
                               (_tuningCycle * 100) / AutoTune::MAX_TUNING_CYCLES);
      
      // Show PID values being tested
      lcd_debugPidValues(_tuningMotorId, _testKp, _testKi, _testKd);
      
      // Reset metrics
      _metrics.reset();
      
      // CRITICAL: Apply test PID values ke runtime
      // Temporary override untuk test cycle ini saja
      Serial.printf("[AutoTune] Applying test PID: Kp=%.4f, Ki=%.4f, Kd=%.4f\n", 
                   _testKp, _testKi, _testKd);
      
      // Apply ke motor yang sedang di-tune
      if (_tuningMotorId == 0) {
        Tuning::RPM_WHEEL1.kp = _testKp;
        Tuning::RPM_WHEEL1.ki = _testKi;
        Tuning::RPM_WHEEL1.kd = _testKd;
      } else if (_tuningMotorId == 1) {
        Tuning::RPM_WHEEL2.kp = _testKp;
        Tuning::RPM_WHEEL2.ki = _testKi;
        Tuning::RPM_WHEEL2.kd = _testKd;
      } else {
        Tuning::RPM_WHEEL3.kp = _testKp;
        Tuning::RPM_WHEEL3.ki = _testKi;
        Tuning::RPM_WHEEL3.kd = _testKd;
      }
      
      // CRITICAL: Reset PID state untuk prevent integral windup
      // Error history dari test sebelumnya harus di-clear
      uint8_t pidChannel = (_tuningMotorId == 0) ? PidChannel::WHEEL1_RPM :
                           (_tuningMotorId == 1) ? PidChannel::WHEEL2_RPM :
                                                   PidChannel::WHEEL3_RPM;
      control_resetPidChannel(pidChannel);
      
      Serial.println("[AutoTune] ✓ Test PID applied, integral reset");
      
      // Start motor dengan target RPM (HANYA motor yang di-tune)
      float targetRpm[3] = {0.0f, 0.0f, 0.0f};
      targetRpm[_tuningMotorId] = AutoTune::TARGET_RPM;
      control_setRpmAllWheels(targetRpm[0], targetRpm[1], targetRpm[2]);
      
      _testStartTime = currentTime;
      _autotuneState = AUTOTUNE_RAMP_UP;  // Transition ke RAMP_UP (sesuai TUNER.txt)
    }
      break;
      
    case AUTOTUNE_RAMP_UP:  // Testing phase (sesuai TUNER.txt naming)
    {
      uint32_t elapsed = currentTime - _testStartTime;
      
      if (elapsed < AutoTune::TEST_DURATION_MS) {
        // CRITICAL: Ignore first 300ms untuk stabilisasi
        // Saat PID baru di-apply, response awal bisa wild
        if (elapsed < 300) {
          // Skip measurement, let motor stabilize
          static uint32_t lastStabilizeMsg = 0;
          if (currentTime - lastStabilizeMsg >= 100) {
            Serial.println("[AutoTune] Stabilizing...");
            lastStabilizeMsg = currentTime;
          }
          yield();  // Maintain PS3 connection
          break;
        }
        
        // Collect metrics selama test (after stabilization)
        float currentRpm = hardware_getEncoderRpm(_tuningMotorId);
        
        // Detect initial burst (dalam 500ms setelah stabilization)
        if (!_metrics.burstDetected && elapsed < 800) {
          if (currentRpm > AutoTune::TARGET_RPM * 1.3f) {
            _metrics.initialBurst = currentRpm;
            _metrics.burstDetected = true;
            Serial.printf("[AutoTune] BURST: %.1f RPM at %lums\n", currentRpm, elapsed);
          }
        }
        
        // Calculate error
        float error = fabsf(currentRpm - AutoTune::TARGET_RPM);
        _metrics.totalError += error;
        _metrics.sampleCount++;
        
        // Track overshoot
        if (currentRpm > AutoTune::TARGET_RPM) {
          float overshoot = ((currentRpm - AutoTune::TARGET_RPM) / AutoTune::TARGET_RPM) * 100.0f;
          if (overshoot > _metrics.maxOvershoot) {
            _metrics.maxOvershoot = overshoot;
          }
        }
        
        // Track rise time (10% → 90%)
        if (!_metrics.hasCrossed10pct && currentRpm >= AutoTune::TARGET_RPM * 0.1f) {
          _metrics.riseTime = currentTime;
          _metrics.hasCrossed10pct = true;
        }
        if (_metrics.hasCrossed10pct && !_metrics.hasCrossed90pct && 
            currentRpm >= AutoTune::TARGET_RPM * 0.9f) {
          _metrics.riseTime = currentTime - _metrics.riseTime;
          _metrics.hasCrossed90pct = true;
        }
        
        // Real-time monitoring (setiap 100ms)
        static uint32_t lastPrint = 0;
        if (currentTime - lastPrint >= 100) {
          Serial.printf("t=%lums, RPM=%.1f, Error=%.1f\n", elapsed, currentRpm, error);
          lastPrint = currentTime;
          
          // CRITICAL: Yield untuk prevent watchdog timeout dan maintain PS3 connection
          yield();
        }
        
      } else {
        // Test selesai, lanjut ke analisis
        _autotuneState = AUTOTUNE_ANALYZING;
      }
    }
      break;
      
    case AUTOTUNE_ANALYZING:
    {
      // Display metrics di LCD
      float score = _calculatePerformanceScore();
      lcd_debugMetrics(_metrics.maxOvershoot, _metrics.riseTime, score);
      
      // ═══ SAVE TO HISTORY TABLE (NEW FEATURE!) ═══
      // Record hasil cycle ini untuk analisa dan transparansi
      if (_historyCount < AutoTune::MAX_TUNING_CYCLES) {
        _historyTable[_historyCount].cycleNumber = _tuningCycle + 1;
        _historyTable[_historyCount].testKp = _testKp;
        _historyTable[_historyCount].testKi = _testKi;
        _historyTable[_historyCount].testKd = _testKd;
        _historyTable[_historyCount].score = score;
        _historyTable[_historyCount].maxOvershoot = _metrics.maxOvershoot;
        _historyTable[_historyCount].riseTime = _metrics.riseTime;
        _historyTable[_historyCount].avgError = _metrics.getAverageError();
        _historyTable[_historyCount].precision = (uint8_t)_tunePrecision;
        _historyCount++;
        
        // Print cycle result dengan accuracy percentage
        float accuracy = _historyTable[_historyCount - 1].getAccuracyPercent();
        Serial.printf("[Cycle %d/%d] Score: %.2f (Accuracy: %.1f%%) - Kp=%.3f, Ki=%.3f, Kd=%.3f\n",
                     _tuningCycle + 1, AutoTune::MAX_TUNING_CYCLES, score, accuracy,
                     _testKp, _testKi, _testKd);
      }
      
      // Update best if improved
      if (score < _bestScore) {
        _bestScore = score;
        _bestKp = _testKp;
        _bestKi = _testKi;
        _bestKd = _testKd;
        _cyclesWithoutImprovement = 0;
        Serial.println(">>> NEW BEST PARAMETERS! <<<");
        
        // Save immediately ke Preferences saat best parameters updated
        // Ini memastikan jika ada crash/interrupt, parameter terbaik sudah tersimpan
        Serial.println("[AutoTune] Saving new best parameters to flash...");
        if (storage_savePidParameters(_tuningMotorId, _bestKp, _bestKi, _bestKd)) {
          Serial.println("[AutoTune] ✓ Best parameters saved successfully");
        } else {
          Serial.println("[AutoTune] ✗ WARNING: Failed to save parameters");
        }
      } else {
        _cyclesWithoutImprovement++;
      }
      
      // Stop motor untuk cooldown
      control_setRpmAllWheels(0.0f, 0.0f, 0.0f);
      
      _testStartTime = currentTime;
      _autotuneState = AUTOTUNE_COOLDOWN;
    }
      break;
      
    case AUTOTUNE_COOLDOWN:
    {
      // Cooldown period untuk stabilisasi motor sebelum cycle berikutnya
      // Parameter terbaik sudah di-save saat ANALYZING (real-time save)
      
      if (currentTime - _testStartTime >= AutoTune::COOLDOWN_DURATION_MS) {
        _tuningCycle++;
        
        if (_tuneAllMotors) {
          // TUNE ALL MOTORS mode
          const uint8_t CYCLES_PER_MOTOR = AutoTune::MAX_TUNING_CYCLES / 3;  // 6 cycles per motor
          
          // Check apakah current motor sudah selesai cycles-nya
          if (_tuningCycle >= (_tuneAllCurrentMotor + 1) * CYCLES_PER_MOTOR) {
            // Motor ini selesai, simpan best parameters
            _bestKpPerMotor[_tuneAllCurrentMotor] = _bestKp;
            _bestKiPerMotor[_tuneAllCurrentMotor] = _bestKi;
            _bestKdPerMotor[_tuneAllCurrentMotor] = _bestKd;
            _bestScorePerMotor[_tuneAllCurrentMotor] = _bestScore;
            
            Serial.printf("\n[AutoTune] Motor %d COMPLETED: Kp=%.4f, Ki=%.4f, Kd=%.4f (Score: %.2f)\n",
                         _tuneAllCurrentMotor + 1, _bestKp, _bestKi, _bestKd, _bestScore);
            
            // Check apakah masih ada motor berikutnya
            if (_tuneAllCurrentMotor < 2) {
              // Ada motor berikutnya → SWITCH_MOTOR
              _autotuneState = AUTOTUNE_SWITCH_MOTOR;
            } else {
              // Semua motor selesai → FINISHED
              _autotuneState = AUTOTUNE_FINISHED;
            }
          } else {
            // Lanjut tuning motor yang sama
            _autotuneState = AUTOTUNE_UPDATING;
          }
        } else {
          // SINGLE MOTOR mode
          // Check apakah tuning selesai
          if (_tuningCycle >= AutoTune::MAX_TUNING_CYCLES) {
            _autotuneState = AUTOTUNE_FINISHED;
          } else {
            _autotuneState = AUTOTUNE_UPDATING;
          }
        }
      }
    }
      break;
      
    case AUTOTUNE_SWITCH_MOTOR:
    {
      // State untuk switch ke motor berikutnya (hanya untuk tune all mode)
      // Sesuai TUNER.txt state machine
      
      Serial.println("\n=== SWITCHING TO NEXT MOTOR ===");
      Serial.printf("Completed Motor %d: Kp=%.4f, Ki=%.4f, Kd=%.4f (Score: %.2f)\n",
                   _tuneAllCurrentMotor + 1, _bestKp, _bestKi, _bestKd, _bestScore);
      
      // Switch ke motor berikutnya
      _tuneAllCurrentMotor++;
      _tuningMotorId = _tuneAllCurrentMotor;
      
      // Load starting parameters untuk motor berikutnya
      _testKp = _bestKpPerMotor[_tuneAllCurrentMotor];
      _testKi = _bestKiPerMotor[_tuneAllCurrentMotor];
      _testKd = _bestKdPerMotor[_tuneAllCurrentMotor];
      
      // Reset best values untuk motor baru
      _bestKp = _testKp;
      _bestKi = _testKi;
      _bestKd = _testKd;
      _bestScore = 999999.0f;
      
      // Reset precision untuk motor baru
      _tunePrecision = PRECISION_COARSE;
      _cyclesWithoutImprovement = 0;
      _precisionStageCount = 0;
      _currentKpStep = AutoTune::KP_COARSE_STEP;
      _currentKiStep = AutoTune::KI_COARSE_STEP;
      _currentKdStep = AutoTune::KD_COARSE_STEP;
      
      Serial.printf("Starting Motor %d with: Kp=%.4f, Ki=%.4f, Kd=%.4f\n",
                   _tuneAllCurrentMotor + 1, _testKp, _testKi, _testKd);
      Serial.println("==============================\n");
      
      _autotuneState = AUTOTUNE_STARTING;
    }
      break;
      
    case AUTOTUNE_UPDATING:
    {
      // Check precision transition
      if (_shouldTransitionPrecision()) {
        _updatePrecisionStage();
      }
      
      // Adjust parameters untuk next cycle
      _adjustParameters();
      
      _autotuneState = AUTOTUNE_STARTING;
    }
      break;
      
    case AUTOTUNE_FINISHED:
    {
      Serial.println("\n========== TUNING COMPLETE ==========");
      
      if (_tuneAllMotors) {
        // Display hasil semua motor
        Serial.println("=== ALL MOTORS TUNING RESULTS ===");
        for (uint8_t i = 0; i < 3; i++) {
          Serial.printf("\nMotor %d:\n", i + 1);
          Serial.printf("  Kp = %.4f\n", _bestKpPerMotor[i]);
          Serial.printf("  Ki = %.4f\n", _bestKiPerMotor[i]);
          Serial.printf("  Kd = %.4f\n", _bestKdPerMotor[i]);
          Serial.printf("  Score = %.2f ", _bestScorePerMotor[i]);
          
          // Quality rating per motor
          if (_bestScorePerMotor[i] < AutoTune::EXCELLENT_SCORE) {
            Serial.println("(EXCELLENT ✓✓✓)");
          } else if (_bestScorePerMotor[i] < AutoTune::GOOD_SCORE) {
            Serial.println("(GOOD ✓✓)");
          } else if (_bestScorePerMotor[i] < AutoTune::ACCEPTABLE_SCORE) {
            Serial.println("(ACCEPTABLE ✓)");
          } else {
            Serial.println("(NEEDS IMPROVEMENT)");
          }
        }
        
        // Save all motor parameters
        Serial.println("\nSaving all motor parameters...");
        for (uint8_t i = 0; i < 3; i++) {
          if (storage_savePidParameters(i, _bestKpPerMotor[i], _bestKiPerMotor[i], _bestKdPerMotor[i])) {
            Serial.printf("✓ Motor %d saved\n", i + 1);
          } else {
            Serial.printf("✗ Motor %d FAILED\n", i + 1);
          }
        }
        
        // LCD feedback untuk tune all
        lcd_debugMessage("All Motors DONE!", "Check Serial");
        delay(3000);
        
      } else {
        // Single motor result
        Serial.printf("Motor %d Best Parameters:\n", _tuningMotorId + 1);
        Serial.printf("  Kp = %.4f\n", _bestKp);
        Serial.printf("  Ki = %.4f\n", _bestKi);
        Serial.printf("  Kd = %.4f\n", _bestKd);
        Serial.printf("  Score = %.2f\n", _bestScore);
        
        // Evaluate quality
        if (_bestScore < AutoTune::EXCELLENT_SCORE) {
          Serial.println("Quality: EXCELLENT ✓✓✓");
        } else if (_bestScore < AutoTune::GOOD_SCORE) {
          Serial.println("Quality: GOOD ✓✓");
        } else if (_bestScore < AutoTune::ACCEPTABLE_SCORE) {
          Serial.println("Quality: ACCEPTABLE ✓");
        } else {
          Serial.println("Quality: NEEDS IMPROVEMENT");
        }
        
        // ═══ PRINT HISTORY TABLE (NEW FEATURE!) ═══
        // Show semua cycle results untuk transparansi dan analisa
        Serial.println("\n========== TUNING HISTORY TABLE ==========");
        Serial.println("Cycle | Kp    | Ki    | Kd    | Score | Accuracy | Quality");
        Serial.println("------|-------|-------|-------|-------|----------|--------");
        
        uint8_t bestCycleIndex = 0;
        for (uint8_t i = 0; i < _historyCount; i++) {
          const TuningHistoryEntry& entry = _historyTable[i];
          float accuracy = entry.getAccuracyPercent();
          
          // Mark best cycle dengan arrow
          bool isBest = (fabsf(entry.score - _bestScore) < 0.01f &&
                         fabsf(entry.testKp - _bestKp) < 0.001f);
          if (isBest) bestCycleIndex = i;
          
          // Quality indicator
          const char* quality = (entry.score < AutoTune::EXCELLENT_SCORE) ? "EXCLNT" :
                               (entry.score < AutoTune::GOOD_SCORE) ? "GOOD  " :
                               (entry.score < AutoTune::ACCEPTABLE_SCORE) ? "OK    " : "POOR  ";
          
          Serial.printf("%s%-2d   | %.3f | %.3f | %.3f | %5.1f | %5.1f%%   | %s\n",
                       isBest ? "→ " : "  ",
                       entry.cycleNumber,
                       entry.testKp, entry.testKi, entry.testKd,
                       entry.score, accuracy, quality);
        }
        
        Serial.println("==========================================");
        Serial.printf("BEST CYCLE: #%d (Score: %.2f, Accuracy: %.1f%%)\n",
                     _historyTable[bestCycleIndex].cycleNumber,
                     _historyTable[bestCycleIndex].score,
                     _historyTable[bestCycleIndex].getAccuracyPercent());
        Serial.println("==========================================\n");
        
        // NOTE: Best parameters sudah di-save real-time setiap improvement
        // Final save hanya untuk memastikan consistency
        Serial.println("\nFinal save verification...");
        if (storage_savePidParameters(_tuningMotorId, _bestKp, _bestKi, _bestKd)) {
          Serial.println("✓ Final parameters verified in flash");
        } else {
          Serial.println("✗ WARNING: Final save failed (best params may already be saved)");
        }
        
        // LCD feedback lengkap untuk inform user tuning DONE
        char line1[17];
        snprintf(line1, sizeof(line1), "M%d: DONE!", _tuningMotorId + 1);
        
        if (_bestScore < AutoTune::EXCELLENT_SCORE) {
          lcd_debugMessage(line1, "Quality: EXCLNT");
        } else if (_bestScore < AutoTune::GOOD_SCORE) {
          lcd_debugMessage(line1, "Quality: GOOD");
        } else if (_bestScore < AutoTune::ACCEPTABLE_SCORE) {
          lcd_debugMessage(line1, "Quality: OK");
        } else {
          lcd_debugMessage(line1, "Quality: POOR");
        }
        
        delay(3000);  // Show result selama 3 detik
        
        // Show saved values
        lcd_debugPidValues(_tuningMotorId, _bestKp, _bestKi, _bestKd);
        delay(3000);
      }
      
      Serial.println("=====================================\n");
      
      // Clear LCD untuk indicate tuning finished
      lcd_clear();
      lcd_debugMessage("Tuning Complete", "Ready for next");
      delay(2000);
      
      _autotuneState = AUTOTUNE_IDLE;
    }
      break;
      
    default:
      break;
  }
}

/**
 * @brief Cancel ongoing auto-tuning
 */
void autotuning_cancel() {
  if (_autotuneState != AUTOTUNE_IDLE) {
    Serial.println("\n[AutoTune] CANCELLED by user");
    
    // Stop motor
    control_setRpmAllWheels(0.0f, 0.0f, 0.0f);
    
    // Restore saved PID
    // (akan di-implement dengan reload dari preferences)
    
    _autotuneState = AUTOTUNE_IDLE;
  }
}

/**
 * @brief Get tuning progress (0-100%)
 */
uint8_t autotuning_getProgress() {
  if (_autotuneState == AUTOTUNE_IDLE) return 0;
  if (_autotuneState == AUTOTUNE_FINISHED) return 100;
  
  return (_tuningCycle * 100) / AutoTune::MAX_TUNING_CYCLES;
}

/**
 * @brief Check if tuning is active
 */
bool autotuning_isActive() {
  return (_autotuneState != AUTOTUNE_IDLE);
}
