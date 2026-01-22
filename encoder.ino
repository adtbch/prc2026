/**
 * ============================================================
 * FILE: encoder.ino
 * LAYER: Hardware Abstraction Layer
 * ============================================================
 * 
 * DESCRIPTION:
 * Quadrature encoder reading menggunakan interrupt untuk tracking
 * wheel rotation. Menghitung RPM untuk closed-loop speed control.
 * 
 * FEATURES:
 * - Interrupt-based quadrature decoding (tidak miss pulses)
 * - RPM calculation dengan interval timing
 * - Signed count (track arah rotasi)
 * 
 * HARDWARE:
 * - 4 quadrature encoders (2 channels: A & B per encoder)
 * - ESP32 GPIO interrupt support
 * - PPR: 210 pulses per revolution
 * 
 * PUBLIC FUNCTIONS:
 * - hardware_initializeEncoders()
 * - hardware_updateEncoderRpm()
 * - hardware_getEncoderRpm(motorId)
 * - hardware_getEncoderCount(motorId)
 * - hardware_resetEncoders()
 * 
 * ============================================================
 */

// ══════════════════════════════════════════════════════════
// GLOBAL VARIABLES
// ══════════════════════════════════════════════════════════

// Encoder pulse counts (signed - bisa negatif untuk reverse)
volatile long encoderCount[4] = {0, 0, 0, 0};

// Pulse count absolut untuk RPM calculation (selalu positif)
static volatile unsigned long _encoderPulseCount[4] = {0, 0, 0, 0};

// Current RPM values (updated setiap INTERVAL_MS)
float encoderRpm[4] = {0.0f, 0.0f, 0.0f, 0.0f};

// Last state untuk quadrature decoding
static uint8_t _lastStateEncoder[4] = {0, 0, 0, 0};

// Timing untuk RPM calculation
static unsigned long _lastRpmUpdateTime = 0;

// ══════════════════════════════════════════════════════════
// INTERRUPT SERVICE ROUTINES (ISR)
// ══════════════════════════════════════════════════════════
// CATATAN: ISR harus se-simple mungkin (fast execution)
// ══════════════════════════════════════════════════════════

/**
 * @brief ISR untuk Encoder 1 (Motor 1)
 * 
 * Dipanggil saat pin A berubah (RISING atau FALLING edge)
 * Quadrature decoding: compare state A vs B untuk deteksi arah
 */
void IRAM_ATTR _isr_encoder1() {
  bool stateA = digitalRead(Pin::ENCODER1_A);
  bool stateB = digitalRead(Pin::ENCODER1_B);
  
  // Quadrature logic: jika A == B -> forward, jika A != B -> backward
  if (stateA == stateB) {
    encoderCount[0]++;
  } else {
    encoderCount[0]--;
  }
  
  _encoderPulseCount[0]++;  // Selalu increment (untuk RPM)
}

/**
 * @brief ISR untuk Encoder 2 (Motor 2)
 */
void IRAM_ATTR _isr_encoder2() {
  bool stateA = digitalRead(Pin::ENCODER2_A);
  bool stateB = digitalRead(Pin::ENCODER2_B);
  
  if (stateA == stateB) {
    encoderCount[1]++;
  } else {
    encoderCount[1]--;
  }
  
  _encoderPulseCount[1]++;
}

/**
 * @brief ISR untuk Encoder 3 (Motor 3)
 */
void IRAM_ATTR _isr_encoder3() {
  bool stateA = digitalRead(Pin::ENCODER3_A);
  bool stateB = digitalRead(Pin::ENCODER3_B);
  
  if (stateA == stateB) {
    encoderCount[2]++;
  } else {
    encoderCount[2]--;
  }
  
  _encoderPulseCount[2]++;
}

/**
 * @brief ISR untuk Encoder 4 (Motor 4)
 */
void IRAM_ATTR _isr_encoder4() {
  bool stateA = digitalRead(Pin::ENCODER4_A);
  bool stateB = digitalRead(Pin::ENCODER4_B);
  
  if (stateA == stateB) {
    encoderCount[3]++;
  } else {
    encoderCount[3]--;
  }
  
  _encoderPulseCount[3]++;
}

// ══════════════════════════════════════════════════════════
// PUBLIC FUNCTIONS
// ══════════════════════════════════════════════════════════

/**
 * @brief Initialize encoder pins dan attach interrupts
 * 
 * Setup pin mode sebagai INPUT dan attach ISR ke setiap encoder channel A.
 * Interrupt dipasang pada CHANGE (trigger both rising & falling edge).
 */
void hardware_initializeEncoders() {
  // Setup pin modes
  pinMode(Pin::ENCODER1_A, INPUT);
  pinMode(Pin::ENCODER1_B, INPUT);
  pinMode(Pin::ENCODER2_A, INPUT);
  pinMode(Pin::ENCODER2_B, INPUT);
  pinMode(Pin::ENCODER3_A, INPUT);
  pinMode(Pin::ENCODER3_B, INPUT);
  pinMode(Pin::ENCODER4_A, INPUT);
  pinMode(Pin::ENCODER4_B, INPUT);
  
  // Attach interrupts pada channel A (CHANGE = rising + falling)
  attachInterrupt(digitalPinToInterrupt(Pin::ENCODER1_A), _isr_encoder1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(Pin::ENCODER2_A), _isr_encoder2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(Pin::ENCODER3_A), _isr_encoder3, CHANGE);
  attachInterrupt(digitalPinToInterrupt(Pin::ENCODER4_A), _isr_encoder4, CHANGE);
  
  // Initialize timing
  _lastRpmUpdateTime = millis();
}

/**
 * @brief Update RPM calculation untuk semua encoders
 * 
 * Dipanggil di loop() setiap cycle. Menghitung RPM berdasarkan
 * pulse count sejak update terakhir.
 * 
 * Formula:
 * RPM = (pulseCount * 60000) / (interval_ms * PPR)
 * 
 * Dimana:
 * - pulseCount = jumlah pulse dalam interval
 * - 60000 = konversi ms ke menit (60 sec * 1000 ms)
 * - interval_ms = waktu sejak update terakhir
 * - PPR = pulse per revolution
 */
void hardware_updateEncoderRpm() {
  unsigned long currentTime = millis();
  unsigned long deltaTime = currentTime - _lastRpmUpdateTime;
  
  // Update hanya setiap INTERVAL_MS untuk stability
  if (deltaTime >= Encoder::RPM_UPDATE_INTERVAL_MS) {
    // Konstanta untuk konversi: (60000 ms/min) / (interval * PPR)
    float conversionFactor = 60000.0f / (deltaTime * Encoder::PULSES_PER_REVOLUTION);
    
    // Hitung RPM untuk setiap motor
    // Disable interrupt sementara untuk atomic read
    noInterrupts();
    for (uint8_t i = 0; i < 4; i++) {
      encoderRpm[i] = _encoderPulseCount[i] * conversionFactor;
      _encoderPulseCount[i] = 0;  // Reset counter
    }
    interrupts();
    
    _lastRpmUpdateTime = currentTime;
  }
}

/**
 * @brief Ambil RPM value untuk satu motor
 * 
 * @param motorId Motor ID (0-3)
 * @return Current RPM value (updated setiap INTERVAL_MS)
 */
float hardware_getEncoderRpm(uint8_t motorId) {
  if (motorId >= 4) return 0.0f;
  return encoderRpm[motorId];
}

/**
 * @brief Ambil pulse count untuk satu motor
 * 
 * Pulse count adalah signed value (bisa negatif untuk reverse).
 * Berguna untuk odometry calculation.
 * 
 * @param motorId Motor ID (0-3)
 * @return Accumulated pulse count (signed)
 */
long hardware_getEncoderCount(uint8_t motorId) {
  if (motorId >= 4) return 0;
  
  // Atomic read (interrupt-safe)
  noInterrupts();
  long count = encoderCount[motorId];
  interrupts();
  
  return count;
}

/**
 * @brief Reset semua encoder counts ke 0
 * 
 * Berguna untuk reset odometry atau saat mulai waypoint baru
 */
void hardware_resetEncoders() {
  noInterrupts();
  for (uint8_t i = 0; i < 4; i++) {
    encoderCount[i] = 0;
    _encoderPulseCount[i] = 0;
    encoderRpm[i] = 0.0f;
  }
  interrupts();
  
  _lastRpmUpdateTime = millis();
}
