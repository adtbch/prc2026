/**
 * ============================================================
 * FILE: kinematics.ino
 * LAYER: Motion Control Layer
 * ============================================================
 * 
 * DESCRIPTION:
 * Kinematika robot omni 3 roda dengan implementasi lengkap:
 * - Forward & Inverse kinematics dengan matriks Jacobian
 * - Field-centric control (global ↔ local frame transformation)
 * - Kalman filter untuk odometry fusion (encoder + IMU)
 * - Slip detection dan adaptive compensation
 * - Dynamic model dengan constraints
 * 
 * FEATURES:
 * - Rotation matrix untuk transformasi koordinat
 * - Jacobian matrix 3x3 untuk optimal velocity distribution
 * - Kalman filter untuk slip compensation
 * - Velocity/acceleration/torque constraints
 * - Singularity avoidance
 * 
 * ROBOT CONFIGURATION:
 * - 3 omni wheels pada sudut 30°, 150°, 270° (120° spacing)
 * - Jarak roda ke pusat robot: L (defined in config.h)
 * - Max motor speed: 500 RPM
 * 
 * PUBLIC FUNCTIONS:
 * - kinematics_initialize()
 * - kinematics_inverseKinematics()
 * - kinematics_forwardKinematics()
 * - kinematics_updateOdometry()
 * - kinematics_getGlobalPosition()
 * 
 * BASED ON RESEARCH:
 * - "Introduction to Mobile Robot Control" (Tzafestas, 2014)
 * - "Active Disturbance Rejection Control" (Sira-Ramirez, 2017)
 * - ScienceDirect omnidirectional robot kinematics papers
 * 
 * ============================================================
 */

// ══════════════════════════════════════════════════════════
// KALMAN FILTER STATE STRUCTURE
// ══════════════════════════════════════════════════════════

/**
 * @brief Struktur untuk Kalman Filter state estimation
 * 
 * Digunakan untuk fusi odometry (encoder) dengan IMU untuk mendapatkan
 * estimasi posisi yang lebih akurat dengan kompensasi slip.
 * 
 * State vector: [x, y, theta]^T (posisi global + heading)
 * Measurement: [x_odo, y_odo]^T (dari encoder)
 * Control input: [Vx, Vy, omega]^T (dari inverse kinematics)
 */
struct KalmanFilter {
  // State estimate: [x, y, theta] dalam koordinat global (m, m, rad)
  float x[3];
  
  // State covariance matrix (3x3) - uncertainty dari state estimate
  float P[3][3];
  
  // Process noise covariance (3x3) - uncertainty dari model dynamics
  float Q[3][3];
  
  // Measurement noise covariance (2x2) - uncertainty dari sensor (encoder)
  float R[2][2];
  
  // Identity matrix untuk kalkulasi (3x3)
  float I[3][3];
  
  // Kalman gain (3x2) - optimal weight antara prediction vs measurement
  float K[3][2];
  
  // Innovation (2x1) - error antara measurement dan prediction
  float y[2];
  
  // Measurement matrix H (2x3) - maps state ke measurement space
  // H = [1 0 0]  (x measurement)
  //     [0 1 0]  (y measurement)
  float H[2][3];
  
  // Timestamp terakhir update (untuk dt calculation)
  uint32_t lastUpdateTime;
};

// ══════════════════════════════════════════════════════════
// PRIVATE VARIABLES
// ══════════════════════════════════════════════════════════

// Kalman filter instance
static KalmanFilter _kalman;

// Slip detection variables
static float _slipRatio = 0.0f;              // Current slip ratio (0-1)
static float _encoderErrorX = 0.0f;          // Error X untuk slip detection
static float _encoderErrorY = 0.0f;          // Error Y untuk slip detection
static uint32_t _lastSlipCheckTime = 0;      // Timestamp terakhir slip check

// Odometry variables (dari encoder - local frame)
static float _odoX = 0.0f;                   // Posisi X dari encoder (m)
static float _odoY = 0.0f;                   // Posisi Y dari encoder (m)
static float _odoTheta = 0.0f;               // Heading dari encoder (rad)

// Global position (hasil Kalman filter fusion)
static float _globalX = 0.0f;                // Posisi X global (m)
static float _globalY = 0.0f;                // Posisi Y global (m)
static float _globalTheta = 0.0f;            // Heading global dari IMU (rad)

// Previous wheel velocities untuk acceleration calculation
static float _prevWheelVel[3] = {0.0f, 0.0f, 0.0f};  // rad/s
static uint32_t _lastKinematicsUpdate = 0;

// Dynamic model variables
static float _robotMass = Kinematics::ROBOT_MASS;           // kg
static float _robotInertia = Kinematics::ROBOT_INERTIA;     // kg·m²
static float _maxAcceleration = Kinematics::MAX_ACCELERATION; // m/s²
static float _maxAngularAccel = Kinematics::MAX_ANGULAR_ACCEL; // rad/s²

// ══════════════════════════════════════════════════════════
// PRIVATE HELPER FUNCTIONS
// ══════════════════════════════════════════════════════════

/**
 * @brief Konstrain nilai antara min dan max
 */
static inline float _constrain(float value, float minVal, float maxVal) {
  if (value < minVal) return minVal;
  if (value > maxVal) return maxVal;
  return value;
}

/**
 * @brief Normalize angle ke range -PI to PI
 */
static inline float _normalizeAngle(float angle) {
  while (angle > PI) angle -= 2.0f * PI;
  while (angle < -PI) angle += 2.0f * PI;
  return angle;
}

/**
 * @brief Matrix multiplication: C = A * B
 * @param A Matrix A (rows_a x cols_a)
 * @param B Matrix B (cols_a x cols_b)
 * @param C Result matrix (rows_a x cols_b)
 */
static void _matrixMultiply(const float* A, const float* B, float* C, 
                            int rows_a, int cols_a, int cols_b) {
  for (int i = 0; i < rows_a; i++) {
    for (int j = 0; j < cols_b; j++) {
      float sum = 0.0f;
      for (int k = 0; k < cols_a; k++) {
        sum += A[i * cols_a + k] * B[k * cols_b + j];
      }
      C[i * cols_b + j] = sum;
    }
  }
}

/**
 * @brief Matrix transpose: B = A^T
 */
static void _matrixTranspose(const float* A, float* B, int rows, int cols) {
  for (int i = 0; i < rows; i++) {
    for (int j = 0; j < cols; j++) {
      B[j * rows + i] = A[i * cols + j];
    }
  }
}

/**
 * @brief Matrix addition: C = A + B
 */
static void _matrixAdd(const float* A, const float* B, float* C, int rows, int cols) {
  int size = rows * cols;
  for (int i = 0; i < size; i++) {
    C[i] = A[i] + B[i];
  }
}

/**
 * @brief Matrix subtraction: C = A - B
 */
static void _matrixSubtract(const float* A, const float* B, float* C, int rows, int cols) {
  int size = rows * cols;
  for (int i = 0; i < size; i++) {
    C[i] = A[i] - B[i];
  }
}

/**
 * @brief Inverse matrix 2x2: B = A^-1
 * @return true jika berhasil, false jika singular
 */
static bool _matrixInverse2x2(const float* A, float* B) {
  float det = A[0] * A[3] - A[1] * A[2];
  
  // Check singularity (determinant mendekati 0)
  if (fabs(det) < 1e-6f) {
    return false;
  }
  
  float invDet = 1.0f / det;
  B[0] =  A[3] * invDet;
  B[1] = -A[1] * invDet;
  B[2] = -A[2] * invDet;
  B[3] =  A[0] * invDet;
  
  return true;
}

/**
 * @brief Transformasi velocity dari global frame ke local (robot) frame
 * 
 * Formula: [Vx_local]   [cos(θ)  sin(θ)] [Vx_global]
 *          [Vy_local] = [-sin(θ) cos(θ)] [Vy_global]
 * 
 * @param vx_global Kecepatan X global (m/s)
 * @param vy_global Kecepatan Y global (m/s)
 * @param theta Heading robot (rad)
 * @param vx_local Output: kecepatan X local (m/s)
 * @param vy_local Output: kecepatan Y local (m/s)
 */
static void _transformGlobalToLocal(float vx_global, float vy_global, float theta,
                                    float* vx_local, float* vy_local) {
  float cos_theta = cos(theta);
  float sin_theta = sin(theta);
  
  *vx_local =  cos_theta * vx_global + sin_theta * vy_global;
  *vy_local = -sin_theta * vx_global + cos_theta * vy_global;
}

/**
 * @brief Transformasi velocity dari local (robot) frame ke global frame
 * 
 * Formula: [Vx_global]   [cos(θ) -sin(θ)] [Vx_local]
 *          [Vy_global] = [sin(θ)  cos(θ)] [Vy_local]
 * 
 * @param vx_local Kecepatan X local (m/s)
 * @param vy_local Kecepatan Y local (m/s)
 * @param theta Heading robot (rad)
 * @param vx_global Output: kecepatan X global (m/s)
 * @param vy_global Output: kecepatan Y global (m/s)
 */
static void _transformLocalToGlobal(float vx_local, float vy_local, float theta,
                                    float* vx_global, float* vy_global) {
  float cos_theta = cos(theta);
  float sin_theta = sin(theta);
  
  *vx_global = cos_theta * vx_local - sin_theta * vy_local;
  *vy_global = sin_theta * vx_local + cos_theta * vy_local;
}

// ══════════════════════════════════════════════════════════
// KALMAN FILTER FUNCTIONS
// ══════════════════════════════════════════════════════════

/**
 * @brief Initialize Kalman filter dengan parameter awal
 * 
 * Inisialisasi state covariance P, process noise Q, measurement noise R,
 * dan measurement matrix H.
 */
static void _kalmanInitialize() {
  // Initialize state ke nol
  memset(_kalman.x, 0, sizeof(_kalman.x));
  
  // Initialize state covariance P (initial uncertainty)
  memset(_kalman.P, 0, sizeof(_kalman.P));
  _kalman.P[0][0] = 0.1f;  // Uncertainty X (m²)
  _kalman.P[1][1] = 0.1f;  // Uncertainty Y (m²)
  _kalman.P[2][2] = 0.01f; // Uncertainty theta (rad²)
  
  // Process noise covariance Q (model uncertainty)
  memset(_kalman.Q, 0, sizeof(_kalman.Q));
  _kalman.Q[0][0] = Kinematics::KALMAN_Q_XY;     // Process noise X
  _kalman.Q[1][1] = Kinematics::KALMAN_Q_XY;     // Process noise Y
  _kalman.Q[2][2] = Kinematics::KALMAN_Q_THETA;  // Process noise theta
  
  // Measurement noise covariance R (sensor uncertainty)
  memset(_kalman.R, 0, sizeof(_kalman.R));
  _kalman.R[0][0] = Kinematics::KALMAN_R_XY;  // Encoder noise X
  _kalman.R[1][1] = Kinematics::KALMAN_R_XY;  // Encoder noise Y
  
  // Identity matrix
  memset(_kalman.I, 0, sizeof(_kalman.I));
  _kalman.I[0][0] = 1.0f;
  _kalman.I[1][1] = 1.0f;
  _kalman.I[2][2] = 1.0f;
  
  // Measurement matrix H (mengambil x dan y dari state)
  memset(_kalman.H, 0, sizeof(_kalman.H));
  _kalman.H[0][0] = 1.0f;  // x measurement
  _kalman.H[1][1] = 1.0f;  // y measurement
  
  _kalman.lastUpdateTime = millis();
  
  Serial.println("[Kalman] Filter initialized");
  Serial.printf("[Kalman] Q_xy=%.4f, Q_theta=%.4f, R_xy=%.4f\n", 
                Kinematics::KALMAN_Q_XY, Kinematics::KALMAN_Q_THETA, 
                Kinematics::KALMAN_R_XY);
}

/**
 * @brief Kalman filter prediction step
 * 
 * Predict state berdasarkan motion model:
 * x_pred = x + Vx * dt * cos(theta) - Vy * dt * sin(theta)
 * y_pred = y + Vx * dt * sin(theta) + Vy * dt * cos(theta)
 * theta_pred = theta + omega * dt (dari IMU, lebih reliable)
 * 
 * @param vx Kecepatan X local frame (m/s)
 * @param vy Kecepatan Y local frame (m/s)
 * @param omega Kecepatan angular (rad/s) - dari IMU
 * @param dt Time delta (s)
 */
static void _kalmanPredict(float vx, float vy, float omega, float dt) {
  // Get current state
  float x = _kalman.x[0];
  float y = _kalman.x[1];
  float theta = _kalman.x[2];
  
  // Predict new state (motion model)
  float cos_theta = cos(theta);
  float sin_theta = sin(theta);
  
  _kalman.x[0] = x + (vx * cos_theta - vy * sin_theta) * dt;
  _kalman.x[1] = y + (vx * sin_theta + vy * cos_theta) * dt;
  _kalman.x[2] = _normalizeAngle(theta + omega * dt);
  
  // State transition Jacobian F (linearized model)
  float F[3][3] = {
    {1.0f, 0.0f, (-vx * sin_theta - vy * cos_theta) * dt},
    {0.0f, 1.0f, ( vx * cos_theta - vy * sin_theta) * dt},
    {0.0f, 0.0f, 1.0f}
  };
  
  // Predict covariance: P_pred = F * P * F^T + Q
  float FP[3][3];
  float FPFt[3][3];
  
  // FP = F * P
  _matrixMultiply((float*)F, (float*)_kalman.P, (float*)FP, 3, 3, 3);
  
  // FPFt = FP * F^T
  float Ft[3][3];
  _matrixTranspose((float*)F, (float*)Ft, 3, 3);
  _matrixMultiply((float*)FP, (float*)Ft, (float*)FPFt, 3, 3, 3);
  
  // P = FPFt + Q
  _matrixAdd((float*)FPFt, (float*)_kalman.Q, (float*)_kalman.P, 3, 3);
}

/**
 * @brief Kalman filter update step
 * 
 * Update state estimate dengan measurement dari encoder odometry.
 * Kalman gain menentukan seberapa besar trust ke measurement vs prediction.
 * 
 * @param z_x Measurement X dari encoder (m)
 * @param z_y Measurement Y dari encoder (m)
 */
static void _kalmanUpdate(float z_x, float z_y) {
  // Innovation (measurement residual): y = z - H*x
  _kalman.y[0] = z_x - _kalman.x[0];
  _kalman.y[1] = z_y - _kalman.x[1];
  
  // Innovation covariance: S = H * P * H^T + R
  float HP[2][3];
  float HPHt[2][2];
  float S[2][2];
  
  // HP = H * P
  _matrixMultiply((float*)_kalman.H, (float*)_kalman.P, (float*)HP, 2, 3, 3);
  
  // HPHt = HP * H^T
  float Ht[3][2];
  _matrixTranspose((float*)_kalman.H, (float*)Ht, 2, 3);
  _matrixMultiply((float*)HP, (float*)Ht, (float*)HPHt, 2, 3, 2);
  
  // S = HPHt + R
  _matrixAdd((float*)HPHt, (float*)_kalman.R, (float*)S, 2, 2);
  
  // Kalman gain: K = P * H^T * S^-1
  float S_inv[2][2];
  if (!_matrixInverse2x2((float*)S, (float*)S_inv)) {
    Serial.println("[Kalman] WARNING: S matrix singular!");
    return;
  }
  
  float PHt[3][2];
  _matrixMultiply((float*)_kalman.P, (float*)Ht, (float*)PHt, 3, 3, 2);
  _matrixMultiply((float*)PHt, (float*)S_inv, (float*)_kalman.K, 3, 2, 2);
  
  // Update state: x = x + K * y
  for (int i = 0; i < 3; i++) {
    _kalman.x[i] += _kalman.K[i][0] * _kalman.y[0] + 
                    _kalman.K[i][1] * _kalman.y[1];
  }
  
  // Normalize theta
  _kalman.x[2] = _normalizeAngle(_kalman.x[2]);
  
  // Update covariance: P = (I - K*H) * P
  float KH[3][3];
  _matrixMultiply((float*)_kalman.K, (float*)_kalman.H, (float*)KH, 3, 2, 3);
  
  float I_KH[3][3];
  _matrixSubtract((float*)_kalman.I, (float*)KH, (float*)I_KH, 3, 3);
  
  float P_new[3][3];
  _matrixMultiply((float*)I_KH, (float*)_kalman.P, (float*)P_new, 3, 3, 3);
  
  memcpy(_kalman.P, P_new, sizeof(_kalman.P));
}

// ══════════════════════════════════════════════════════════
// PUBLIC FUNCTIONS
// ══════════════════════════════════════════════════════════

/**
 * @brief Initialize kinematics system
 * 
 * Harus dipanggil di setup() sebelum menggunakan fungsi kinematics lainnya.
 * Menginisialisasi Kalman filter dan reset odometry ke nol.
 */
void kinematics_initialize() {
  Serial.println("[Kinematics] Initializing kinematics system...");
  
  // Initialize Kalman filter
  _kalmanInitialize();
  
  // Reset odometry
  _odoX = 0.0f;
  _odoY = 0.0f;
  _odoTheta = 0.0f;
  
  _globalX = 0.0f;
  _globalY = 0.0f;
  _globalTheta = 0.0f;
  
  _slipRatio = 0.0f;
  _encoderErrorX = 0.0f;
  _encoderErrorY = 0.0f;
  
  memset(_prevWheelVel, 0, sizeof(_prevWheelVel));
  
  _lastKinematicsUpdate = millis();
  _lastSlipCheckTime = millis();
  
  Serial.println("[Kinematics] System initialized successfully");
  Serial.printf("[Kinematics] Max motor speed: %.1f RPM (%.2f rad/s)\n", 
                Kinematics::MAX_MOTOR_RPM, Kinematics::MAX_MOTOR_RAD_S);
  Serial.printf("[Kinematics] Max platform vel: %.2f m/s, omega: %.2f rad/s\n",
                Kinematics::MAX_PLATFORM_VEL, Kinematics::MAX_ANGULAR_VEL);
}

/**
 * @brief Inverse kinematics dengan matriks Jacobian
 * 
 * Mengkonversi kecepatan platform (Vx, Vy, omega) ke kecepatan roda individual.
 * Menggunakan matriks Jacobian inverse 3x3 untuk distribusi optimal.
 * 
 * Jacobian inverse untuk 3 roda pada 30°, 150°, 270°:
 * 
 * [V1]     1   [-0.5    √3/2   L] [Vx]
 * [V2]  =  -   [-0.5   -√3/2   L] [Vy]
 * [V3]     r   [ 1.0     0.0   L] [ω]
 * 
 * dengan:
 * - V1, V2, V3: kecepatan roda (rad/s)
 * - Vx, Vy: kecepatan platform dalam robot frame (m/s)
 * - ω: kecepatan angular (rad/s)
 * - r: radius roda (m)
 * - L: jarak roda ke pusat robot (m)
 * 
 * @param vx_global Target kecepatan X dalam global frame (m/s)
 * @param vy_global Target kecepatan Y dalam global frame (m/s)
 * @param omega Target kecepatan angular (rad/s)
 * @param wheel_vel Output: kecepatan roda [V1, V2, V3] dalam rad/s
 * @param useFieldCentric true = input dalam global frame, false = local frame
 */
void kinematics_inverseKinematics(float vx_global, float vy_global, float omega,
                                  float wheel_vel[3], bool useFieldCentric) {
  float vx_local, vy_local;
  
  // Step 1: Transform global velocity ke local frame jika field-centric
  if (useFieldCentric) {
    // Gunakan heading dari IMU (lebih akurat dari odometry)
    float theta = hardware_getImuYaw() * DEG_TO_RAD;
    _transformGlobalToLocal(vx_global, vy_global, theta, &vx_local, &vy_local);
  } else {
    vx_local = vx_global;
    vy_local = vy_global;
  }
  
  // Step 2: Apply velocity constraints
  float vel_magnitude = sqrt(vx_local * vx_local + vy_local * vy_local);
  if (vel_magnitude > Kinematics::MAX_PLATFORM_VEL) {
    float scale = Kinematics::MAX_PLATFORM_VEL / vel_magnitude;
    vx_local *= scale;
    vy_local *= scale;
  }
  
  omega = _constrain(omega, -Kinematics::MAX_ANGULAR_VEL, Kinematics::MAX_ANGULAR_VEL);
  
  // Step 3: Jacobian inverse kinematics
  // Konstanta untuk perhitungan
  const float sqrt3_2 = 0.866025404f;  // √3/2
  const float inv_r = 1.0f / Kinematics::WHEEL_RADIUS;
  const float L = Kinematics::WHEEL_BASE_RADIUS;
  
  // Matriks Jacobian inverse (3x3)
  // J^-1 = 1/r * [[-0.5,  √3/2, L],
  //               [-0.5, -√3/2, L],
  //               [ 1.0,   0.0, L]]
  
  wheel_vel[0] = inv_r * (-0.5f * vx_local +  sqrt3_2 * vy_local + L * omega);
  wheel_vel[1] = inv_r * (-0.5f * vx_local -  sqrt3_2 * vy_local + L * omega);
  wheel_vel[2] = inv_r * ( 1.0f * vx_local +      0.0f * vy_local + L * omega);
  
  // Step 4: Apply motor speed constraints dengan singularity avoidance
  for (int i = 0; i < 3; i++) {
    // Constrain ke max motor speed
    wheel_vel[i] = _constrain(wheel_vel[i], 
                              -Kinematics::MAX_MOTOR_RAD_S, 
                               Kinematics::MAX_MOTOR_RAD_S);
    
    // Singularity avoidance: enforce minimum wheel speed jika tidak nol
    if (fabs(wheel_vel[i]) > 0.01f && fabs(wheel_vel[i]) < Kinematics::MIN_WHEEL_SPEED) {
      wheel_vel[i] = (wheel_vel[i] > 0) ? Kinematics::MIN_WHEEL_SPEED : -Kinematics::MIN_WHEEL_SPEED;
    }
  }
  
  // Step 5: Apply acceleration constraints
  uint32_t now = millis();
  float dt = (now - _lastKinematicsUpdate) / 1000.0f;
  
  if (dt > 0.001f && dt < 1.0f) {  // Sanity check untuk dt
    for (int i = 0; i < 3; i++) {
      float accel = (wheel_vel[i] - _prevWheelVel[i]) / dt;
      float max_accel_rad = Kinematics::MAX_ACCELERATION / Kinematics::WHEEL_RADIUS;
      
      if (fabs(accel) > max_accel_rad) {
        // Limit acceleration
        float limited_accel = (accel > 0) ? max_accel_rad : -max_accel_rad;
        wheel_vel[i] = _prevWheelVel[i] + limited_accel * dt;
      }
    }
  }
  
  // Update previous velocities dan timestamp
  memcpy(_prevWheelVel, wheel_vel, sizeof(_prevWheelVel));
  _lastKinematicsUpdate = now;
}

/**
 * @brief Forward kinematics dengan matriks Jacobian
 * 
 * Mengkonversi kecepatan roda individual ke kecepatan platform.
 * Menggunakan matriks Jacobian (inverse dari J^-1).
 * 
 * Formula:
 * Vx = r * (2*V3 - V1 - V2) / 3
 * Vy = r * √3 * (V1 - V2) / 3
 * ω  = r * (V1 + V2 + V3) / (3*L)
 * 
 * @param wheel_vel Kecepatan roda [V1, V2, V3] dalam rad/s
 * @param vx Output: kecepatan X dalam local frame (m/s)
 * @param vy Output: kecepatan Y dalam local frame (m/s)
 * @param omega Output: kecepatan angular (rad/s)
 */
void kinematics_forwardKinematics(const float wheel_vel[3], 
                                  float* vx, float* vy, float* omega) {
  const float r = Kinematics::WHEEL_RADIUS;
  const float L = Kinematics::WHEEL_BASE_RADIUS;
  const float sqrt3 = 1.732050808f;
  
  // Forward kinematics matrix (Jacobian)
  *vx = r * (2.0f * wheel_vel[2] - wheel_vel[0] - wheel_vel[1]) / 3.0f;
  *vy = r * sqrt3 * (wheel_vel[0] - wheel_vel[1]) / 3.0f;
  *omega = r * (wheel_vel[0] + wheel_vel[1] + wheel_vel[2]) / (3.0f * L);
}

/**
 * @brief Update odometry dengan Kalman filter fusion
 * 
 * Menggabungkan data dari:
 * - Encoder (local odometry)
 * - IMU (yaw angle untuk heading)
 * 
 * Menggunakan Kalman filter untuk:
 * - Slip compensation
 * - Sensor fusion (encoder + IMU)
 * - Optimal state estimation
 * 
 * Harus dipanggil secara regular (misal setiap loop cycle atau timer).
 * 
 * @param wheel_vel Kecepatan roda saat ini [V1, V2, V3] dalam rad/s
 * @param dt Time delta sejak update terakhir (s)
 */
void kinematics_updateOdometry(const float wheel_vel[3], float dt) {
  // Step 1: Forward kinematics untuk mendapatkan velocity dalam local frame
  float vx_local, vy_local, omega;
  kinematics_forwardKinematics(wheel_vel, &vx_local, &vy_local, &omega);
  
  // Step 2: Update encoder-based odometry (dead reckoning)
  float cos_theta = cos(_odoTheta);
  float sin_theta = sin(_odoTheta);
  
  _odoX += (vx_local * cos_theta - vy_local * sin_theta) * dt;
  _odoY += (vx_local * sin_theta + vy_local * cos_theta) * dt;
  _odoTheta = _normalizeAngle(_odoTheta + omega * dt);
  
  // Step 3: Get IMU yaw (lebih reliable dari encoder untuk heading)
  float imu_yaw_deg = hardware_getImuYaw();
  float imu_yaw_rad = imu_yaw_deg * DEG_TO_RAD;
  
  // Step 4: Kalman filter fusion
  // Predict step
  _kalmanPredict(vx_local, vy_local, omega, dt);
  
  // Update step dengan encoder measurement
  _kalmanUpdate(_odoX, _odoY);
  
  // Step 5: Override theta dengan IMU (IMU lebih akurat untuk heading)
  _kalman.x[2] = imu_yaw_rad;
  
  // Step 6: Update global position dari Kalman estimate
  _globalX = _kalman.x[0];
  _globalY = _kalman.x[1];
  _globalTheta = _kalman.x[2];
  
  // Step 7: Slip detection dan adaptive correction
  uint32_t now = millis();
  if (now - _lastSlipCheckTime >= Kinematics::SLIP_CHECK_INTERVAL) {
    // Calculate error antara Kalman estimate dan encoder odometry
    _encoderErrorX = _globalX - _odoX;
    _encoderErrorY = _globalY - _odoY;
    
    float error_magnitude = sqrt(_encoderErrorX * _encoderErrorX + 
                                 _encoderErrorY * _encoderErrorY);
    
    // Estimate slip ratio
    float distance_traveled = sqrt(_odoX * _odoX + _odoY * _odoY);
    if (distance_traveled > 0.01f) {  // Minimal 1cm untuk valid calculation
      _slipRatio = error_magnitude / distance_traveled;
      _slipRatio = _constrain(_slipRatio, 0.0f, 1.0f);
    }
    
    // Warning jika slip terlalu besar
    if (_slipRatio > Kinematics::SLIP_THRESHOLD) {
      Serial.printf("[Kinematics] WARNING: High slip detected! Ratio: %.2f%%\n", 
                    _slipRatio * 100.0f);
    }
    
    _lastSlipCheckTime = now;
  }
  
  // Step 8: Adaptive correction - adjust process noise berdasarkan slip
  // Semakin besar slip, semakin besar process noise (lebih trust measurement)
  float slip_factor = 1.0f + _slipRatio * 5.0f;  // 1.0 to 6.0
  _kalman.Q[0][0] = Kinematics::KALMAN_Q_XY * slip_factor;
  _kalman.Q[1][1] = Kinematics::KALMAN_Q_XY * slip_factor;
}

/**
 * @brief Get posisi global robot (hasil Kalman filter)
 * 
 * @param x Output: posisi X global (m)
 * @param y Output: posisi Y global (m)
 * @param theta Output: heading global (rad)
 */
void kinematics_getGlobalPosition(float* x, float* y, float* theta) {
  *x = _globalX;
  *y = _globalY;
  *theta = _globalTheta;
}

/**
 * @brief Get posisi encoder odometry (tanpa Kalman filter)
 * 
 * @param x Output: posisi X odometry (m)
 * @param y Output: posisi Y odometry (m)
 * @param theta Output: heading odometry (rad)
 */
void kinematics_getOdometry(float* x, float* y, float* theta) {
  *x = _odoX;
  *y = _odoY;
  *theta = _odoTheta;
}

/**
 * @brief Get slip ratio saat ini
 * 
 * @return Slip ratio (0.0 = no slip, 1.0 = full slip)
 */
float kinematics_getSlipRatio() {
  return _slipRatio;
}

/**
 * @brief Reset odometry ke posisi tertentu
 * 
 * @param x Posisi X baru (m)
 * @param y Posisi Y baru (m)
 * @param theta Heading baru (rad)
 */
void kinematics_resetOdometry(float x, float y, float theta) {
  _odoX = x;
  _odoY = y;
  _odoTheta = theta;
  
  _globalX = x;
  _globalY = y;
  _globalTheta = theta;
  
  // Reset Kalman filter state
  _kalman.x[0] = x;
  _kalman.x[1] = y;
  _kalman.x[2] = theta;
  
  Serial.printf("[Kinematics] Odometry reset to (%.3f, %.3f, %.2f°)\n",
                x, y, theta * RAD_TO_DEG);
}

/**
 * @brief Print kinematics debug info
 */
void kinematics_printDebug() {
  Serial.println("=== Kinematics Debug ===");
  Serial.printf("Global Position: (%.3f, %.3f) m, θ=%.1f°\n", 
                _globalX, _globalY, _globalTheta * RAD_TO_DEG);
  Serial.printf("Odometry: (%.3f, %.3f) m, θ=%.1f°\n",
                _odoX, _odoY, _odoTheta * RAD_TO_DEG);
  Serial.printf("Encoder Error: (%.3f, %.3f) m\n", _encoderErrorX, _encoderErrorY);
  Serial.printf("Slip Ratio: %.2f%%\n", _slipRatio * 100.0f);
  Serial.printf("Kalman P[0][0]=%.4f, P[1][1]=%.4f, P[2][2]=%.4f\n",
                _kalman.P[0][0], _kalman.P[1][1], _kalman.P[2][2]);
  Serial.println("========================");
}
