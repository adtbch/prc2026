#include "config.h"
#include <MPU6050_6Axis_MotionApps20.h>

// Objek MPU6050
MPU6050 mpu;

// MPU control/status vars
bool dmpReady = false;
uint8_t mpuIntStatus;
uint8_t devStatus;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];

// Orientation/motion vars
Quaternion q;
VectorFloat gravity;
float ypr[3];

// Variabel global yaw, pitch, roll
float yaw = 0;
float pitch = 0;
float roll = 0;
float yaw0 = 0;  // Nilai offset/kalibrasi yaw
float yaw1 = 0;  // Yaw setelah dikurangi offset

void setupMPU() {
  // Initialize I2C
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock

  // Initialize MPU6050
  mpu.initialize();

  // Load and configure the DMP
  devStatus = mpu.dmpInitialize();

  // Supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788);

  // Make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // Calibration Time: generate offsets and calibrate our MPU6050
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu.PrintActiveOffsets();
    
    // Turn on the DMP, now that it's ready
    mpu.setDMPEnabled(true);

    mpuIntStatus = mpu.getIntStatus();
    dmpReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();
  }
}

void mpu6500() {
  if (!dmpReady) return;
  
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    
    yaw = ypr[0] * 180 / M_PI;
    pitch = ypr[1] * 180 / M_PI;
    roll = ypr[2] * 180 / M_PI;
    
    yaw = normalizeAngle(yaw, 0, 360);
    yaw1 = yaw - yaw0;
    yaw1 = normalizeAngle(yaw1, 0, 360);
  }
}

float degToRad(float deg) {
  return deg * (PI / 180.0);
}

float normalizeAngle(float angle, float minAngle, float maxAngle) {
  float range = maxAngle - minAngle;
  // Pakai while agar aman walaupun angle jauh di luar range (mis. drift/offset besar)
  while (angle < minAngle) angle += range;
  while (angle >= maxAngle) angle -= range;
  return angle;
}

// Fungsi untuk reset yaw (kalibrasi)
void resetYaw() {
  yaw0 = yaw;
  yaw1 = 0;
}

