

#include "config.h"
#include "ps3.h"
//manual control 
struct varEEPROM {
  int SPEED[3] = { 300, 400, 600 };
  int SPEEDR[3] = { 60, 80, 100 };
  double KP[3] = { 0.1, 0.15, 0.2 };
  double KA[3] = { 0.2, 0.4, 0.8 };
};
// struct varEEPROM {
//   int SPEED[3] = { 250, 600, 900 };
//   int SPEEDR[3] = { 180, 200, 400 };
//   double KP[3] = { 0.07, 0.2, 0.3 };
//   double KA[3] = { 0.01, 0.04, 0.1 };
// };
varEEPROM EE;

int SPEED;
int SPEEDR;
int rpmX, rpmY, rpmW;
bool setMode = 0, changeValue = 0;
int modeSpeed = 1;
void setup() {  
  // Setup pin mode untuk semua motor
  pinMode(MOTOR1_A, OUTPUT);
  pinMode(MOTOR1_B, OUTPUT);
  pinMode(MOTOR2_A, OUTPUT);
  pinMode(MOTOR2_B, OUTPUT);
  pinMode(MOTOR3_A, OUTPUT);
  pinMode(MOTOR3_B, OUTPUT);
  pinMode(MOTOR4_A, OUTPUT);
  pinMode(MOTOR4_B, OUTPUT);

  // Setup PWM untuk semua motor
  setupPwmChannel(M1_PWM_CHANNEL, EN_MOTOR_1);
  setupPwmChannel(M2_PWM_CHANNEL, EN_MOTOR_2);
  setupPwmChannel(M3_PWM_CHANNEL, EN_MOTOR_3);
  setupPwmChannel(M4_PWM_CHANNEL, EN_MOTOR_4);

  // Default: semua motor berhenti
  pwmMotor(0, 0, 0);

  // Setup encoder dengan sistem baru
  setupEncoder();

  // Inisialisasi MPU6050/MPU9250
  setupMPU();

  // Inisialisasi LCD
  initLCD();
  setupPS3();
  // Odom dari encoder (X kanan, Y maju)
  setupOdometry();
}

void loop() {

  // Pembacaan MPU (yaw, pitch, roll)
  mpu6500();
  
  // Pembacaan RPM (otomatis setiap interval)
  pembacaan_RPM();

  // Update odometry dari encoder count
  updateOdometry();
  


  
  // Update tampilan RPM di LCD
  // displayRpmStatus();
  // pwmMotor(600, 500, 400);
  // rpmMotor(300, -300/2, -300/2); 
  // moveRobotRpm(300, 0, 0, 300);
  // moveRobotRpmGlobal(200, 0, 0, 200);
  // moveRobotRpmYawHold(200, 0, 0, 200);
  // moveRobotRpmYawHoldGlobal(0, 0, 0, 300);
  // Contoh: 0.5 = 50 cm
  // pidKinematik(0, 0, 0, 200, 50);
  

  // runWaypoints();
  // executeWaypoint(0, 1000, 0, 10, 5, 200, 50);
  // moveRobot(0, 1000, 0, 1000);
  // displayYawStatus();
  // kecepatanxy2Global(0, 200, 0);
  control_robot();
  // Tampil odom + delta encoder M3/M4
  displayOdomStatus();
}

void control_robot() {
  if (R3 == 1) {
    rpmMotor(0, 0, 0);
  } else {
    // Jalankan manual control jika R3 tidak ditekan
    // manual_control();
    manual_control2();
  }
}