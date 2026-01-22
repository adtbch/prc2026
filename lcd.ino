#include "config.h"
#include <LiquidCrystal_I2C.h>

// Inisialisasi LCD 16x2 dengan alamat I2C 0x68
LiquidCrystal_I2C lcd(0x27, 16, 2);

void initLCD() {
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("PRC2026 Encoder");
  lcd.setCursor(0, 1);
  lcd.print("Initializing...");
  delay(1000);
  lcd.clear();
}

void displayEncoderStatus() {
  // Baris 1: Motor 1 dan Motor 2
  lcd.setCursor(0, 0);
  lcd.print("M1:");
  lcd.print(encoder1Count);
  lcd.print("    "); // Padding untuk clear sisa karakter
  lcd.setCursor(8, 0);
  lcd.print("M2:");
  lcd.print(encoder2Count);
  lcd.print("    ");

  // Baris 2: Motor 3 dan Motor 4
  lcd.setCursor(0, 1);
  lcd.print("M3:");
  lcd.print(encoder3Count);
  lcd.print("    ");
  lcd.setCursor(8, 1);
  lcd.print("M4:");
  lcd.print(encoder4Count);
  lcd.print("    ");
}

void displayRpmStatus() {
  // Baris 1: RPM Motor 1 dan Motor 2
  lcd.setCursor(0, 0);
  lcd.print("M1:");
  lcd.print((int)getRPM_Motor1());
  lcd.print("    "); // Padding untuk clear sisa karakter
  lcd.setCursor(8, 0);
  lcd.print("M2:");
  lcd.print((int)getRPM_Motor2());
  lcd.print("    ");

  // Baris 2: RPM Motor 3 dan Motor 4
  lcd.setCursor(0, 1);
  lcd.print("M3:");
  lcd.print((int)getRPM_Motor3());
  lcd.print("    ");
  lcd.setCursor(8, 1);
  lcd.print("M4:");
  lcd.print((int)getRPM_Motor4());
  lcd.print("    ");
}

void displayYawStatus() {
  // Baris 1: Yaw
  lcd.setCursor(0, 0);
  lcd.print("Yaw: ");
  // Tampilkan juga versi "signed" supaya 340 terbaca sebagai -20 (lebih gampang dipahami saat target=0)
  float y = yaw1;
  if (y > 180.0f) y -= 360.0f;
  lcd.print((int)y);
  lcd.print(" deg    "); // Padding untuk clear sisa karakter

  // Baris 2: RPM Motor 1 dan Motor 2 (ringkas)
  lcd.setCursor(0, 1);
  lcd.print("M1:");
  lcd.print((int)getRPM_Motor1());
  lcd.print(" M2:");
  lcd.print((int)getRPM_Motor2());
  lcd.print("    ");
}

// Odom + arah encoder M3/M4:
// - Baris 1: X dan Y (cm)
// - Baris 2: dM3 dan dM4 (delta count terakhir)
void displayOdomStatus() {
  // Konversi ke cm biar enak dibaca

  lcd.setCursor(0, 0);
  lcd.print(cmdX);
  lcd.print(" ");
  lcd.print(cmdY);
  lcd.print(" ");
  lcd.print(cmdW);
  lcd.print(" | ");
  // lcd.print("X:");
  // lcd.print((int)x_cm);
  // lcd.print(" Y:");
  // lcd.print((int)y_cm);
  // tampilkan yaw juga
  lcd.print("Yaw:");
  float y = yaw1;
  if (y > 180.0f) y -= 360.0f;
  lcd.print((int)y);
  lcd.print("     "); // padding
  // Baris 2: bergantian tampil (d3/d4) dan (waypoint target)
  static unsigned long lastToggleMs = 0;
  static bool showWaypoint = false;
  unsigned long now = millis();
  if (now - lastToggleMs >= 1000) {
    lastToggleMs = now;
    showWaypoint = !showWaypoint;
  }

  lcd.setCursor(0, 1);

  int idx = getActiveWaypointIndex();
  lcd.print(rpmX);
  lcd.print(" ");
  lcd.print(rpmY);
  lcd.print(" ");
  lcd.print(rpmW);
  lcd.print(" | ");
  // modeSpeed
  // lcd.print("Mode:");
  lcd.print(modeSpeed);
  lcd.print(" ");

#if WAYPOINT_UNITS_CM
  // sudah cm
  int tx_cm = (int)(getActiveWaypointX());
  int ty_cm = (int)(getActiveWaypointY());
  lcd.print("X");
  lcd.print(tx_cm);
  lcd.print(" Y");
  lcd.print(ty_cm);
#elif WAYPOINT_UNITS_MM
  // mm -> cm
  int tx_cm = (int)(getActiveWaypointX() / 10.0f);
  int ty_cm = (int)(getActiveWaypointY() / 10.0f);
  lcd.print("X");
  lcd.print(tx_cm);
  lcd.print(" Y");
  lcd.print(ty_cm);
#else
  // meter -> cm
  int tx_cm = (int)(getActiveWaypointX() * 100.0f);
  int ty_cm = (int)(getActiveWaypointY() * 100.0f);
  lcd.print("X");
  lcd.print(tx_cm);
  lcd.print(" Y");
  lcd.print(ty_cm);
#endif
  lcd.print("   ");
}
