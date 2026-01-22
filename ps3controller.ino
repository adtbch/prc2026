#include <Arduino.h>
#include <Ps3Controller.h>  

void onConnect() {
  psconect = true;
  Ps3.setPlayer(1);
}

void onDisConnect() {
  psconect = false;
  xL = 0;
  yL = 0;
  xR = 0;
  yR = 0;
  X = 0;
  S = 0;
  T = 0;
  C = 0;
  up = 0;
  dw = 0;
  lf = 0;
  rg = 0;
  L1 = 0;
  L2 = 0;
  L3 = 0;
  R1 = 0;
  R2 = 0;
  R3 = 0;
  SELECT = 0;
  START = 0;
}
void notify() {
  xR = Ps3.data.analog.stick.rx;   // Right stick - x axis
  yR = -Ps3.data.analog.stick.ry;  // Right stick - y axis
  xL = Ps3.data.analog.stick.lx;   // Left stick - x axis
  yL = -Ps3.data.analog.stick.ly;  // Left stick - y axis

  X = Ps3.data.button.cross;
  S = Ps3.data.button.square;
  T = Ps3.data.button.triangle;
  C = Ps3.data.button.circle;

  up = Ps3.data.button.up;
  dw = Ps3.data.button.down;
  lf = Ps3.data.button.left;
  rg = Ps3.data.button.right;

  L1 = Ps3.data.button.l1;
  L2 = Ps3.data.button.l2;
  L3 = Ps3.data.button.l3;
  R1 = Ps3.data.button.r1;
  R2 = Ps3.data.button.r2;
  R3 = Ps3.data.button.r3;

  SELECT = Ps3.data.button.select;
  START = Ps3.data.button.start;
}

void setupPS3() {
  Ps3.attach(notify);
  Ps3.attachOnConnect(onConnect);
  Ps3.attachOnDisconnect(onDisConnect);
  Ps3.begin();
}
