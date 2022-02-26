#include <algorithm>

const float kd = 1, ki = 1, kp = 1;

struct MPUData { float wx, wy, wz, ax, ay, az; };

struct MPU {
  MPUData old_data, data;

  MPUData read();
};

MPU mpu;
MPUData e;

void setPins();
void calculate();
void run();

void setup() {
  // put your setup code here, to run once:

  setPins();
  mpu.old_data = mpu.read();
}

void loop() {
  // put your main code here, to run repeatedly:

  mpu.data = mpu.read();
  calculate();
  run();
  mpu.old_data = mpu.data;
}

MPUData MPU::read() {

}

void calculate() {
  float p, i, d;
  p = mpu.data.ax;
  // i = ...
  d = mpu.data.ax - mpu.old_data.ax;

  return kp * p + ki * i + kd * d;
}

void run() {

}

void setPins() {

}
