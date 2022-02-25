#include <algorithm>

const float kd = 1, ki = 1, kp = 1;

struct MPUData { float wx, wy, wz, ax, ay, az; };

struct MPU {
  MPUData data, new_data;

  MPUData read();
};

MPU mpu;

void setup() {
  // put your setup code here, to run once:

  setPins();
  mpu.data = mpu.read();
}

void loop() {
  // put your main code here, to run repeatedly:

  mpu.new_data = mpu.read();
  calculate();
  run();
  mpu.data = mpu.new_data;
}

MPUData MPU::read() {

}

void calculate() {
  
}

void run() {

}

void setPins() {

}
