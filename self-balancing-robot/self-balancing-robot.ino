# include "Wire.h"
# include "math.h"
# include "MPU6050.h"
# include "I2Cdev.h"

# define RMotor_DIG 4
# define RMotor_PWM 5
# define LMotor_DIG 6
# define LMotor_PWM 7

# define Kp 40
# define Ki 40
# define Kd 0.01
# define sampleTime 0.005
# define targetAngle 0.5

MPU6050 mpu;

int16_t ACC_Y, ACC_Z, GYRO_X;
volatile int motorPower, gyroRate;
volatile float accAngle, gyroAngle, currentAngle, prevAngle = 0;
volatile float error, prevError = 0, errorSum = 0;
volatile byte count = 0;

void setMotor(int LMSpeed, int RMSpeed){
  if(LMSpeed >= 0) {
    analogWrite(LMotor_PWM, LMSpeed);
    digitalWrite(LMotor_DIG, LOW);
  }
  else {
    analogWrite(LMotor_PWM, 255 + LMSpeed);
    digitalWrite(LMotor_DIG, HIGH);
  }
  if(RMSpeed >= 0) {
    analogWrite(RMotor_PWM, RMSpeed);
    digitalWrite(RMotor_DIG, LOW);
  }
  else {
    analogWrite(RMotor_PWM, 255 + RMSpeed);
    digitalWrite(RMotor_DIG, HIGH);
  }  
}

void setTimer(){
  // initial Timer1 for interupt
  cli(); // disable global interupt first
  TCCR1A = 0;
  TCCR1B = 0;
  // set compare match register to set sample time 5ms
  OCR1A = 9999;
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS11 bit for prescaling by 8
  TCCR1B |= (1 << CS11);
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);
  sei(); // enable global interrupts
}

void setup() {
  Serial.begin(9600);
  // set the motor control
  pinMode(LMotor_DIG, OUTPUT);
  pinMode(LMotor_PWM, OUTPUT);
  pinMode(RMotor_DIG, OUTPUT);
  pinMode(RMotor_PWM, OUTPUT);

  // initialize the MPU6050
  mpu.initialize();
  mpu.setYAccelOffset(175);
  mpu.setZAccelOffset(701);
  mpu.setXGyroOffset(97);

  // initialize the timer and PID
  setTimer();
}

void loop() {
  // get the acceleration and gyroscope values
  ACC_Y = mpu.getAccelerationY();
  ACC_Z = mpu.getAccelerationZ();
  GYRO_X = mpu.getRotationX();  

  motorPower = constrain(motorPower, -255, 255);
  setMotor(motorPower, motorPower);
}

ISR(TIMER1_COMPA_vect){
  accAngle = atan2(ACC_Y, ACC_Z)*RAD_TO_DEG;
  gyroRate = map(GYRO_X, -32768, 32767, -250, 250);
  gyroAngle = (float)gyroRate*sampleTime;
  currentAngle = 0.9934*(prevAngle + gyroAngle) + 0.0066*(accAngle);

  error = currentAngle - targetAngle;
  errorSum = errorSum + error;  
  errorSum = constrain(errorSum, -300, 300);
  //calculate output from P, I and D values
  motorPower = Kp*(error) + Ki*(errorSum)*sampleTime - Kd*(currentAngle-prevAngle)/sampleTime;
  prevAngle = currentAngle;
}
