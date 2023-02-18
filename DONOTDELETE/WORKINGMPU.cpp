#include <Arduino.h>
#include <math.h>
#include <I2Cdev.h>
#include "MPU6050_6Axis_MotionApps20.h"
#include <Wire.h>

#define FREQUENCY 1
#define INTERRUPT_PIN 2

MPU6050 mpu6050;
uint8_t fifoBuffer[64]; // FIFO storage buffer
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

volatile bool mpuInterrupt = false;
void dmpDataReady()
{
  mpuInterrupt = true;
}

void writeToSerial(void* address, uint8_t size) {
  uint8_t* bytePtr = (uint8_t*) address;
  for (int8_t i = 0; i < size; i++) {
    Serial.write(*bytePtr); 
    bytePtr++;
  }
}

void initialiseMPU()
{
  mpu6050.initialize();
  mpu6050.dmpInitialize();
  mpu6050.setXGyroOffset(220);
  mpu6050.setYGyroOffset(76);
  mpu6050.setZGyroOffset(-85);
  mpu6050.setZAccelOffset(1788); // 1688 factory default for my test chip
  mpu6050.CalibrateAccel(6);
  mpu6050.CalibrateGyro(6);
  mpu6050.setDMPEnabled(true);
  attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
}

void mpuGetData()
{
  mpu6050.dmpGetCurrentFIFOPacket(fifoBuffer);

  mpu6050.dmpGetQuaternion(&q, fifoBuffer);
  mpu6050.dmpGetAccel(&aa, fifoBuffer);
  mpu6050.dmpGetGravity(&gravity, &q);
  mpu6050.dmpGetLinearAccel(&aaReal, &aa, &gravity);
  mpu6050.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
  mpu6050.dmpGetYawPitchRoll(ypr, &q, &gravity);

  ypr[0] = ypr[0] * 180 / M_PI;
  ypr[1] = ypr[1] * 180 / M_PI;
  ypr[2] = ypr[2] * 180 / M_PI;

  Serial.write((uint8_t) 0x0a);
  Serial.write((uint8_t) 25);
  unsigned long time = millis();
  writeToSerial(&time, 4);
  Serial.write((uint8_t) 0x00);
  writeToSerial(&ypr[0], 4);
  writeToSerial(&ypr[1], 4);
  writeToSerial(&ypr[2], 4);
  writeToSerial(&aaWorld.x, 2);
  writeToSerial(&aaWorld.y, 2);
  writeToSerial(&aaWorld.z, 2);

  mpuInterrupt = false;
}

void setup()
{
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);
  initialiseMPU();
}

void loop()
{
  if (mpuInterrupt)
   mpuGetData();
}