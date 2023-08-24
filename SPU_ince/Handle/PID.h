#include <Arduino.h>
#include <QTRSensors.h>

#ifndef PID_h
#define PID_h

#include "motor.h"

QTRSensors qtr;

const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];

const int BZ1 = 4;

// ตั้งค่าขาที่ใช้ในการควบคุม L298P Shield
const int M1_1 = 10;  // ขาที่ใช้ในการควบคุมความเร็วมอเตอร์ A
const int MA1_1 = 12; // ให้มอเตอร์หมุนไปด้านหน้า
const int MA2_2 = 3;  // ให้มอเตอร์หมุนไปด้านหลัง

const int M2_2 = 11;  // ขาที่ใช้ในการควบคุมความเร็วมอเตอร์ B
const int MB1_1 = 13; // ให้มอเตอร์หมุนไปด้านหน้า
const int MB2_2 = 8;  // ให้มอเตอร์หมุนไปด้านหลัง

int* sensorsValue = new int[7]; // do not use 0 index

int lineV = 0;
int groundV = 0;
int meanV = 500;
int N = 5;

int motorSpeed;
int baseSpeed = 100;

int rightSpeed, leftSpeed;
int maxSpeed = 100;

int sum_error = 0;

// PID
int error = 0;
int pre_error = 0;
int Kp = 1.0;
int Kd = 0.03;
int Ki = 0.1;

void motor2(int PWA1, int PWB1, int MAA1_1, int MAA2_2, int MBB2_1, int MBB2_2) {
  analogWrite(M1_1, PWA1);
  analogWrite(M2_2, PWB1);

  digitalWrite(MA1_1, MAA1_1);
  digitalWrite(MB1_1, MBB2_1);
  digitalWrite(MA2_2, MAA2_2);
  digitalWrite(MB2_2, MBB2_2);
}

bool B(int n) {
  if (n > 980) { // is black
    return true;
  }
  else {
    return false;
  }
}

bool W(int n) {
  if (n <= 979) { // is white
    return true;
  }
  else {
    return false;
  }
}

void Tl() {

  for (int i = 0; i < N; i++) {
    sensorsValue[i] = analogRead(i);
  }

  if (W(sensorsValue[0]) && W(sensorsValue[1]) && W(sensorsValue[2]) && W(sensorsValue[3]) && W(sensorsValue[4]) && W(sensorsValue[5]) && W(sensorsValue[6]) && B(sensorsValue[7])) {
    error = 4;
  }
  else if (W(sensorsValue[0]) && W(sensorsValue[1]) && W(sensorsValue[2]) && W(sensorsValue[3]) && W(sensorsValue[4]) && W(sensorsValue[5]) && B(sensorsValue[6]) && B(sensorsValue[7])) {
    error = 4;
  }
  else if (W(sensorsValue[0]) && W(sensorsValue[1]) && W(sensorsValue[2]) && W(sensorsValue[3]) && W(sensorsValue[4]) && B(sensorsValue[5]) && B(sensorsValue[6]) && B(sensorsValue[7])) {
    error = 3;
  }
  else if (W(sensorsValue[0]) && W(sensorsValue[1]) && W(sensorsValue[2]) && W(sensorsValue[3]) && B(sensorsValue[4]) && B(sensorsValue[5]) && B(sensorsValue[6]) && B(sensorsValue[7])) {
    error = 3;
  }
  else if (W(sensorsValue[0]) && W(sensorsValue[1]) && W(sensorsValue[2]) && B(sensorsValue[3]) && B(sensorsValue[4]) && B(sensorsValue[5]) && B(sensorsValue[6]) && B(sensorsValue[7])) {
    error = 2;
  }
  else if (W(sensorsValue[0]) && W(sensorsValue[1]) && B(sensorsValue[2]) && B(sensorsValue[3]) && B(sensorsValue[4]) && B(sensorsValue[5]) && B(sensorsValue[6]) && B(sensorsValue[7])) {
    error = 2;
  }
  else if (W(sensorsValue[0]) && B(sensorsValue[1]) && B(sensorsValue[2]) && B(sensorsValue[3]) && B(sensorsValue[4]) && B(sensorsValue[5]) && B(sensorsValue[6]) && B(sensorsValue[7])) {
    error = 1;
  }
  else if (B(sensorsValue[0]) && B(sensorsValue[1]) && B(sensorsValue[2]) && B(sensorsValue[3]) && B(sensorsValue[4]) && B(sensorsValue[5]) && B(sensorsValue[6]) && B(sensorsValue[7])) {
    error = 0;
  }
  else if (W(sensorsValue[0]) && B(sensorsValue[1]) && B(sensorsValue[2]) && B(sensorsValue[3]) && B(sensorsValue[4]) && B(sensorsValue[5]) && B(sensorsValue[6]) && W(sensorsValue[7])) {
    error = 0;
  }
  else if (W(sensorsValue[0]) && W(sensorsValue[1]) && B(sensorsValue[2]) && B(sensorsValue[3]) && B(sensorsValue[4]) && B(sensorsValue[5]) && W(sensorsValue[6]) && W(sensorsValue[7])) {
    error = 0;
  }
  else if (W(sensorsValue[0]) && W(sensorsValue[1]) && W(sensorsValue[2]) && B(sensorsValue[3]) && B(sensorsValue[4]) && B(sensorsValue[5]) && W(sensorsValue[6]) && W(sensorsValue[7])) {
      error = 0;
  }
  else if (B(sensorsValue[0]) && W(sensorsValue[1]) && W(sensorsValue[2]) && W(sensorsValue[3]) && W(sensorsValue[4]) && W(sensorsValue[5]) && W(sensorsValue[6]) && W(sensorsValue[7])) {
    error = -4;
  }
  else if (B(sensorsValue[0]) && B(sensorsValue[1]) && W(sensorsValue[2]) && W(sensorsValue[3]) && W(sensorsValue[4]) && W(sensorsValue[5]) && W(sensorsValue[6]) && W(sensorsValue[7])) {
    error = -4;
  }
  else if (B(sensorsValue[0]) && B(sensorsValue[1]) && B(sensorsValue[2]) && W(sensorsValue[3]) && W(sensorsValue[4]) && W(sensorsValue[5]) && W(sensorsValue[6]) && W(sensorsValue[7])) {
    error = -3;
  }
  else if (B(sensorsValue[0]) && B(sensorsValue[1]) && B(sensorsValue[2]) && B(sensorsValue[3]) && W(sensorsValue[4]) && W(sensorsValue[5]) && W(sensorsValue[6]) && W(sensorsValue[7])) {
    error = -3;
  }
  else if (B(sensorsValue[0]) && B(sensorsValue[1]) && W(sensorsValue[2]) && B(sensorsValue[3]) && B(sensorsValue[4]) && W(sensorsValue[5]) && W(sensorsValue[6]) && W(sensorsValue[7])) {
    error = -2;
  }
  else if (B(sensorsValue[0]) && B(sensorsValue[1]) && W(sensorsValue[2]) && B(sensorsValue[3]) && B(sensorsValue[4]) && B(sensorsValue[5]) && W(sensorsValue[6]) && W(sensorsValue[7])) {
    error = -2;
  }
  else if (B(sensorsValue[0]) && B(sensorsValue[1]) && W(sensorsValue[2]) && B(sensorsValue[3]) && B(sensorsValue[4]) && B(sensorsValue[5]) && B(sensorsValue[6]) && W(sensorsValue[7])) {
    error = -1;
    /// check WWWWW
    if (W(sensorsValue[0]) && W(sensorsValue[1]) && W(sensorsValue[2]) && W(sensorsValue[3]) && W(sensorsValue[4])) {
      error = pre_error;
    }

    motorSpeed = Kp * error + Kd * (error - pre_error) + Ki * (sum_error);
    leftSpeed = baseSpeed + motorSpeed;
    rightSpeed = baseSpeed - motorSpeed;

    if (leftSpeed > maxSpeed) {
      leftSpeed = maxSpeed;
    }
    if (rightSpeed > maxSpeed) { 
      rightSpeed = maxSpeed;
    }
    if (leftSpeed < -maxSpeed) {
      leftSpeed = -maxSpeed;
    }
    if (rightSpeed < -maxSpeed) {
      rightSpeed = -maxSpeed;
    }
      
    motor2(leftSpeed, rightSpeed, 1, 0, 1, 0);

    pre_error = error;
    sum_error += error;
  }
}

void sensor_test() {
  // อ่านค่าเซ็นเซอร์ที่ปรับเทียบแล้วและรับการวัดตำแหน่งเส้น
  // ตั้งแต่ 0 ถึง 5,000 (สำหรับเส้นสีขาว ให้ใช้ readLineWhite() แทน)
  uint16_t position = qtr.readLineBlack(sensorValues);

  // พิมพ์ค่าเซ็นเซอร์เป็นตัวเลขตั้งแต่ 0 - 1,000 โดยที่ 0 หมายถึงสูงสุด
  // การสะท้อนแสง และ 1,000 หมายถึงการสะท้อนแสงขั้นต่ำ ตามด้วยเส้น
  // ตำแหน่ง
  for (uint8_t i = 0; i < SensorCount; i++) {
    Serial.print(sensorValues[i]);
    Serial.print('\t');
  }
  Serial.println(position);

  delay(250);
}

void callibrated() {
  // 2.5 ms RC หมดเวลาอ่าน (ค่าเริ่มต้น) * 10 อ่านต่อ calibrate() โทร
  // = ~25 ms ต่อการโทร calibrate()
  // โทร calibrate() 400 ครั้งเพื่อให้การสอบเทียบใช้เวลาประมาณ 10 วินาที
  for (uint16_t i = 0; i < 400; i++) {
    qtr.calibrate();
  }
  digitalWrite(LED_BUILTIN, LOW);  // turn off Arduino's LED to indicate we are through with calibration

  // พิมพ์ค่าต่ำสุดของการสอบเทียบที่วัดได้เมื่อเปิดอิมิตเตอร์
  Serial.begin(9600);
  for (uint8_t i = 0; i < SensorCount; i++) {
    Serial.print(qtr.calibrationOn.minimum[i]);
    Serial.print(' ');
  }
  Serial.println();

  // พิมพ์ค่าสูงสุดของการสอบเทียบที่วัดได้เมื่อเปิดอิมิตเตอร์
  for (uint8_t i = 0; i < SensorCount; i++) {
    Serial.print(qtr.calibrationOn.maximum[i]);
    Serial.print(' ');
  }
  Serial.println();
  Serial.println();

  delay(1000);
}

#endif