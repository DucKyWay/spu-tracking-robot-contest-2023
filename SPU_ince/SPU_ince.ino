#include "Handle/motor.h"

#include <Arduino.h>
#include <QTRSensors.h>
#include <EEPROM.h>

QTRSensors qtr;

/////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////

const uint8_t SensorCount = 8;

uint16_t sensorValues[SensorCount];

const int BZ1 = 4;

float Error = 0;
float LastError = 0;
float Integral = 0;
float Derivative = 0;

// ตั้งค่าขาที่ใช้ในการควบคุม L298P Shield
const int M1_1 = 10;   // ขาที่ใช้ในการควบคุมความเร็วมอเตอร์ A
const int MA1_1 = 12;  // ให้มอเตอร์หมุนไปด้านหน้า
const int MA2_2 = 3;   // ให้มอเตอร์หมุนไปด้านหลัง

const int M2_2 = 11;   // ขาที่ใช้ในการควบคุมความเร็วมอเตอร์ B
const int MB1_1 = 13;  // ให้มอเตอร์หมุนไปด้านหน้า
const int MB2_2 = 8;   // ให้มอเตอร์หมุนไปด้านหลัง



int lineV = 0;
int groundV = 0;
int meanV = 500;
int N = 5;

int motorSpeed;
int baseSpeed = 50;

int rightSpeed, leftSpeed;
int maxSpeed = 100;

int sum_error = 0;

// PID
int error = 0;
int pre_error = 0;
int Kp = 1.0;
int Kd = 0.01;
int Ki = 0.5;

// Desired position should be the center of your sensors.
const uint16_t DesiredPosition = SensorCount * 1000 / 2;  // assuming each sensor can have a max value of 1000

/////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////

// void motor2(int PWA1, int PWB1, int MAA1_1, int MAA2_2, int MBB2_1, int MBB2_2) {
//   analogWrite(M1_1, PWA1);
//   analogWrite(M2_2, PWB1);

//   digitalWrite(MA1_1, MAA1_1);
//   digitalWrite(MB1_1, MBB2_1);
//   digitalWrite(MA2_2, MAA2_2);
//   digitalWrite(MB2_2, MBB2_2);
// }
bool B(int n) {
  if (n > 900) {  // is black
    return true;
  } else {
    return false;
  }
}

/////////////////////////////////////////////////////////////////////////

bool W(int n) {
  if (n <= 899) {  // is white
    return true;
  } else {
    return false;
  }
}
/////////////////////////////////////////////////////////////////////////



/////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////

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
  delay(10);

   if (W(sensorValues[0]) && W(sensorValues[1]) && W(sensorValues[2]) && W(sensorValues[3]) && W(sensorValues[4]) && W(sensorValues[5]) && W(sensorValues[6]) && B(sensorValues[7])) {
    error = 4;
  } else if (W(sensorValues[0]) && W(sensorValues[1]) && W(sensorValues[2]) && W(sensorValues[3]) && W(sensorValues[4]) && W(sensorValues[5]) && B(sensorValues[6]) && B(sensorValues[7])) {
    error = 4;
  } else if (W(sensorValues[0]) && W(sensorValues[1]) && W(sensorValues[2]) && W(sensorValues[3]) && W(sensorValues[4]) && B(sensorValues[5]) && B(sensorValues[6]) && B(sensorValues[7])) {
    error = 3;
  } else if (W(sensorValues[0]) && W(sensorValues[1]) && W(sensorValues[2]) && W(sensorValues[3]) && B(sensorValues[4]) && B(sensorValues[5]) && B(sensorValues[6]) && B(sensorValues[7])) {
    error = 3;
  } else if (W(sensorValues[0]) && W(sensorValues[1]) && W(sensorValues[2]) && B(sensorValues[3]) && B(sensorValues[4]) && B(sensorValues[5]) && B(sensorValues[6]) && B(sensorValues[7])) {
    error = 2;
  } else if (W(sensorValues[0]) && W(sensorValues[1]) && B(sensorValues[2]) && B(sensorValues[3]) && B(sensorValues[4]) && B(sensorValues[5]) && B(sensorValues[6]) && B(sensorValues[7])) {
    error = 2;
  } else if (W(sensorValues[0]) && B(sensorValues[1]) && B(sensorValues[2]) && B(sensorValues[3]) && B(sensorValues[4]) && B(sensorValues[5]) && B(sensorValues[6]) && B(sensorValues[7])) {
    error = 1;
  } else if (B(sensorValues[0]) && B(sensorValues[1]) && B(sensorValues[2]) && B(sensorValues[3]) && B(sensorValues[4]) && B(sensorValues[5]) && B(sensorValues[6]) && B(sensorValues[7])) {
    error = 0;
  } else if (W(sensorValues[0]) && B(sensorValues[1]) && B(sensorValues[2]) && B(sensorValues[3]) && B(sensorValues[4]) && B(sensorValues[5]) && B(sensorValues[6]) && W(sensorValues[7])) {
    error = 0;
  } else if (W(sensorValues[0]) && W(sensorValues[1]) && B(sensorValues[2]) && B(sensorValues[3]) && B(sensorValues[4]) && B(sensorValues[5]) && W(sensorValues[6]) && W(sensorValues[7])) {
    error = 0;
  } else if (W(sensorValues[0]) && W(sensorValues[1]) && W(sensorValues[2]) && B(sensorValues[3]) && B(sensorValues[4]) && B(sensorValues[5]) && W(sensorValues[6]) && W(sensorValues[7])) {
    error = 0;
  } else if (B(sensorValues[0]) && W(sensorValues[1]) && W(sensorValues[2]) && W(sensorValues[3]) && W(sensorValues[4]) && W(sensorValues[5]) && W(sensorValues[6]) && W(sensorValues[7])) {
    error = -4;
  } else if (B(sensorValues[0]) && B(sensorValues[1]) && W(sensorValues[2]) && W(sensorValues[3]) && W(sensorValues[4]) && W(sensorValues[5]) && W(sensorValues[6]) && W(sensorValues[7])) {
    error = -4;
  } else if (B(sensorValues[0]) && B(sensorValues[1]) && B(sensorValues[2]) && W(sensorValues[3]) && W(sensorValues[4]) && W(sensorValues[5]) && W(sensorValues[6]) && W(sensorValues[7])) {
    error = -3;
  } else if (B(sensorValues[0]) && B(sensorValues[1]) && B(sensorValues[2]) && B(sensorValues[3]) && W(sensorValues[4]) && W(sensorValues[5]) && W(sensorValues[6]) && W(sensorValues[7])) {
    error = -3;
  } else if (B(sensorValues[0]) && B(sensorValues[1]) && W(sensorValues[2]) && B(sensorValues[3]) && B(sensorValues[4]) && W(sensorValues[5]) && W(sensorValues[6]) && W(sensorValues[7])) {
    error = -2;
  } else if (B(sensorValues[0]) && B(sensorValues[1]) && W(sensorValues[2]) && B(sensorValues[3]) && B(sensorValues[4]) && B(sensorValues[5]) && W(sensorValues[6]) && W(sensorValues[7])) {
    error = -2;
  } else if (B(sensorValues[0]) && B(sensorValues[1]) && W(sensorValues[2]) && B(sensorValues[3]) && B(sensorValues[4]) && B(sensorValues[5]) && B(sensorValues[6]) && W(sensorValues[7])) {
    error = -1;
    /// check WWWWW
  } else if (W(sensorValues[0]) && W(sensorValues[1]) && W(sensorValues[2]) && W(sensorValues[3]) && W(sensorValues[4]) && W(sensorValues[5]) && W(sensorValues[6])&& W(sensorValues[7])) {
      error = pre_error;
    }

    error += 3500 - position;

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

    // motor(rightSpeed, leftSpeed, 1, LOW, 1, LOW);
    // delay(30);

    motor(90, 90, 1, LOW, 1, LOW);
    delay(2000);
    motor(90, 10, 1, LOW, 1, LOW);
    delay(2000);
    motor(90, 60, 1, LOW, 1, LOW);
    delay(2000);

    Serial.print(leftSpeed);
    Serial.print("  ");
    Serial.println(rightSpeed);
    pre_error = error;
    sum_error += error;
}

/////////////////////////////////////////////////////////////////////////

void callibrated() {
  // 2.5 ms RC หมดเวลาอ่าน (ค่าเริ่มต้น) * 10 อ่านต่อ calibrate() โทร
  // = ~25 ms / calibrate() 1 ครั้ง
  // เรียก calibrate() 400 ครั้ง ใช้เวลาประมาณ 10 วินาที
  for (uint16_t i = 0; i < 400; i++) {
    qtr.calibrate();
  }
  digitalWrite(LED_BUILTIN, LOW);  // turn off Arduino's LED to indicate we are through with calibration

  // พิมพ์ค่าต่ำสุดของที่วัดได้เมื่อเปิด calibrate
  Serial.begin(9600);
  for (uint8_t i = 0; i < SensorCount; i++) {
    Serial.print(qtr.calibrationOn.minimum[i]);
    Serial.print(' ');
  }
  Serial.println();

  // พิมพ์ค่าสูงสุดของที่วัดได้เมื่อเปิด calibrate
  for (uint8_t i = 0; i < SensorCount; i++) {
    Serial.print(qtr.calibrationOn.maximum[i]);
    Serial.print(' ');
  }
  Serial.println();
  Serial.println();

  delay(1000);
}

/////////////////////////////////////////////////////////////////////////

void calibrate(int i) {

  Serial.println("Working in calibrate().");
  Serial.begin(9600);
  if(0 <= i <= 100) {
    Serial.println("Calibrate White");
  }
  else if(101 <= i <= 200) { 
    Serial.println("Calibrate Black");
  }

  delay(1000);
}

/////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////

void setup() {
  pinMode(7, OUTPUT);
  digitalWrite(7, HIGH);
  //Setup Channel A

  pinMode(9, INPUT);

  pinMode(4, OUTPUT);
  // configure the sensors
  qtr.setTypeRC();
  qtr.setSensorPins(
    (const uint8_t[]){
      5, 3, A5, A4, A3, A2, A1, A0 },
    SensorCount);
  qtr.setEmitterPin(2);
  Serial.println("Start Calibrate");
  Beep();

  delay(500);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);  // turn on Arduino's LED to indicate we are in calibration mode

  // if (!loadCalibrationFromEEPROM()) {
  //   Serial.println("work!");
  //   callibrated();  // Call the calibration function if loading fails
  // }
  callibrated();
  Serial.println("Stop Calibrated");
  Beep();
}

bool ch = false;

/////////////////////////////////////////////////////////////////////////

void loop() {
  // if (digitalRead9t) == 1) {
  //   ch = true;

  //   for (int i = 0 ; i <= 100 ; i++ ) {
  //     calibrate(i); //calibrate white
  //   }
  //   for (int i = 101 ; i <= 200 ; i++ ) {
  //     calibrate(i); //calibrate black line
  //   }
  // }

  if (ch == false) {
    sensor_test();
  } else {
    uint16_t position = qtr.readLineBlack(sensorValues);  //อ่านค่าเส้นสีดำให้เป็นเส้นที่ถูกตรวจสอบเป็นหลัก
    Error = 3500 - position;  // 3500 is the center position for an 8 sensor array
    Integral += Error;
    Derivative = Error - LastError;

    // Adjust motor speed based on PID Value

    LastError = Error;

    // ตรวจสอบว่าถึงเวลาตรวจสอบยัง
  }
}