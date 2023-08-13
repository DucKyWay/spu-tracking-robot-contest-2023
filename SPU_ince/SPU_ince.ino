#include "Handle/motor.h"
#include "Handle/PID.h"
#include <QTRSensors.h>
#include <EEPROM.h>
QTRSensors qtr;

#define BaseSpeed 255

/////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////

const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];

// PID coefficients
const float Kp = 5;   // Proportional coefficient
const float Ki = 1;  // Integral coefficient
const float Kd = 3;     // Derivative coefficient

float Error = 0;
float LastError = 0;
float Integral = 0;
float Derivative = 0;

// Desired position should be the center of your sensors.
const uint16_t DesiredPosition = SensorCount * 1000 / 2;  // assuming each sensor can have a max value of 1000

/////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////

void setup() {
  pinMode(7, OUTPUT);
  digitalWrite(7, HIGH);
  //Setup Channel A
  pinMode(12, OUTPUT);  //Motor A1
  pinMode(3, OUTPUT);   //Motor A2
  pinMode(10, OUTPUT);  //Speed PWM Motor A

  //Setup Channel B
  pinMode(13, OUTPUT);  //Motor B1
  pinMode(8, OUTPUT);   //Motor B2
  pinMode(11, OUTPUT);  //Speed PWM Motor B

  pinMode(9, INPUT);

  pinMode(4, OUTPUT);
  // configure the sensors
  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){
                      A0, A1, A2, A3, A4, A5, 5, 3 },
                    SensorCount);
  qtr.setEmitterPin(2);
  Beep();

  delay(500);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);  // turn on Arduino's LED to indicate we are in calibration mode

  if (!loadCalibrationFromEEPROM()) {
    Serial.println("work!");
    callibrated();  // Call the calibration function if loading fails
  }
  callibrated();
  Beep();
}

bool ch = false;

/////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////

void loop() {
  if (digitalRead(9) == 1) {
    ch = true;
  }

  if (ch == false) {
    sensor_test();
  } else {
    uint16_t position = qtr.readLineBlack(sensorValues);  //อ่านค่าเส้นสีดำให้เป็นเส้นที่ถูกตรวจสอบเป็นหลัก

    Error = 3500 - position;  // 3500 is the center position for an 8 sensor array
    Integral += Error;
    Derivative = Error - LastError;

    float PID_Value = (Kp * Error) + (Ki * Integral) + (Kd * Derivative);  //คำนวนค่า PID ออกมา

    // Adjust motor speed based on PID Value
    motor(BaseSpeed - PID_Value, BaseSpeed + PID_Value, 1, 0, 1, 0);  //จัดความเร็วให้กับมอเตอร์

    LastError = Error;

    // ตรวจสอบว่าถึงเวลาตรวจสอบยัง
    if (millis() - lastCheckTime >= checkInterval) {  // สามารถรันโค้ดอันอื่นไปได้โดยไม่ยึดกับโค้ดนี้
      if (digitalRead(9) == HIGH) {
        ch = false;
      }
      lastCheckTime = millis();  // อัพเดทเวลาที่ตรวจสอบล่าสุด
    }
  }
}

/////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////

void setMotorSpeed(int left, int right) {
  // Limit speed values between 0 and 255
  left = constrain(left, 0, 255);
  right = constrain(right, 0, 255);

  motor(left, right, 1, 0, 1, 0);
}

/////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////

void saveCalibrationToEEPROM() {
  for (uint8_t i = 0; i < SensorCount; i++) {
    EEPROM.put(i * sizeof(uint16_t), qtr.calibrationOn.minimum[i]);
    EEPROM.put((SensorCount + i) * sizeof(uint16_t), qtr.calibrationOn.maximum[i]);
    Serial.println(i * sizeof(uint16_t), qtr.calibrationOn.minimum[i]);
    Serial.println((SensorCount + i) * sizeof(uint16_t), qtr.calibrationOn.maximum[i]);
  }
}

bool loadCalibrationFromEEPROM() {
  for (uint8_t i = 0; i < SensorCount; i++) {
    EEPROM.get(i * sizeof(uint16_t), qtr.calibrationOn.minimum[i]);
    EEPROM.get((SensorCount + i) * sizeof(uint16_t), qtr.calibrationOn.maximum[i]);
    Serial.println(i * sizeof(uint16_t), qtr.calibrationOn.minimum[i]);
    Serial.println((SensorCount + i) * sizeof(uint16_t), qtr.calibrationOn.maximum[i]);

    // ถ้าเกิดว่าในตัว รอมใน อาดูโน้ว ไม่มีข้อมูลค่าที่ คอลลิเบรต เอาไว้ให้หยุด แล้วไป คอลลิเบรตใหม่
    if ((EEPROM.get(i * sizeof(uint16_t), qtr.calibrationOn.minimum[i]) == NULL) || (EEPROM.get((SensorCount + i) * sizeof(uint16_t), qtr.calibrationOn.maximum[i])) == NULL) {
      return 0;
      break;
    }
  }
  return true;
}

/////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////

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

  saveCalibrationToEEPROM();

  delay(1000);
}

/////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////

void sensor_test() {
  // อ่านค่าเซ็นเซอร์ที่ปรับเทียบแล้วและรับการวัดตำแหน่งเส้น
  // ตั้งแต่ 0 ถึง 5,000 (สำหรับเส้นสีขาว ให้ใช้ readLineWhite() แทน)
  uint16_t position = qtr.readLineBlack(sensorValues);

  // พิมพ์ค่าเซ็นเซอร์เป็นตัวเลขตั้งแต่ 0 ถึง 1,000 โดยที่ 0 หมายถึงสูงสุด
  // การสะท้อนแสง และ 1,000 หมายถึงการสะท้อนแสงขั้นต่ำ ตามด้วยเส้น
  // ตำแหน่ง
  for (uint8_t i = 0; i < SensorCount; i++) {
    Serial.print(sensorValues[i]);
    Serial.print('\t');
  }
  Serial.println(position);

  delay(250);
}
