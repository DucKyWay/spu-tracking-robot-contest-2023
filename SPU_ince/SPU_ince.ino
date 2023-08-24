#include "Handle/PID.h"
#include "Handle/motor.h"
#include <QTRSensors.h>
#include <EEPROM.h>
QTRSensors qtr;

/////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////

const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];


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

  pinMode(9, INPUT);

  pinMode(4, OUTPUT);
  // configure the sensors
  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){
                      5,3,A5,A4,A3,A2,A1,A0},
                                            
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

    // Adjust motor speed based on PID Value
    test_hello();
    sensor_test();

    LastError = Error;

    // ตรวจสอบว่าถึงเวลาตรวจสอบยัง
    
  }
}
