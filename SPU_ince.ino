#include "Handle/motor.h"
#include <QTRSensors.h>
QTRSensors qtr;

const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];

void setup() {
    //Setup Channel A
  pinMode(12, OUTPUT); //Motor A1
  pinMode(3, OUTPUT); //Motor A2
  pinMode(10, OUTPUT); //Speed PWM Motor A
 
  //Setup Channel B
  pinMode(13, OUTPUT);  //Motor B1
  pinMode(8, OUTPUT);  //Motor B2
  pinMode(11, OUTPUT); //Speed PWM Motor B

  pinMode(4,OUTPUT);

  // configure the sensors
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4, A5 ,5 ,3}, SensorCount);
  qtr.setEmitterPin(2);

  delay(500);
  for (uint16_t i = 0; i < 200; i++)
  {
    qtr.calibrate();
  }

  // print the calibration minimum values measured when emitters were on
  Serial.begin(9600);
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(qtr.calibrationOn.minimum[i]);
    Serial.print(' ');
  }
  Serial.println();

  // print the calibration maximum values measured when emitters were on
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(qtr.calibrationOn.maximum[i]);
    Serial.print(' ');
  }

  Serial.println();
  Serial.println();

  delay(1000);
}

void loop() {
    // read calibrated sensor values and obtain a measure of the line position
  // from 0 to 5000 (for a white line, use readLineWhite() instead)
  uint16_t position = qtr.readLineBlack(sensorValues);

  // print the sensor values as numbers from 0 to 1000, where 0 means maximum
  // reflectance and 1000 means minimum reflectance, followed by the line
  // position
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(sensorValues[i]);
    Serial.print('\t');
  }
  Serial.println(position);

  delay(250);
}
