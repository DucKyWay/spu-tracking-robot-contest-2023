#ifndef motor_h
#define motor_h

#include <Arduino.h> // นำเข้าไลบรารีสำหรับ Arduino

const int BZ = 4;

// ตั้งค่าขาที่ใช้ในการควบคุม L298P Shield
const int M1 = 10;  // ขาที่ใช้ในการควบคุมความเร็วมอเตอร์ A
const int MA1 = 12; // ให้มอเตอร์หมุนไปด้านหน้า
const int MA2 = 3;  // ให้มอเตอร์หมุนไปด้านหลัง

const int M2 = 11;  // ขาที่ใช้ในการควบคุมความเร็วมอเตอร์ B
const int MB1 = 13; // ให้มอเตอร์หมุนไปด้านหน้า
const int MB2 = 8;  // ให้มอเตอร์หมุนไปด้านหลัง
const int VMX = 255;
int value;
int CH;

void Beep()
{
  tone(4,100,100);
  delay(100);
}

void motor(int PWA, int PWB, int MAA1, int MAA2, int MBB1, int MBB2)
{
  analogWrite(M1, PWA);
  analogWrite(M2, PWB);

  digitalWrite(MA1, MAA1);
  digitalWrite(MB1, MBB1);
  digitalWrite(MA2, MAA2);
  digitalWrite(MB2, MBB2);
}

void test_motor()
{
  for (value = 0; value <= VMX; value += 5)
  {
    motor(value, value, 1, 0, 1, 0);
    delay(30);
  }
  delay(3000);
  for (value = VMX; value >= 0; value -= 5)
  {
    motor(value, value, 1, 0, 1, 0);
    delay(30);
  }
  delay(500);
  for (value = 0; value <= VMX; value += 5)
  {
    motor(value, value, 0, 1, 0, 1);
    delay(30);
  }
  delay(3000);
  for (value = VMX; value >= 0; value -= 5)
  {
    motor(value, value, 0, 1, 0, 1);
    delay(30);
  }
  delay(500);
}

void stop_motor()
{
  motor(0, 0, 0, 0, 0, 0);
  delay(30);
}

/////////// Driving Zone //////////////

void forward(int power)
{
  if (CH == -1)
  {
    for (value = value; value >= 0; value -= 20)
    {
      motor(value, value, 0, 1, 0, 1);
      delay(30);
    }
    delay(100);
  }
  int power2 = power;
  for (value = value; value <= power2; value += 10)
  {
    motor(value, value, 1, 0, 1, 0);
    delay(30);
  }
  CH = 1;
}

void backward(int power)
{
  if (CH == 1)
  {

    for (value = value; value >= 0; value -= 20)
    {
      motor(value, value, 1, 0, 1, 0);
      delay(30);
    }
  }
  int power2 = power;
  for (value = value; value <= power2; value += 10)
  {
    motor(value, value, 0, 1, 0, 1);
    delay(30);
  }
  CH = -1;
}

void turnleft(int power) {
  for (value = value; value <= power; value += 10)
  {
    motor(value, LOW, 1, 0, 1, 0);
    delay(30);
  }
}

void turnright(int power) {
  for (value = value; value <= power; value += 10)
  {
    motor(LOW, value, 1, 0, 1, 0);
    delay(30);
  }
}

////// Break Motor ////////////

void break_motor()
{
  motor(LOW, LOW, 0, 0, 0, 0);
}

#endif
