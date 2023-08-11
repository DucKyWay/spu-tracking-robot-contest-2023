#ifndef PID_h
#define PID_h

#include <Arduino.h>
#ifndef motor_h
#define motor_h


const int BZ = 4;

// ตั้งค่าขาที่ใช้ในการควบคุม L298P Shield
const int M1 = 10;  // ขาที่ใช้ในการควบคุมความเร็วมอเตอร์ A
const int MA1 = 12; // ให้มอเตอร์หมุนไปด้านหน้า
const int MA2 = 3;  // ให้มอเตอร์หมุนไปด้านหลัง

const int M2 = 11;  // ขาที่ใช้ในการควบคุมความเร็วมอเตอร์ B
const int MB1 = 13; // ให้มอเตอร์หมุนไปด้านหน้า
const int MB2 = 8;  // ให้มอเตอร์หมุนไปด้านหลัง



int[] sensorsValue = new int[7]; // do not use 0 index

int lineV = 0;
int groundV = 0;
int meanV = 500;
int N = 5;

int motorSpeed;
int baseSpeed = 100;

int rightSpeed,leftSpeed;
int maxSpeed = 100;

int sum_error = 0;

// PID
int error = 0;
int pre_error = 0;
int Kp = 1.0;
int Kd = 0.03;
int Ki = 0.1;

void motor(int PWA, int PWB, int MAA1, int MAA2, int MBB1, int MBB2)
{
  analogWrite(M1, PWA);
  analogWrite(M2, PWB);

  digitalWrite(MA1, MAA1);
  digitalWrite(MB1, MBB1);
  digitalWrite(MA2, MAA2);
  digitalWrite(MB2, MBB2);
}


bool B(int n){  
  if(n > 980){ // is black
      return true;
  }else{
      return false;
  } 
}

bool W(int n){  
    if(n <= 979){ // is white
      return true;
  }else{
      return false;
  } 
}

void Tl(){
    
    for(int i=0;i<N;i++){
        sensorsValue[i] = analog(i);
    }
   
    if( W(sensorsValue[0]) && W(sensorsValue[1]) && W(sensorsValue[2]) && W(sensorsValue[3]) && W(sensorsValue[4]) &&W(sensorsValue[5]) &&W(sensorsValue[6]) &&B(sensorsValue[7]) ){ 
        error = 4;
    }else if( W(sensorsValue[0]) && W(sensorsValue[1]) && W(sensorsValue[2]) && W(sensorsValue[3]) && W(sensorsValue[4]) &&W(sensorsValue[5]) &&B(sensorsValue[6]) &&B(sensorsValue[7]) ){
      error = 4;
    }else if( W(sensorsValue[0]) && W(sensorsValue[1]) && W(sensorsValue[2]) && W(sensorsValue[3]) && W(sensorsValue[4]) &&B(sensorsValue[5]) &&B(sensorsValue[6]) &&B(sensorsValue[7])){
      error = 3;
    }else if( W(sensorsValue[0]) && W(sensorsValue[1]) && W(sensorsValue[2]) && W(sensorsValue[3]) && B(sensorsValue[4]) &&B(sensorsValue[5]) &&B(sensorsValue[6]) &&B(sensorsValue[7])) {
      error = 3;
    }else if( W(sensorsValue[0]) && W(sensorsValue[1]) && W(sensorsValue[2]) && B(sensorsValue[3]) && B(sensorsValue[4]) &&B(sensorsValue[5]) &&B(sensorsValue[6]) &&B(sensorsValue[7])) {
      error = 2;
    }else if( W(sensorsValue[0]) && W(sensorsValue[1]) && B(sensorsValue[2]) && B(sensorsValue[3]) && B(sensorsValue[4]) &&B(sensorsValue[5]) &&B(sensorsValue[6]) &&B(sensorsValue[7])){
      error = 2;
    }else if( W(sensorsValue[0]) && B(sensorsValue[1]) && B(sensorsValue[2]) && B(sensorsValue[3]) && B(sensorsValue[4]) &&B(sensorsValue[5]) &&B(sensorsValue[6]) &&B(sensorsValue[7])){
      error = 1;
    }else if( B(sensorsValue[0]) && B(sensorsValue[1]) && B(sensorsValue[2]) && B(sensorsValue[3]) && B(sensorsValue[4]) &&B(sensorsValue[5]) &&B(sensorsValue[6]) &&B(sensorsValue[7])){
      error = 0;
    }else if( W(sensorsValue[0]) && B(sensorsValue[1]) && B(sensorsValue[2]) && B(sensorsValue[3]) && B(sensorsValue[4]) &&B(sensorsValue[5]) &&B(sensorsValue[6]) &&W(sensorsValue[7])){
      error = 0;
    }else if( W(sensorsValue[0]) && W(sensorsValue[1]) && B(sensorsValue[2]) && B(sensorsValue[3]) && B(sensorsValue[4]) &&B(sensorsValue[5]) &&W(sensorsValue[6]) &&W(sensorsValue[7])){
      error = 0;
    }else if( W(sensorsValue[0]) && W(sensorsValue[1]) && W(sensorsValue[2]) && B(sensorsValue[3]) && B(sensorsValue[4]) &&BWsensorsValue[5]) &&W(sensorsValue[6]) &&W(sensorsValue[7])){
      error = 0;
    }else if( B(sensorsValue[0]) && W(sensorsValue[1]) && W(sensorsValue[2]) && W(sensorsValue[3]) && W(sensorsValue[4]) &&W(sensorsValue[5]) &&W(sensorsValue[6]) &&W(sensorsValue[7]) ){
      error = -4;
    }else if( B(sensorsValue[0]) && B(sensorsValue[1]) && W(sensorsValue[2]) && W(sensorsValue[3]) && W(sensorsValue[4]) &&W(sensorsValue[5]) &&W(sensorsValue[6]) &&W(sensorsValue[7]) ){
      error = -4;
    }else if( B(sensorsValue[0]) && B(sensorsValue[1]) && B(sensorsValue[2]) && W(sensorsValue[3]) && W(sensorsValue[4]) &&W(sensorsValue[5]) &&W(sensorsValue[6]) &&W(sensorsValue[7]) ){
      error = -3;
    }else if( B(sensorsValue[0]) && B(sensorsValue[1]) && B(sensorsValue[2]) && B(sensorsValue[3]) && W(sensorsValue[4]) &&W(sensorsValue[5]) &&W(sensorsValue[6]) &&W(sensorsValue[7]) ){
      error = -3;
    }else if( B(sensorsValue[0]) && B(sensorsValue[1]) && W(sensorsValue[2]) && B(sensorsValue[3]) && B(sensorsValue[4]) &&W(sensorsValue[5]) &&W(sensorsValue[6]) &&W(sensorsValue[7]) ){
      error = -2;
    }else if( B(sensorsValue[0]) && B(sensorsValue[1]) && W(sensorsValue[2]) && B(sensorsValue[3]) && B(sensorsValue[4]) &&B(sensorsValue[5]) &&W(sensorsValue[6]) &&W(sensorsValue[7]) ){
      error = -2;
    }else if( B(sensorsValue[0]) && B(sensorsValue[1]) && W(sensorsValue[2]) && B(sensorsValue[3]) && B(sensorsValue[4]) &&B(sensorsValue[5]) &&B(sensorsValue[6]) &&W(sensorsValue[7]) ){
      error = -1;
    /// check WWWWW
    else if( W(sensorsValue[0]) && W(sensorsValue[1]) && W(sensorsValue[2]) && W(sensorsValue[3]) && W(sensorsValue[4]) ){
        error = pre_error;
    }


   
   motorSpeed = Kp*error + Kd*(error - pre_error) + Ki*(sum_error);
   leftSpeed = baseSpeed + motorSpeed;
   rightSpeed = baseSpeed - motorSpeed;
   
   if(leftSpeed > maxSpeed) leftSpeed = maxSpeed;
   if(rightSpeed > maxSpeed) rightSpeed = maxSpeed;

   if(leftSpeed < -maxSpeed) leftSpeed = -maxSpeed;
   if(rightSpeed < -maxSpeed) rightSpeed = -maxSpeed;
   
   motor(leftSpeed,rightSpeed,1,0,1,0);
  
   
   pre_error = error;
   sum_error += error;
   
   
   lcd("e=%d pe=%d|Ls=%d RS=%d|kp=%d kd=%d", error,pre_error, leftSpeed, rightSpeed ,Kp*error, Kd*(error - pre_error) );

          
}

#endif