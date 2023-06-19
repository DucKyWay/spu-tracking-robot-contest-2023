#include <QTRSensors.h>
#include <PID_v1.h>

#define motor_left 9
#define motor_right 10

QTRSensors qtr;
PID pid(&qtr, &Motor);

double Kp = 0.2;
double Ki = 0.0;
double Kd = 0.1;

int motor_speed = 150;

void setup() {
  
  const uint8_t pins[] = {2, 3, 4, 5, 6, 7}; //QTR Pin

  // ตั้งค่าเซ็นเซอร์ QTR
  qtr.setTypeRC();
  qtr.setSensorPins(pins, sizeof(pins) / sizeof(pins[0]));
  qtr.setEmitterPin(13); // (ถ้าใช้)

  // ตั้งค่าแป้นค่าไหล QTR (ค่าที่ได้จะอยู่ระหว่าง 0 ถึง 1000)
  qtr.setEmitterValue(200);

  pinMode(motor_left, OUTPUT);
  pinMode(motor_right, OUTPUT);

  // ตั้งค่าค่าคงที่ PID
  pid.SetTunings(Kp, Ki, Kd);
  pid.SetOutputLimits(-255, 255); // สูงสุดและต่ำสุดของ Output
  pid.SetMode(AUTOMATIC);         // ตั้งโหมดเป็น AUTOMATIC
}

void loop() {
    
  qtr.read();

  int pid_output = pid.Compute();

  // ปรับความเร็วมอเตอร์ตามผลลัพธ์ PID
  if (pid_output > 0)
  {
    // เคลื่อนที่ไปทางขวา
    digitalWrite(motor_left, motor_speed);
    digitalWrite(motor_right, motor_speed - pid_output);
  }
  else
  {
    // เคลื่อนที่ไปทางซ้าย
    digitalWrite(motor_left, motor_speed + pid_output);
    digitalWrite(motor_right, motor_speed);
  }
}
