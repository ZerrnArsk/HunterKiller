#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm;

#define Servo_phi 7
#define Servo_theta 8
#define ServoMIN 100
#define ServoMAX 500

// 서보모터를 원하는 각도로 회전시키는 함수
void rotateServo(uint8_t servoNum, uint16_t angle) {
  uint16_t pwmValue = map(angle, 0, 180, ServoMIN, ServoMAX);  // PWM 값 계산
  pwm.setPWM(servoNum, 0, pwmValue);  // PCA9685 드라이버를 통해 PWM 값 설정
}

void setup() {
  Serial.begin(9600);
  pwm.begin();
  pwm.setPWMFreq(50);  // PCA9685의 PWM 주파수 설정 (보통 50Hz)
  
  rotateServo(Servo_phi, 90);
  rotateServo(Servo_theta, 0);
}

void loop() {
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    int commaIndex = command.indexOf(',');
    if (commaIndex != -1) {
      String value1 = command.substring(0, commaIndex);
      String value2 = command.substring(commaIndex + 1);
      value1.trim();  // 좌우 공백 제거
      value2.trim();  // 좌우 공백 제거
      int servo1Angle = value1.toInt();
      int servo2Angle = value2.toInt();
      
      if (servo1Angle < 0) servo1Angle = 0;
      else if (servo1Angle > 180) servo1Angle = 180;
      
      if (servo2Angle < 0) servo2Angle = 0;
      else if (servo2Angle > 180) servo2Angle = 180;
      
      rotateServo(Servo_phi, servo1Angle);
      rotateServo(Servo_theta, servo2Angle);
    }
  }
}