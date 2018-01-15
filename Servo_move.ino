#include <HCPCA9685.h> // Include the HCPCA9685 library created by Andrew Davies

#define I2CAdd 0x40 // Default address of the PCA9685 Module
HCPCA9685 HCPCA9685(I2CAdd); // Define Library to use I2C communication

void setup() {
  Serial.begin(9600);

  HCPCA9685.Init(SERVO_MODE); // Set to Servo Mode
  HCPCA9685.Sleep(false); // Wake up PCA9685 module


  stand();
}


void loop() {
  //steps();
  //forward();
  //stand();
  //test();
}

void test() {
  HCPCA9685.Servo(4, 150);
  HCPCA9685.Servo(5, 350);
  delay(200);
  HCPCA9685.Servo(5, 270);
  HCPCA9685.Servo(4, 225);
  delay(200);
  HCPCA9685.Servo(4, 135);
  HCPCA9685.Servo(5, 270);
  delay(200);
}

void forward() {
  HCPCA9685.Servo(0, 120);
  HCPCA9685.Servo(6, 240);
  HCPCA9685.Servo(2, 240);
  HCPCA9685.Servo(4, 120);
  delay(800);

  HCPCA9685.Servo(1, 200);
  HCPCA9685.Servo(7, 160);
  HCPCA9685.Servo(3, 60);
  HCPCA9685.Servo(5, 300);
  delay(300);

  HCPCA9685.Servo(0, 240);
  HCPCA9685.Servo(6, 120);
  HCPCA9685.Servo(2, 120);
  HCPCA9685.Servo(4, 240);
  delay(800);

  HCPCA9685.Servo(1, 60);
  HCPCA9685.Servo(7, 300);
  HCPCA9685.Servo(3, 200);
  HCPCA9685.Servo(5, 160);
  delay(300);
}

void stand() {
  HCPCA9685.Servo(0, 225);
  HCPCA9685.Servo(1, 90);

  HCPCA9685.Servo(2, 225);
  HCPCA9685.Servo(3, 90);

  HCPCA9685.Servo(4, 135);
  HCPCA9685.Servo(5, 270);

  HCPCA9685.Servo(6, 135);
  HCPCA9685.Servo(7, 270);

  delay(100);
}


void steps() {
  HCPCA9685.Servo(2, 225);
  HCPCA9685.Servo(3, 90);
  HCPCA9685.Servo(0, 285);
  HCPCA9685.Servo(1, 10);
  delay(100);

  HCPCA9685.Servo(0, 225);
  HCPCA9685.Servo(1, 90);
  HCPCA9685.Servo(6, 75);
  HCPCA9685.Servo(7, 350);
  delay(100);

  HCPCA9685.Servo(6, 135);
  HCPCA9685.Servo(7, 270);
  HCPCA9685.Servo(4, 75);
  HCPCA9685.Servo(5, 350);
  delay(100);

  HCPCA9685.Servo(4, 135);
  HCPCA9685.Servo(5, 270);
  HCPCA9685.Servo(2, 285);
  HCPCA9685.Servo(3, 10);
  delay(100);
}





