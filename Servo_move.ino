#include <HCPCA9685.h> // Include the HCPCA9685 library created by Andrew Davies
#include <pt.h>
#include <PT_timer.h>

// 0x40 is default address of the PCA9685 Module
// Define Library to use I2C communication
HCPCA9685 HCPCA9685(0x40);

Pt footPt[4];
PT_timer delayTimer[4];
bool PtEnable[4];

void setup() {
  HCPCA9685.Init(SERVO_MODE); // Set to Servo Mode
  HCPCA9685.Sleep(false); // Wake up PCA9685 module

  for (uint8_t i = 0; i < 4; i++)
    PT_INIT(&footPt[i]);
  PtEnable[1] = false;

  stand();
}

/////////////////////////////////////////////////
static void foot0_move(Pt *pt) {
  PT_BEGIN(pt);
  while (1) {
    PT_WAIT_UNTIL(pt, PtEnable[0]);

  }
  PT_END(pt);
}

static void foot1_move(Pt *pt) {
  PT_BEGIN(pt);
  while (1) {
    PT_WAIT_UNTIL(pt, PtEnable[1]);


    HCPCA9685.Servo(2, 240);
    delayTimer[3].setTimer(800);
    PT_WAIT_UNTIL(pt, delayTimer[3].Expired());

    HCPCA9685.Servo(3, 60);
    delayTimer[3].setTimer(300);
    PT_WAIT_UNTIL(pt, delayTimer[3].Expired());

    HCPCA9685.Servo(2, 120);
    delayTimer[3].setTimer(800);
    PT_WAIT_UNTIL(pt, delayTimer[3].Expired());

    HCPCA9685.Servo(3, 200);
    delayTimer[3].setTimer(300);
    PT_WAIT_UNTIL(pt, delayTimer[3].Expired());
  }
  PT_END(pt);
}

static void foot2_move(Pt *pt) {
  PT_BEGIN(pt);
  while (1) {
    PT_WAIT_UNTIL(pt, PtEnable[2]);

  }
  PT_END(pt);
}

static void foot3_move(Pt *pt) {
  PT_BEGIN(pt);
  while (1) {
    PT_WAIT_UNTIL(pt, PtEnable[3]);

  }
  PT_END(pt);
}

//////////////////////////////////////////
void loop() {
  foot0_move(&footPt[0]);
  foot1_move(&footPt[1]);
  foot2_move(&footPt[2]);
  foot3_move(&footPt[3]);

  //steps();
  //forward();
  //stand();
  //test();
}

/*
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

  void forward() { --------------------------------
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
  }*/

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



