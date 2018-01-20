#include <HCPCA9685.h> // Include the HCPCA9685 library created by Andrew Davies
#include <pt.h>
#include <PT_timer.h>

// 0x40 is default address of the PCA9685 Module
// Define Library to use I2C communication
HCPCA9685 HCPCA9685(0x40);

Pt footPt[4], forwardPt;
PT_timer delayTimer[5];
static bool PtEnable[4], isforward = true;

void setup() {
  Serial.begin(115200);

  HCPCA9685.Init(SERVO_MODE); // Set to Servo Mode
  HCPCA9685.Sleep(false); // Wake up PCA9685 module

  PT_INIT(&forwardPt);
  for (uint8_t i = 0; i < 4; i++)
    PT_INIT(&footPt[i]);

  for (uint8_t i = 0; i < 4; i++)
    ;//PtEnable[i] = true;

  isforward = true;
  stand();
}

static void forward_mission(Pt *pt) {
  PT_BEGIN(pt);

  if (!isforward) {

    PT_WAIT_UNTIL(pt, isforward);
  }


  PtEnable[0] = true;
  delayTimer[4].setTimer(300);
  PT_WAIT_UNTIL(pt, delayTimer[4].Expired());

  PtEnable[3] = true;
  delayTimer[4].setTimer(300);
  PT_WAIT_UNTIL(pt, delayTimer[4].Expired());

  PtEnable[2] = true;
  delayTimer[4].setTimer(300);
  PT_WAIT_UNTIL(pt, delayTimer[4].Expired());

  PtEnable[1] = true;
  delayTimer[4].setTimer(300);
  PT_WAIT_UNTIL(pt, delayTimer[4].Expired());

  isforward = false;

  PT_END(pt);
}

void loop() {
  forward_mission(&forwardPt);

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


/////////////////////////////////////////////////
static void foot0_move(Pt *pt) {
  PT_BEGIN(pt);
  while (1) {
    PT_WAIT_UNTIL(pt, PtEnable[0]);
    static int16_t i;

    for (i = 120; i <= 240; i += 5) {
      HCPCA9685.Servo(0, i);

      delayTimer[0].setTimer(20);
      PT_WAIT_UNTIL(pt, delayTimer[0].Expired());
    }

    for (i = 200; i >= 60; i -= 5) {
      HCPCA9685.Servo(1, i);

      delayTimer[0].setTimer(15);
      PT_WAIT_UNTIL(pt, delayTimer[0].Expired());
    }

    for (i = 240; i >= 120; i -= 5) {
      HCPCA9685.Servo(0, i);

      delayTimer[0].setTimer(20);
      PT_WAIT_UNTIL(pt, delayTimer[0].Expired());
    }

    for (i = 60; i <= 200; i += 5) {
      HCPCA9685.Servo(1, i);

      delayTimer[0].setTimer(15);
      PT_WAIT_UNTIL(pt, delayTimer[0].Expired());
    }

  }
  PT_END(pt);
}

static void foot1_move(Pt *pt) {
  PT_BEGIN(pt);
  while (1) {
    PT_WAIT_UNTIL(pt, PtEnable[1]);
    static int16_t i;

    for (i = 120; i <= 240; i += 5) {
      HCPCA9685.Servo(2, i);

      delayTimer[1].setTimer(20);
      PT_WAIT_UNTIL(pt, delayTimer[1].Expired());
    }

    for (i = 200; i >= 60; i -= 5) {
      HCPCA9685.Servo(3, i);

      delayTimer[1].setTimer(15);
      PT_WAIT_UNTIL(pt, delayTimer[1].Expired());
    }

    for (i = 240; i >= 120; i -= 5) {
      HCPCA9685.Servo(2, i);

      delayTimer[1].setTimer(20);
      PT_WAIT_UNTIL(pt, delayTimer[1].Expired());
    }

    for (i = 60; i <= 200; i += 5) {
      HCPCA9685.Servo(3, i);

      delayTimer[1].setTimer(15);
      PT_WAIT_UNTIL(pt, delayTimer[1].Expired());
    }

  }
  PT_END(pt);
}

static void foot2_move(Pt *pt) {
  PT_BEGIN(pt);
  while (1) {
    PT_WAIT_UNTIL(pt, PtEnable[2]);
    static int16_t i;

    for (i = 240; i >= 120; i -= 5) {
      HCPCA9685.Servo(4, i);

      delayTimer[2].setTimer(20);
      PT_WAIT_UNTIL(pt, delayTimer[2].Expired());
    }

    for (i = 160; i <= 300; i += 5) {
      HCPCA9685.Servo(5, i);

      delayTimer[2].setTimer(15);
      PT_WAIT_UNTIL(pt, delayTimer[2].Expired());
    }

    for (i = 120; i <= 240; i += 5) {
      HCPCA9685.Servo(4, i);

      delayTimer[2].setTimer(20);
      PT_WAIT_UNTIL(pt, delayTimer[2].Expired());
    }

    for (i = 300; i >= 160; i -= 5) {
      HCPCA9685.Servo(5, i);

      delayTimer[2].setTimer(15);
      PT_WAIT_UNTIL(pt, delayTimer[2].Expired());
    }

  }
  PT_END(pt);
}

static void foot3_move(Pt *pt) {
  PT_BEGIN(pt);
  while (1) {
    PT_WAIT_UNTIL(pt, PtEnable[3]);
    static int16_t i;

    for (i = 240; i >= 120; i -= 5) {
      HCPCA9685.Servo(6, i);

      delayTimer[3].setTimer(20);
      PT_WAIT_UNTIL(pt, delayTimer[3].Expired());
    }

    for (i = 160; i <= 300; i += 5) {
      HCPCA9685.Servo(7, i);

      delayTimer[3].setTimer(15);
      PT_WAIT_UNTIL(pt, delayTimer[3].Expired());
    }

    for (i = 120; i <= 240; i += 5) {
      HCPCA9685.Servo(6, i);

      delayTimer[3].setTimer(20);
      PT_WAIT_UNTIL(pt, delayTimer[3].Expired());
    }

    for (i = 300; i >= 160; i -= 5) {
      HCPCA9685.Servo(7, i);

      delayTimer[3].setTimer(15);
      PT_WAIT_UNTIL(pt, delayTimer[3].Expired());
    }

  }
  PT_END(pt);
}





