#define PT_USE_TIMER

#include "HCPCA9685.h" // Include the HCPCA9685 library created by Andrew Davies
#include <pt.h>

// 0x40 is default address of the PCA9685 Module
// Define Library to use I2C communication
HCPCA9685 HCPCA9685(0x40);

Pt footPt[4], actiondPt, receivePt, testPt;
static bool PtEnable[4];

enum DogAction {
  stehen = 0,

  geradeaus = 1,
  links = 2,
  rechts = 3,
  zuruek = 4,

  sitzen = 5,
  hinlegen = 6,
  haendeSchuetteln = 7
};

DogAction action;

void setup() {
  Serial.begin(115200);

  HCPCA9685.Init(SERVO_MODE); // Set to Servo Mode
  HCPCA9685.Sleep(false); // Wake up PCA9685 module

  //
  PT_INIT(&testPt);
  //
  PT_INIT(&actiondPt);
  PT_INIT(&receivePt);
  for (uint8_t i = 0; i < 4; i++)
    PT_INIT(&footPt[i]);

  action = stehen;//geradeaus;//sitzen;
  stand();
  delay(2000);
}

static void Action_mission(Pt *pt) {
  PT_BEGIN(pt);

  static DogAction _action = stehen;
  if (_action == action)
    return;
  PT_YIELD(pt);
  _action = action;

  if (_action == stehen) {
    PtEnable[0] = false;
    PtEnable[1] = false;
    PtEnable[2] = false;
    PtEnable[3] = false;

    PT_YIELD(pt);
    stand();
  }
  else if (_action == sitzen) {
    PtEnable[0] = false;
    PtEnable[1] = false;
    PtEnable[2] = false;
    PtEnable[3] = false;

    PT_YIELD(pt);
    //sitdown();
  } else {
    PtEnable[0] = true;
    PtEnable[3] = true;
    PT_TIMER_DELAY(pt, 550); //950
    PtEnable[1] = true;
    PtEnable[2] = true;

    PT_YIELD(pt);
  }

  PT_END(pt);
}

void loop() {
  Action_mission(&actiondPt);
  receiveMessage(&receivePt);
  //
  test(&testPt);
  //
  foot0_move(&footPt[0]);
  foot1_move(&footPt[1]);
  foot2_move(&footPt[2]);
  foot3_move(&footPt[3]);
}

static void test(Pt *pt) {
  PT_BEGIN(pt);

  action = stehen;
  PT_TIMER_DELAY(pt, 7000);
  action = geradeaus;
  PT_TIMER_DELAY(pt, 7000);
  action = sitzen;
  PT_TIMER_DELAY(pt, 7000);
  action = stehen;
  PT_TIMER_DELAY(pt, 7000);

  PT_YIELD(pt);
  PT_END(pt);
}

static void receiveMessage(Pt *pt) {
  PT_BEGIN(pt);

  PT_WAIT_UNTIL(pt, Serial.available());
  static String receive;

  Serial.flush();
  receive = "";
  while (Serial.available()) {
    receive += (char)Serial.read();
    PT_TIMER_DELAY(pt, 2);
  }
  PT_YIELD(pt);

  if (receive.indexOf("_Der_Hund_geradeaus_#01") >= 0) {
    action = geradeaus;
  }
  else if (receive.indexOf("_Der_Hund_stehen_#00") >= 0) {
    action = stehen;
  }
  else if (receive.indexOf("_Der_Hund_sitzen_#05") >= 0) {
    action = sitzen;
  }

  PT_YIELD(pt);

  PT_END(pt);
}


void stand() {
  HCPCA9685.Servo(0, 200);
  HCPCA9685.Servo(1, 90);

  HCPCA9685.Servo(2, 225);
  HCPCA9685.Servo(3, 100);

  HCPCA9685.Servo(4, 135);
  HCPCA9685.Servo(5, 270);

  HCPCA9685.Servo(6, 140);
  HCPCA9685.Servo(7, 270);
}

void sitdown() {
  HCPCA9685.Servo(0, 260);
  HCPCA9685.Servo(1, 160);

  HCPCA9685.Servo(2, 30);
  HCPCA9685.Servo(3, 360);

  HCPCA9685.Servo(4, 80);
  HCPCA9685.Servo(5, 165);

  HCPCA9685.Servo(6, 340);
  HCPCA9685.Servo(7, 20);
}



/////////////////////////////////////////////////
uint8_t aaa = 15, bbb = 10;

static void foot0_move(Pt *pt) {
  PT_BEGIN(pt);
  while (1) {
    PT_WAIT_UNTIL(pt, PtEnable[0]);
    static int16_t i;
    static uint8_t posSwingSpeed = 15, negSwingSpeed = 10;

    for (i = 150; i <= 210 && PtEnable[0]; i += 5) {
      HCPCA9685.Servo(0, i);

      PT_TIMER_DELAY(pt, aaa);
    }

    for (i = 200; i >= 60 && PtEnable[0]; i -= 5) {
      HCPCA9685.Servo(1, i);

      PT_TIMER_DELAY(pt, bbb);
    }

    for (i = 210; i >= 150 && PtEnable[0]; i -= 5) {
      HCPCA9685.Servo(0, i);

      PT_TIMER_DELAY(pt, aaa);
    }

    for (i = 60; i <= 200 && PtEnable[0]; i += 5) {
      HCPCA9685.Servo(1, i);

      PT_TIMER_DELAY(pt, bbb);
    }

    PT_YIELD(pt);
  }
  PT_END(pt);
}

static void foot1_move(Pt *pt) {
  PT_BEGIN(pt);
  while (1) {
    PT_WAIT_UNTIL(pt, PtEnable[1]);
    static int16_t i;
	static uint8_t posSwingSpeed = 15, negSwingSpeed = 10;

    for (i = 210; i <= 270 && PtEnable[1]; i += 5) {
      HCPCA9685.Servo(2, i);

      PT_TIMER_DELAY(pt, aaa);
    }

    for (i = 200; i >= 60 && PtEnable[1]; i -= 5) {
      HCPCA9685.Servo(3, i);

      PT_TIMER_DELAY(pt, bbb);
    }

    for (i = 270; i >= 210 && PtEnable[1]; i -= 5) {
      HCPCA9685.Servo(2, i);

      PT_TIMER_DELAY(pt, aaa);
    }

    for (i = 60; i <= 200 && PtEnable[1]; i += 5) {
      HCPCA9685.Servo(3, i);

      PT_TIMER_DELAY(pt, bbb);
    }

    PT_YIELD(pt);
  }
  PT_END(pt);
}

static void foot2_move(Pt *pt) {
  PT_BEGIN(pt);
  while (1) {
    PT_WAIT_UNTIL(pt, PtEnable[2]);
    static int16_t i;
	static uint8_t posSwingSpeed = 15, negSwingSpeed = 10;

    for (i = 210; i >= 150 && PtEnable[2]; i -= 5) {
      HCPCA9685.Servo(4, i);

      PT_TIMER_DELAY(pt, aaa);
    }

    for (i = 160; i <= 300 && PtEnable[2]; i += 5) {
      HCPCA9685.Servo(5, i);

      PT_TIMER_DELAY(pt, bbb);
    }

    for (i = 150; i <= 210 && PtEnable[2]; i += 5) {
      HCPCA9685.Servo(4, i);

      PT_TIMER_DELAY(pt, aaa);
    }

    for (i = 300; i >= 160 && PtEnable[2]; i -= 5) {
      HCPCA9685.Servo(5, i);

      PT_TIMER_DELAY(pt, bbb);
    }

    PT_YIELD(pt);
  }
  PT_END(pt);
}

static void foot3_move(Pt *pt) {
  PT_BEGIN(pt);
  while (1) {
    PT_WAIT_UNTIL(pt, PtEnable[3]);
    static int16_t i;
	static uint8_t posSwingSpeed = 15, negSwingSpeed = 10;

    for (i = 150; i >= 90 && PtEnable[3]; i -= 5) {
      HCPCA9685.Servo(6, i);

      PT_TIMER_DELAY(pt, aaa);
    }

    for (i = 160; i <= 300 && PtEnable[3]; i += 5) {
      HCPCA9685.Servo(7, i);

      PT_TIMER_DELAY(pt, bbb);
    }

    for (i = 90; i <= 150 && PtEnable[3]; i += 5) {
      HCPCA9685.Servo(6, i);

      PT_TIMER_DELAY(pt, aaa);
    }

    for (i = 300; i >= 160 && PtEnable[3]; i -= 5) {
      HCPCA9685.Servo(7, i);

      PT_TIMER_DELAY(pt, bbb);
    }

    PT_YIELD(pt);
  }
  PT_END(pt);
}





