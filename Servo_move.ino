#define PT_USE_TIMER

#include "HCPCA9685.h" // Include the HCPCA9685 library created by Andrew Davies
#include <pt.h>

// 0x40 is default address of the PCA9685 Module
// Define Library to use I2C communication
HCPCA9685 HCPCA9685(0x40);

Pt footPt[4], actiondPt, receivePt, sitzenPt, testPt;
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
  PT_INIT(&sitzenPt);
  for (uint8_t i = 0; i < 4; i++)
    PT_INIT(&footPt[i]);

  action = stehen;//geradeaus;//sitzen;
  stand();
  delay(2000);
}

static void Action_mission(Pt *pt) {
  PT_BEGIN(pt);

  static DogAction _action = stehen;
  PT_WAIT_UNTIL(pt, _action != action);
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
  sitdownAction(&sitzenPt);
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
  PT_TIMER_DELAY(pt, 1000);
  action = geradeaus;
  PT_TIMER_DELAY(pt, 2000);
  action = sitzen;
  PT_TIMER_DELAY(pt, 3000);
  action = stehen;
  PT_TIMER_DELAY(pt, 5000);

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

static void sitdownAction(Pt *pt) {
  PT_BEGIN(pt);

  PT_WAIT_UNTIL(pt, action == sitzen);
  stand();
  PT_YIELD(pt);

  static int16_t i, j;
  static const uint16_t sitPos[8] = {260, 180, 30, 360, 80, 165, 340, 20};
  static uint16_t sitArg[8];
  sitArg[0] = 200; sitArg[1] = 90;  sitArg[2] = 225; sitArg[3] = 100;
  sitArg[4] = 135; sitArg[5] = 270; sitArg[6] = 140; sitArg[7] = 270;

  for (i = 0; i < 52; i++) {
    HCPCA9685.Servo(0, sitArg[0]);
    sitArg[0] += 2;
    if (sitArg[0] > sitPos[0]) sitArg[0] = sitPos[0];
    PT_YIELD(pt);

    HCPCA9685.Servo(1, sitArg[1]);
    sitArg[1] += 5;
    if (sitArg[1] > sitPos[1]) sitArg[1] = sitPos[1];
    PT_YIELD(pt);

    HCPCA9685.Servo(2, sitArg[2]);
    sitArg[2] -= 10;
    if (sitArg[2] < sitPos[2]) sitArg[2] = sitPos[2];
    PT_YIELD(pt);

    HCPCA9685.Servo(3, sitArg[3]);
    sitArg[3] += 13;
    if (sitArg[3] > sitPos[3]) sitArg[3] = sitPos[3];
    PT_YIELD(pt);

    HCPCA9685.Servo(4, sitArg[4]);
    sitArg[4] -= 2;
    if (sitArg[4] < sitPos[4]) sitArg[4] = sitPos[4];
    PT_YIELD(pt);

    HCPCA9685.Servo(5, sitArg[5]);
    sitArg[5] -= 5;
    if (sitArg[5] < sitPos[5]) sitArg[5] = sitPos[5];
    PT_YIELD(pt);

    HCPCA9685.Servo(6, sitArg[6]);
    sitArg[6] += 10;
    if (sitArg[6] > sitPos[6]) sitArg[6] = sitPos[6];
    PT_YIELD(pt);

    HCPCA9685.Servo(7, sitArg[7]);
    sitArg[7] -= 13;
    if (sitArg[7] < sitPos[7]) sitArg[7] = sitPos[7];
    PT_YIELD(pt);

    PT_TIMER_DELAY(pt, 15);
  }
  //
  PT_WAIT_UNTIL(pt, action != sitzen);

  // 260, 90
  for (i = 260, j = 90; i >= 90, j <= 260; i -= 5, j += 5) {
    HCPCA9685.Servo(0, i);
    HCPCA9685.Servo(4, j);
    delay(10);
  }
  delay(500);

  stand();
  PT_YIELD(pt);

  PT_END(pt);
}


/////////////////////////////////////////////////
static void foot0_move(Pt *pt) {
  PT_BEGIN(pt);
  while (1) {
    PT_WAIT_UNTIL(pt, PtEnable[0]);
    static int16_t i;
    static const uint8_t thighSpeed = 15, crusSpeed = 10, step = 5;
    static const uint16_t actionPosition[4] = {150, 210, 200, 60};

    for (i = actionPosition[0]; i <= actionPosition[1] && PtEnable[0]; i += step) {
      HCPCA9685.Servo(0, i);

      PT_TIMER_DELAY(pt, thighSpeed);
    }

    for (i = actionPosition[2]; i >= actionPosition[3] && PtEnable[0]; i -= step) {
      HCPCA9685.Servo(1, i);

      PT_TIMER_DELAY(pt, crusSpeed);
    }

    for (i = actionPosition[1]; i >= actionPosition[0] && PtEnable[0]; i -= step) {
      HCPCA9685.Servo(0, i);

      PT_TIMER_DELAY(pt, thighSpeed);
    }

    for (i = actionPosition[3]; i <= actionPosition[2] && PtEnable[0]; i += step) {
      HCPCA9685.Servo(1, i);

      PT_TIMER_DELAY(pt, crusSpeed);
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
    static const uint8_t thighSpeed = 15, crusSpeed = 10, step = 5;
    static const uint16_t actionPosition[4] = {210, 270, 200, 60};

    for (i = actionPosition[0]; i <= actionPosition[1] && PtEnable[1]; i += step) {
      HCPCA9685.Servo(2, i);

      PT_TIMER_DELAY(pt, thighSpeed);
    }

    for (i = actionPosition[2]; i >= actionPosition[3] && PtEnable[1]; i -= step) {
      HCPCA9685.Servo(3, i);

      PT_TIMER_DELAY(pt, crusSpeed);
    }

    for (i = actionPosition[1]; i >= actionPosition[0] && PtEnable[1]; i -= step) {
      HCPCA9685.Servo(2, i);

      PT_TIMER_DELAY(pt, thighSpeed);
    }

    for (i = actionPosition[3]; i <= actionPosition[2] && PtEnable[1]; i += step) {
      HCPCA9685.Servo(3, i);

      PT_TIMER_DELAY(pt, crusSpeed);
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
    static const uint8_t thighSpeed = 15, crusSpeed = 10, step = 5;
    static const uint16_t actionPosition[4] = {210, 150, 160, 300};

    for (i = actionPosition[0]; i >= actionPosition[1] && PtEnable[2]; i -= step) {
      HCPCA9685.Servo(4, i);

      PT_TIMER_DELAY(pt, thighSpeed);
    }

    for (i = actionPosition[2]; i <= actionPosition[3] && PtEnable[2]; i += step) {
      HCPCA9685.Servo(5, i);

      PT_TIMER_DELAY(pt, crusSpeed);
    }

    for (i = actionPosition[1]; i <= actionPosition[0] && PtEnable[2]; i += step) {
      HCPCA9685.Servo(4, i);

      PT_TIMER_DELAY(pt, thighSpeed);
    }

    for (i = actionPosition[3]; i >= actionPosition[2] && PtEnable[2]; i -= step) {
      HCPCA9685.Servo(5, i);

      PT_TIMER_DELAY(pt, crusSpeed);
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
    static const uint8_t thighSpeed = 15, crusSpeed = 10, step = 5;
    static const uint16_t actionPosition[4] = {150, 90, 160, 300};

    for (i = actionPosition[0]; i >= actionPosition[1] && PtEnable[3]; i -= step) {
      HCPCA9685.Servo(6, i);

      PT_TIMER_DELAY(pt, thighSpeed);
    }

    for (i = actionPosition[2]; i <= actionPosition[3] && PtEnable[3]; i += step) {
      HCPCA9685.Servo(7, i);

      PT_TIMER_DELAY(pt, crusSpeed);
    }

    for (i = actionPosition[1]; i <= actionPosition[0] && PtEnable[3]; i += step) {
      HCPCA9685.Servo(6, i);

      PT_TIMER_DELAY(pt, thighSpeed);
    }

    for (i = actionPosition[3]; i >= actionPosition[2] && PtEnable[3]; i -= step) {
      HCPCA9685.Servo(7, i);

      PT_TIMER_DELAY(pt, crusSpeed);
    }

    PT_YIELD(pt);
  }
  PT_END(pt);
}





