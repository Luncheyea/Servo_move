#define PT_USE_TIMER
#define DOG_DEGUB_MOD

#include <pt.h>
#include <Wire.h>
#include <HCPCA9685.h>
#include <CapacitiveSensor.h>

// 0x40 is default address of the PCA9685 Module
HCPCA9685 HCPCA9685(0x40);
CapacitiveSensor cs_4_5 = CapacitiveSensor(4, 5);

Pt footPt[4], actiondPt, receivePt, sitzenUndHandshakePt, testPt;
bool PtEnable[4];
uint16_t sycArc[4] = {0};
int8_t actionIndex = 0;

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
  PT_INIT(&sitzenUndHandshakePt);
  for (uint8_t i = 0; i < 4; i++)
    PT_INIT(&footPt[i]);

  action = stehen;
  stand();
  delay(2000);
}

static void Action_mission(Pt *pt) {
  PT_BEGIN(pt);

  static DogAction _action = stehen;
  PT_WAIT_UNTIL(pt, _action != action);
  _action = action;

  PtEnable[0] = false;
  PtEnable[1] = false;
  PtEnable[2] = false;
  PtEnable[3] = false;
  PT_YIELD(pt);

  if (_action == stehen) {
    sycArc[0] = 0; sycArc[1] = 0; sycArc[2] = 0; sycArc[3] = 0;

    PT_YIELD(pt);
    stand();
  }
  else if (_action == sitzen || _action == haendeSchuetteln) {

    PT_YIELD(pt);
  } else if (_action == geradeaus || _action == links || _action == rechts || _action == zuruek) {
    PtEnable[0] = true;
    PtEnable[3] = true;
    PT_TIMER_DELAY(pt, 550);
    PtEnable[1] = true;
    PtEnable[2] = true;

    PT_YIELD(pt);
  }
  actionIndex = action - 1;

  PT_END(pt);
}

void loop() {
  Action_mission(&actiondPt);
  receiveMessage(&receivePt);
  sitzen_und_handshakeAction(&sitzenUndHandshakePt);
  //
  //test(&testPt);
  //
  foot0_move(&footPt[0]);
  foot1_move(&footPt[1]);
  foot2_move(&footPt[2]);
  foot3_move(&footPt[3]);
}

static void test(Pt *pt) {
  PT_BEGIN(pt);

  action = stehen;
  PT_TIMER_DELAY(pt, 5000);
  action = geradeaus;
  PT_TIMER_DELAY(pt, 4000);
  action = rechts;
  PT_TIMER_DELAY(pt, 9000);
  action = links;
  PT_TIMER_DELAY(pt, 9000);
  //action = sitzen;
  //PT_TIMER_DELAY(pt, 9000);
  //action = stehen;
  //PT_TIMER_DELAY(pt, 5000);

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

  if (receive.indexOf("_Der_Hund_steht_#00") >= 0) {
    action = stehen;
  }
  else if (receive.indexOf("_Der_Hund_geht_nach_geradeaus_#01") >= 0) {
    action = geradeaus;
  }
  else if (receive.indexOf("_Der_Hund_geht_nach_links_#02") >= 0) {
    action = links;
  }
  else if (receive.indexOf("_Der_Hund_geht_nach_rechts_#03") >= 0) {
    action = rechts;
  }
  else if (receive.indexOf("_Der_Hund_geht_nach_zuruek_#04") >= 0) {
    action = zuruek;
  }
  else if (receive.indexOf("_Der_Hund_sitzt_#05") >= 0) {
    action = sitzen;
  }
  else if (receive.indexOf("_Der_Hund_hinlegt_#06") >= 0) {
    action = hinlegen;
  }
  else if (receive.indexOf("_Der_Hund_schuettelt_haende_#07") >= 0) {
    action = haendeSchuetteln;
  }
  PT_YIELD(pt);

  PT_END(pt);
}


void stand() {
  HCPCA9685.Servo(0, 200);
  HCPCA9685.Servo(1,  120);

  HCPCA9685.Servo(2, 205);
  HCPCA9685.Servo(3, 130);

  HCPCA9685.Servo(4, 125);
  HCPCA9685.Servo(5, 220);

  HCPCA9685.Servo(6, 140);
  HCPCA9685.Servo(7, 250);
}

static void sitzen_und_handshakeAction(Pt *pt) {
  PT_BEGIN(pt);

  PT_WAIT_UNTIL(pt, action == haendeSchuetteln || action == sitzen);
  stand();
  PT_TIMER_DELAY(pt, 500);

  static int16_t i, j;
  static const uint16_t sitPos[8] = {280, 180, 30, 370, 70, 150, 350, 30};
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
  PT_YIELD(pt);
  //
label_handshake:
  if (action == haendeSchuetteln) {
    HCPCA9685.Servo(0, 200);
    HCPCA9685.Servo(1, 10);
    PT_TIMER_DELAY(pt, 500);

    PT_WAIT_UNTIL(pt, action != haendeSchuetteln || cs_4_5.capacitiveSensor(30) >= 200);
    PT_TIMER_DELAY(pt, 500);
    PT_WAIT_UNTIL(pt, action != haendeSchuetteln || cs_4_5.capacitiveSensor(30) < 200);

    HCPCA9685.Servo(1, 160);
    for (i = 200; i <= sitPos[0]; i += 5) {
      HCPCA9685.Servo(0, i);
      PT_TIMER_DELAY(pt, 15);
    }

    if (action == haendeSchuetteln || action == sitzen) {
      action = sitzen;
      goto label_handshake;
    }
  } else if (action == sitzen) {
    PT_WAIT_UNTIL(pt, action != sitzen);

    if (action == haendeSchuetteln) {
      goto label_handshake;
    }
  }
  // /////////////// thread lock
  delay(500);
  for (i = 260, j = 90; i >= 30, j <= 320; i -= 5, j += 5) {
    HCPCA9685.Servo(0, i);
    HCPCA9685.Servo(4, j);
    delay(10);
  }
  delay(50);

  for (i = 0; i < 52; i++) {
    HCPCA9685.Servo(2, sitArg[2]);
    sitArg[2] += 5;
    if (sitArg[2] > 225) sitArg[2] = 225;

    HCPCA9685.Servo(3, sitArg[3]);
    sitArg[3] -= 5;
    if (sitArg[3] < 100) sitArg[3] = 100;

    HCPCA9685.Servo(6, sitArg[6]);
    sitArg[6] -= 5;
    if (sitArg[6] < 140) sitArg[6] = 140;

    HCPCA9685.Servo(7, sitArg[7]);
    sitArg[7] += 5;
    if (sitArg[7] > 270) sitArg[7] = 270;

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
    static const uint16_t actionPosition[3][4] = {{140, 210, 195,  60},
      {170, 210, 195,  60},
      {140, 210, 195,  60}
    };

    for (i = actionPosition[actionIndex][0]; i <= actionPosition[actionIndex][1] && PtEnable[0]; i += step) {
      HCPCA9685.Servo(0, i);

      PT_TIMER_DELAY(pt, thighSpeed);
    }

    for (i = actionPosition[actionIndex][2]; i >= actionPosition[actionIndex][3] && PtEnable[0]; i -= step) {
      HCPCA9685.Servo(1, i);

      PT_TIMER_DELAY(pt, crusSpeed);
    }


    for (i = actionPosition[actionIndex][1]; i >= actionPosition[actionIndex][0] && PtEnable[0]; i -= step) {
      HCPCA9685.Servo(0, i);

      PT_TIMER_DELAY(pt, thighSpeed);
    }

    for (i = actionPosition[actionIndex][3]; i <= actionPosition[actionIndex][2] && PtEnable[0]; i += step) {
      HCPCA9685.Servo(1, i);

      PT_TIMER_DELAY(pt, crusSpeed);
    }
    sycArc[0]++;
#ifdef DOG_DEGUB_MOD
    Serial.println(sycArc[0]);
#endif
    PT_WAIT_UNTIL(pt, footSychronized(pt));

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
    static const uint16_t actionPosition[3][4] = {{210, 260, 200,  60},
      {230, 260, 200,  60},
      {210, 260, 200,  60}
    };

    for (i = actionPosition[actionIndex][0]; i <= actionPosition[actionIndex][1] && PtEnable[1]; i += step) {
      HCPCA9685.Servo(2, i);

      PT_TIMER_DELAY(pt, thighSpeed);
    }


    for (i = actionPosition[actionIndex][2]; i >= actionPosition[actionIndex][3] && PtEnable[1]; i -= step) {
      HCPCA9685.Servo(3, i);

      PT_TIMER_DELAY(pt, crusSpeed);
    }
    sycArc[1]++;
#ifdef DOG_DEGUB_MOD
    Serial.println(sycArc[1]);
#endif
    PT_WAIT_UNTIL(pt, footSychronized(pt));

    for (i = actionPosition[actionIndex][1]; i >= actionPosition[actionIndex][0] && PtEnable[1]; i -= step) {
      HCPCA9685.Servo(2, i);

      PT_TIMER_DELAY(pt, thighSpeed);
    }


    for (i = actionPosition[actionIndex][3]; i <= actionPosition[actionIndex][2] && PtEnable[1]; i += step) {
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
    static const uint16_t actionPosition[3][4] = {{190, 130, 160, 300},
      {190, 130, 160, 300},
      {165, 130, 160, 300}
    };

    for (i = actionPosition[actionIndex][0]; i >= actionPosition[actionIndex][1] && PtEnable[2]; i -= step) {
      HCPCA9685.Servo(4, i);

      PT_TIMER_DELAY(pt, thighSpeed);
    }


    for (i = actionPosition[actionIndex][2]; i <= actionPosition[actionIndex][3] && PtEnable[2]; i += step) {
      HCPCA9685.Servo(5, i);

      PT_TIMER_DELAY(pt, crusSpeed);
    }
    sycArc[2]++;
#ifdef DOG_DEGUB_MOD
    Serial.println(sycArc[2]);
#endif
    PT_WAIT_UNTIL(pt, footSychronized(pt));

    for (i = actionPosition[actionIndex][1]; i <= actionPosition[actionIndex][0] && PtEnable[2]; i += step) {
      HCPCA9685.Servo(4, i);

      PT_TIMER_DELAY(pt, thighSpeed);
    }


    for (i = actionPosition[actionIndex][3]; i >= actionPosition[actionIndex][2] && PtEnable[2]; i -= step) {
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
    static const uint16_t actionPosition[3][4] = {{150,  90, 165, 300},
      {150,  90, 165, 300},
      {120,  90, 165, 300}
    };

    for (i = actionPosition[actionIndex][0]; i >= actionPosition[actionIndex][1] && PtEnable[3]; i -= step) {
      HCPCA9685.Servo(6, i);

      PT_TIMER_DELAY(pt, thighSpeed);
    }


    for (i = actionPosition[actionIndex][2]; i <= actionPosition[actionIndex][3] && PtEnable[3]; i += step) {
      HCPCA9685.Servo(7, i);

      PT_TIMER_DELAY(pt, crusSpeed);
    }


    for (i = actionPosition[actionIndex][1]; i <= actionPosition[actionIndex][0] && PtEnable[3]; i += step) {
      HCPCA9685.Servo(6, i);

      PT_TIMER_DELAY(pt, thighSpeed);
    }


    for (i = actionPosition[actionIndex][3]; i >= actionPosition[actionIndex][2] && PtEnable[3]; i -= step) {
      HCPCA9685.Servo(7, i);

      PT_TIMER_DELAY(pt, crusSpeed);
    }
    sycArc[3]++;
#ifdef DOG_DEGUB_MOD
    Serial.println(sycArc[3]);
#endif
    PT_WAIT_UNTIL(pt, footSychronized(pt));

    PT_YIELD(pt);
  }
  PT_END(pt);
}

static bool footSychronized(Pt *pt) {
  static uint8_t index;
  index = (&footPt[0] == pt) ? 0 : (&footPt[1] == pt) ? 1 : (&footPt[2] == pt) ? 2 : 3;

  if (sycArc[0] == sycArc[1] && sycArc[1] == sycArc[2] && sycArc[2] == sycArc[3])
    return true;
  else if (sycArc[index] < sycArc[0] || sycArc[index] < sycArc[1] || sycArc[index] < sycArc[2] || sycArc[index] < sycArc[3])
    return true;
  else
    return false;
}




