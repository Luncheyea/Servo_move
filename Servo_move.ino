#define PT_USE_TIMER
//#define DOG_DEGUB_MOD

#include <pt.h>
#include <Wire.h>
#include <MPU9250.h>
#include <HCPCA9685.h>
#include <CapacitiveSensor.h>

// 0x40 is default address of the PCA9685 Module
HCPCA9685 HCPCA9685(0x40);
// an MPU9250 object with the MPU-9250 sensor on I2C bus 0 with address 0x68
MPU9250 IMU(Wire, 0x68);
CapacitiveSensor cs_4_5 = CapacitiveSensor(4, 5);

Pt footPt[4], actiondPt, statePt, receivePt, directionPt, sitzenUndHandshakePt, liePt, watchdogPt, testPt;
bool PtEnable[4];
const uint8_t RedLedPin = 2, GreenLedPin = 3, YellowLedPin = 6;
uint16_t sycArc[4] = { 0 };
uint32_t watchingTime = 0;
int8_t actionIndex = 0;
int16_t links_slant = 30, rechts_slant = 30;
int16_t backfoot_feedback = 40, forkfoot_feedback = 20;

enum DogAction {
  stehen = 0,

  geradeaus = 1,
  links = 2,
  rechts = 3,
  zuruek = 4,

  sitzen = 5,
  hinlegen = 6,
  haendeSchuetteln = 7,
  schlafen = 8,

  // state
  kennenlernen = 15,
  allesGute = 16,
  sprechen = 17
} action, state;

enum OffsetDirection {
  left = -1,
  middle = 0,
  right = 1
} offset_direction = middle;

void setup() {
  Serial.begin(115200);

  HCPCA9685.Init(SERVO_MODE); // Set to Servo Mode
  HCPCA9685.Sleep(false); // Wake up PCA9685 module

  IMU.begin();
  IMU.setAccelRange(MPU9250::ACCEL_RANGE_2G);
  IMU.setGyroRange(MPU9250::GYRO_RANGE_250DPS);
  IMU.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_184HZ);
  IMU.setSrd(19); // setting SRD to 19 for a 50 Hz update ratU.setSrd(19);

  //
  PT_INIT(&testPt);
  //
  PT_INIT(&actiondPt);
  PT_INIT(&statePt);
  PT_INIT(&receivePt);
  PT_INIT(&directionPt);
  PT_INIT(&sitzenUndHandshakePt);
  PT_INIT(&liePt);
  PT_INIT(&watchdogPt);
  for (uint8_t i = 0; i < 4; i++)
    PT_INIT(&footPt[i]);
  //
  pinMode(RedLedPin, OUTPUT);
  pinMode(GreenLedPin, OUTPUT);
  //
  state = allesGute;
  action = stehen;
  stand();
  delay(2000);
  watchingTime = millis();
}

static void Action_mission(Pt *pt) {
  PT_BEGIN(pt);

  static DogAction _action = stehen;

  for (;;) {
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
    else if (_action == sitzen || _action == haendeSchuetteln || _action == hinlegen ) {

      PT_YIELD(pt);
    } else if (_action == geradeaus || _action == links || _action == rechts || _action == zuruek) {
      actionIndex = action - 1;
      links_slant = 30;
      rechts_slant = 30;

      PtEnable[0] = true;
      PtEnable[3] = true;
      PT_TIMER_DELAY(pt, 550);
      PtEnable[1] = true;
      PtEnable[2] = true;

      PT_YIELD(pt);
    }
    watchingTime = millis();
  }

  PT_END(pt);
}

static void State_mission(Pt *pt) {
  PT_BEGIN(pt);

  static DogAction _state = stehen;

  for (;;) {
    PT_WAIT_UNTIL(pt, _state != state);
    _state = state;

    if (_state == allesGute) {
      digitalWrite(GreenLedPin, HIGH);
      digitalWrite(RedLedPin, LOW);
      digitalWrite(YellowLedPin, LOW);
    }
    else if (_state == kennenlernen) {
      digitalWrite(GreenLedPin, LOW);
      digitalWrite(RedLedPin, HIGH);
      digitalWrite(YellowLedPin, LOW);
    }
    else if (_state == sprechen) {
      digitalWrite(GreenLedPin, LOW);
      digitalWrite(RedLedPin, LOW);
      digitalWrite(YellowLedPin, HIGH);
    }

    PT_YIELD(pt);
    watchingTime = millis();
  }

  PT_END(pt);
}

void loop() {
  Action_mission(&actiondPt);
  State_mission(&statePt);
  receiveMessage(&receivePt);
  sensingDirection(&directionPt);
  sitzen_und_handshakeAction(&sitzenUndHandshakePt);
  lieAction(&liePt);
  //
  //test(&testPt);
  //
  foot0_move(&footPt[0]);
  foot1_move(&footPt[1]);
  foot2_move(&footPt[2]);
  foot3_move(&footPt[3]);
  //
  watchDog(&watchdogPt);
}

static void test(Pt *pt) {
  PT_BEGIN(pt);

  for (;;) {
    action = stehen;
    PT_TIMER_DELAY(pt, 5000);
    action = geradeaus;
    PT_TIMER_DELAY(pt, 9000);
    //action = rechts;
    PT_TIMER_DELAY(pt, 9000);
    //action = links;
    PT_TIMER_DELAY(pt, 9000);
    //action = sitzen;
    //PT_TIMER_DELAY(pt, 9000);
    //action = stehen;
    //PT_TIMER_DELAY(pt, 5000);

    PT_YIELD(pt);
  }
  PT_END(pt);
}

static void receiveMessage(Pt *pt) {
  PT_BEGIN(pt);

  static String receive = "";

  for (;;) {
    PT_WAIT_UNTIL(pt, Serial.available());

    Serial.flush();
    receive = "";
    while (Serial.available()) {
      receive += (char)Serial.read();
      PT_TIMER_DELAY(pt, 2);
    }
    PT_YIELD(pt);

    if (receive.indexOf("_Der_Hund_steht_#00") >= 0) {
      action = stehen;
      watchingTime = millis();
    }
    else if (receive.indexOf("_Der_Hund_geht_nach_geradeaus_#01") >= 0) {
      action = geradeaus;
      watchingTime = millis();
    }
    else if (receive.indexOf("_Der_Hund_geht_nach_links_#02") >= 0) {
      action = links;
      watchingTime = millis();
    }
    else if (receive.indexOf("_Der_Hund_geht_nach_rechts_#03") >= 0) {
      action = rechts;
      watchingTime = millis();
    }
    else if (receive.indexOf("_Der_Hund_geht_nach_zuruek_#04") >= 0) {
      action = zuruek;
      watchingTime = millis();
    }
    else if (receive.indexOf("_Der_Hund_sitzt_#05") >= 0) {
      action = sitzen;
      watchingTime = millis();
    }
    else if (receive.indexOf("_Der_Hund_hinlegt_#06") >= 0) {
      action = hinlegen;
      watchingTime = millis();
    }
    else if (receive.indexOf("_Der_Hund_schuettelt_haende_#07") >= 0) {
      action = haendeSchuetteln;
      watchingTime = millis();
    }
    else if (receive.indexOf("_Der_Hund_lernt_jetzt_kennen_#15") >= 0) {
      state = kennenlernen;
      watchingTime = millis();
    }
    else if (receive.indexOf("_Der_Hund_hat_kennengelernt_#16") >= 0) {
      state = allesGute;
      watchingTime = millis();
    }
    else if (receive.indexOf("_Man_kann_sprechen_#17") >= 0) {
      state = sprechen;
      watchingTime = millis();
    }
    else if (receive.indexOf("_Man_kann_nicht_sprechen_#18") >= 0) {
      state = allesGute;
      watchingTime = millis();
    }

    PT_YIELD(pt);
  }

  PT_END(pt);
}

static const uint16_t standArg[8] = { 180, 190, 165, 160, 165, 190, 190, 170 };//{ 200, 120, 205, 130, 125, 220, 140, 250 };
void stand() {
  HCPCA9685.Servo(0, standArg[0]);
  HCPCA9685.Servo(1, standArg[1]);

  HCPCA9685.Servo(2, standArg[2]);
  HCPCA9685.Servo(3, standArg[3]);

  HCPCA9685.Servo(4, standArg[4]);
  HCPCA9685.Servo(5, standArg[5]);

  HCPCA9685.Servo(6, standArg[6]);
  HCPCA9685.Servo(7, standArg[7]);
}

void stepping() {
  delay(100);
  HCPCA9685.Servo(0, standArg[0] - 50);
  HCPCA9685.Servo(1, standArg[1] + 30);
  delay(300);
  HCPCA9685.Servo(0, standArg[0]);
  HCPCA9685.Servo(1, standArg[1]);

  HCPCA9685.Servo(2, standArg[2] - 50);
  HCPCA9685.Servo(3, standArg[3] + 30);
  delay(300);
  HCPCA9685.Servo(2, standArg[2]);
  HCPCA9685.Servo(3, standArg[3]);

  HCPCA9685.Servo(4, standArg[4] + 50);
  HCPCA9685.Servo(5, standArg[5] - 30);
  delay(300);
  HCPCA9685.Servo(4, standArg[4]);
  HCPCA9685.Servo(5, standArg[5]);

  HCPCA9685.Servo(6, standArg[6] + 50);
  HCPCA9685.Servo(7, standArg[7] - 30);
  delay(300);
  HCPCA9685.Servo(6, standArg[6]);
  HCPCA9685.Servo(7, standArg[7]);
}

static void sitzen_und_handshakeAction(Pt *pt) {
  PT_BEGIN(pt);

  static int16_t i, j;
  static const uint16_t sitPos[8] = {280, 190, 35, 360, 90, 190, 340, 20};
  static uint16_t sitArg[8];

  for (;;) {

    PT_WAIT_UNTIL(pt, action == haendeSchuetteln || action == sitzen);
    PT_TIMER_DELAY(pt, 10);
    stand();
    PT_TIMER_DELAY(pt, 500);

    sitArg[0] = standArg[0]; sitArg[1] = standArg[1]; sitArg[2] = standArg[2]; sitArg[3] = standArg[3];
    sitArg[4] = standArg[4]; sitArg[5] = standArg[5]; sitArg[6] = standArg[6]; sitArg[7] = standArg[7];

    for (i = 0; i < 52; i++) {
      HCPCA9685.Servo(2, sitArg[2]);
      sitArg[2] -= 15;
      if (sitArg[2] < sitPos[2]) sitArg[2] = sitPos[2];
      PT_YIELD(pt);

      HCPCA9685.Servo(3, sitArg[3]);
      sitArg[3] += 13;
      if (sitArg[3] > sitPos[3]) sitArg[3] = sitPos[3];
      PT_YIELD(pt);

      HCPCA9685.Servo(6, sitArg[6]);
      sitArg[6] += 15;
      if (sitArg[6] > sitPos[6]) sitArg[6] = sitPos[6];
      PT_YIELD(pt);

      HCPCA9685.Servo(7, sitArg[7]);
      sitArg[7] -= 13;
      if (sitArg[7] < sitPos[7]) sitArg[7] = sitPos[7];
      PT_YIELD(pt);

      PT_TIMER_DELAY(pt, 15);
    }

    for (i = 0; i < 52; i++) {
      HCPCA9685.Servo(0, sitArg[0]);
      sitArg[0] += 5;
      if (sitArg[0] > sitPos[0]) sitArg[0] = sitPos[0];
      PT_YIELD(pt);

      HCPCA9685.Servo(1, sitArg[1]);
      sitArg[1] += 7;
      if (sitArg[1] > sitPos[1]) sitArg[1] = sitPos[1];
      PT_YIELD(pt);

      HCPCA9685.Servo(4, sitArg[4]);
      sitArg[4] -= 5;
      if (sitArg[4] < sitPos[4]) sitArg[4] = sitPos[4];
      PT_YIELD(pt);

      HCPCA9685.Servo(5, sitArg[5]);
      sitArg[5] -= 7;
      if (sitArg[5] < sitPos[5]) sitArg[5] = sitPos[5];
      PT_YIELD(pt);

      PT_TIMER_DELAY(pt, 15);
    }
    PT_YIELD(pt);

    HCPCA9685.Servo(0, sitPos[0] + 30);
    HCPCA9685.Servo(1, sitPos[1] - 30);
    PT_TIMER_DELAY(pt, 300);
    HCPCA9685.Servo(0, sitPos[0]);
    HCPCA9685.Servo(1, sitPos[1]);
    PT_TIMER_DELAY(pt, 300);
    HCPCA9685.Servo(4, sitPos[4] - 30);
    HCPCA9685.Servo(5, sitPos[5] + 30);
    PT_TIMER_DELAY(pt, 300);
    HCPCA9685.Servo(4, sitPos[4]);
    HCPCA9685.Servo(5, sitPos[5]);
    //
label_handshake:
    if (action == haendeSchuetteln) {
      HCPCA9685.Servo(0, 145);
      HCPCA9685.Servo(1, 10);
      PT_TIMER_DELAY(pt, 500);

      PT_WAIT_UNTIL(pt, action != haendeSchuetteln || cs_4_5.capacitiveSensor(30) >= 200);
      //PT_TIMER_DELAY(pt, 500);
      delay(500);
      PT_WAIT_UNTIL(pt, action != haendeSchuetteln || cs_4_5.capacitiveSensor(30) < 190);

      HCPCA9685.Servo(1, sitPos[1]);
      for (i = 100; i <= sitPos[0]; i += 5) {
        HCPCA9685.Servo(0, i);
        //PT_TIMER_DELAY(pt, 20);
        delay(20);
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
    for (i = sitArg[0], j = sitArg[4]; i >= 30, j <= 320; i -= 5, j += 5) {
      HCPCA9685.Servo(0, i);
      HCPCA9685.Servo(4, j);
      delay(10);
    }
    delay(100);

    HCPCA9685.Servo(3, 30);
    HCPCA9685.Servo(7, 330);
    delay(300);

    for (i = 0; i < 52; i++) {
      HCPCA9685.Servo(2, sitArg[2]);
      sitArg[2] += 5;
      if (sitArg[2] > standArg[2]) sitArg[2] = standArg[2];

      HCPCA9685.Servo(6, sitArg[6]);
      sitArg[6] -= 5;
      if (sitArg[6] < standArg[6]) sitArg[6] = standArg[6];

      delay(10);
    }
    delay(100);

    for (i = 0; i < 52; i++) {
      HCPCA9685.Servo(3, sitArg[3]);
      sitArg[3] += 5;
      if (sitArg[3] > standArg[3]) sitArg[3] = standArg[3];

      HCPCA9685.Servo(7, sitArg[7]);
      sitArg[7] -= 5;
      if (sitArg[7] < standArg[7]) sitArg[7] = standArg[7];

      delay(10);
    }

    delay(300);

    stand();
    stepping();
    PT_YIELD(pt);
  }

  PT_END(pt);
}

static void lieAction(Pt *pt) {
  PT_BEGIN(pt);

  static uint16_t i;
  static const int16_t liePos[8] = { 30, 190, 330, 160, 340, 190, 30, 170 };
  static int16_t lieArg[9];

  for (;;) {
    PT_WAIT_UNTIL(pt, action == hinlegen);
    PT_TIMER_DELAY(pt, 10);
    stand();
    PT_TIMER_DELAY(pt, 500);

    lieArg[0] = standArg[0]; lieArg[1] = standArg[1]; lieArg[2] = standArg[2]; lieArg[3] = standArg[3];
    lieArg[4] = standArg[4]; lieArg[5] = standArg[5]; lieArg[6] = standArg[6]; lieArg[7] = standArg[7];

    for (i = 0; i < 49; i++) {
      HCPCA9685.Servo(0, lieArg[0]);
      lieArg[0] -= 5;
      if (lieArg[0] < liePos[0]) lieArg[0] = liePos[0];
      PT_YIELD(pt);

      HCPCA9685.Servo(1, lieArg[1]);
      lieArg[1] += 5;
      if (lieArg[1] > liePos[1]) lieArg[1] = liePos[1];
      PT_YIELD(pt);

      HCPCA9685.Servo(2, lieArg[2]);
      lieArg[2] += 5;
      if (lieArg[2] > liePos[2]) lieArg[2] = liePos[2];
      PT_YIELD(pt);

      HCPCA9685.Servo(3, lieArg[3]);
      lieArg[3] += 5;
      if (lieArg[3] > liePos[3]) lieArg[3] = liePos[3];
      PT_YIELD(pt);

      HCPCA9685.Servo(4, lieArg[4]);
      lieArg[4] += 5;
      if (lieArg[4] > liePos[4]) lieArg[4] = liePos[4];
      PT_YIELD(pt);

      HCPCA9685.Servo(5, lieArg[5]);
      lieArg[5] -= 5;
      if (lieArg[5] < liePos[5]) lieArg[5] = liePos[5];
      PT_YIELD(pt);

      HCPCA9685.Servo(6, lieArg[6]);
      lieArg[6] -= 5;
      if (lieArg[6] < liePos[6]) lieArg[6] = liePos[6];
      PT_YIELD(pt);

      HCPCA9685.Servo(7, lieArg[7]);
      lieArg[7] -= 5;
      if (lieArg[7] < liePos[7]) lieArg[7] = liePos[7];
      PT_YIELD(pt);

      PT_TIMER_DELAY(pt, 15);
    }
    PT_YIELD(pt);

    PT_WAIT_UNTIL(pt, action != hinlegen);

    // /////////////// thread lock
    delay(500);

    HCPCA9685.Servo(3, 330);
    HCPCA9685.Servo(7, 30);
    delay(300);

    for (i = 0; i < 33; i++) {
      HCPCA9685.Servo(2, lieArg[2]);
      lieArg[2] -= 5;
      if (lieArg[2] < standArg[2]) lieArg[2] = standArg[2];

      //HCPCA9685.Servo(3, lieArg[3]);
      //lieArg[3] -= 5;
      //if (lieArg[3] < standArg[3]) lieArg[3] = standArg[3];

      HCPCA9685.Servo(6, lieArg[6]);
      lieArg[6] += 5;
      if (lieArg[6] > standArg[6]) lieArg[6] = standArg[6];

      //HCPCA9685.Servo(7, lieArg[7]);
      //lieArg[7] += 5;
      //if (lieArg[7] > standArg[7]) lieArg[7] = standArg[7];

      delay(15);
    }
    delay(300);

    HCPCA9685.Servo(3, standArg[3]);
    HCPCA9685.Servo(7, standArg[7]);
    delay(300);

    HCPCA9685.Servo(1, 30);
    HCPCA9685.Servo(5, 330);
    delay(300);

    for (i = 0; i < 49; i++) {
      HCPCA9685.Servo(0, lieArg[0]);
      lieArg[0] += 5;
      if (lieArg[0] > standArg[0]) lieArg[0] = standArg[0];

      //HCPCA9685.Servo(1, lieArg[1]);
      //lieArg[1] -= 5;
      //if (lieArg[1] < standArg[1]) lieArg[1] = standArg[1];

      HCPCA9685.Servo(4, lieArg[4]);
      lieArg[4] -= 5;
      if (lieArg[4] < standArg[4]) lieArg[4] = standArg[4];

      //HCPCA9685.Servo(5, lieArg[5]);
      //lieArg[5] += 5;
      //if (lieArg[5] > standArg[5]) lieArg[5] = standArg[5];

      delay(15);
    }
    delay(300);

    HCPCA9685.Servo(1, standArg[1]);
    HCPCA9685.Servo(5, standArg[5]);
    delay(300);

    stand();
    stepping();
    PT_YIELD(pt);
  }

  PT_END(pt);
}

////////////////////////////////////////////////////////////////////////////////////
uint8_t lowerleg_fork = 100, lowerleg_back = 20;

#define double2(x) (((int)(x * 100.0)) / 100.0)
static void sensingDirection(Pt *pt) {
  PT_BEGIN(pt);

  static double gz;
  static double offset = 0.0;
  static int16_t original_links_slant, original_rechts_slant;

  for (;;) {
    PT_WAIT_UNTIL(pt, action == geradeaus);
    offset = 0.0;
    original_links_slant = links_slant;
    original_rechts_slant = rechts_slant;

    for (;;) {
      // read the sensor
      IMU.readSensor();
      gz = IMU.getGyroZ_rads();
      PT_YIELD(pt);

      offset += double2(gz);

      if (abs(offset) > 3) {
        if (offset > 0) {
          offset_direction = right;
        }
        else {
          offset_direction = left;
        }
      }
      else {
        offset_direction = middle;
      }
      PT_YIELD(pt);


      if (offset_direction == left) {
        //Serial.println("<<");
        links_slant = 40;
        rechts_slant = 20;
      }
      else if (offset_direction == right) {
        //Serial.println(">>");
        links_slant = 20;
        rechts_slant = 40;
      }
      else if (offset_direction == middle) {
        //Serial.println("||");
        links_slant = original_links_slant;
        rechts_slant = original_rechts_slant;
      }


      PT_TIMER_DELAY(pt, 30);
      if (action != geradeaus) {
        links_slant = original_links_slant;
        rechts_slant = original_rechts_slant;

        break;
      }
    }
    PT_YIELD(pt);
  }

  PT_END(pt);
}

static void foot0_move(Pt *pt) {
  PT_BEGIN(pt);

  static int16_t i;
  static const uint8_t thighSpeed = 15, crusSpeed = 10, step = 5;
  static uint16_t actionPosition[3][4] = {{standArg[0] - links_slant - forkfoot_feedback, standArg[0] + links_slant - forkfoot_feedback, standArg[1] + lowerleg_back,  standArg[1] - lowerleg_fork},
    {standArg[0] - 20 - forkfoot_feedback, standArg[0] + 20 - forkfoot_feedback, standArg[1] + lowerleg_back,  standArg[1] - lowerleg_fork},
    {standArg[0] - links_slant - forkfoot_feedback, standArg[0] + links_slant - forkfoot_feedback, standArg[1] + lowerleg_back,  standArg[1] - lowerleg_fork}
  };

  for (;;) {
    PT_WAIT_UNTIL(pt, PtEnable[0]);

    actionPosition[0][0] = standArg[0] - links_slant - forkfoot_feedback;
    actionPosition[0][1] = standArg[0] + links_slant - forkfoot_feedback;
    //actionPosition[1][0] = standArg[0] - 20 - forkfoot_feedback;
    //actionPosition[1][1] = standArg[0] + 20 - forkfoot_feedback;
    //actionPosition[2][0] = standArg[0] - links_slant - forkfoot_feedback;
    //actionPosition[2][1] = standArg[0] + links_slant - forkfoot_feedback;
    PT_YIELD(pt);

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

  static int16_t i;
  static const uint8_t thighSpeed = 15, crusSpeed = 10, step = 5;
  static uint16_t actionPosition[3][4] = {{standArg[2] - links_slant + backfoot_feedback, standArg[2] + links_slant + backfoot_feedback, standArg[3] + lowerleg_back,  standArg[3] - lowerleg_fork},
    {standArg[2] - 20 + backfoot_feedback, standArg[2] + 20 + backfoot_feedback, standArg[3] + lowerleg_back,  standArg[3] - lowerleg_fork},
    {standArg[2] - links_slant + backfoot_feedback, standArg[2] + links_slant + backfoot_feedback, standArg[3] + lowerleg_back,  standArg[3] - lowerleg_fork}
  };

  for (;;) {
    PT_WAIT_UNTIL(pt, PtEnable[1]);

    actionPosition[0][0] = standArg[2] - links_slant + backfoot_feedback;
    actionPosition[0][1] = standArg[2] + links_slant + backfoot_feedback;
    //actionPosition[1][0] = standArg[2] - 20 + backfoot_feedback;
    //actionPosition[1][1] = standArg[2] + 20 + backfoot_feedback;
    //actionPosition[2][0] = standArg[2] - links_slant + backfoot_feedback;
    //actionPosition[2][1] = standArg[2] + links_slant + backfoot_feedback;
    PT_YIELD(pt);

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

  static int16_t i;
  static const uint8_t thighSpeed = 15, crusSpeed = 10, step = 5;
  static uint16_t actionPosition[3][4] = {{standArg[4] + rechts_slant + forkfoot_feedback, standArg[4] - rechts_slant + forkfoot_feedback, standArg[5] - lowerleg_back, standArg[5] + lowerleg_fork},
    {standArg[4] + rechts_slant + forkfoot_feedback, standArg[4] - rechts_slant + forkfoot_feedback, standArg[5] - lowerleg_back, standArg[5] + lowerleg_fork},
    {standArg[4] + 20 + forkfoot_feedback, standArg[4] - 20 + forkfoot_feedback, standArg[5] - lowerleg_back, standArg[5] + lowerleg_fork}
  };

  for (;;) {
    PT_WAIT_UNTIL(pt, PtEnable[2]);

    actionPosition[0][0] = standArg[4] + rechts_slant + forkfoot_feedback;
    actionPosition[0][1] = standArg[4] - rechts_slant + forkfoot_feedback;
    //actionPosition[1][0] = standArg[4] + rechts_slant + forkfoot_feedback;
    //actionPosition[1][1] = standArg[4] - rechts_slant + forkfoot_feedback;
    //actionPosition[2][0] = standArg[4] + 20 + forkfoot_feedback;
    //actionPosition[2][1] = standArg[4] - 20 + forkfoot_feedback;
    PT_YIELD(pt);

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

  static int16_t i;
  static const uint8_t thighSpeed = 15, crusSpeed = 10, step = 5;
  static uint16_t actionPosition[3][4] = {{standArg[6] + rechts_slant - backfoot_feedback,  standArg[6] - rechts_slant - backfoot_feedback, standArg[7] - lowerleg_back, standArg[7] + lowerleg_fork},
    {standArg[6] + rechts_slant - backfoot_feedback,  standArg[6] - rechts_slant - backfoot_feedback, standArg[7] - lowerleg_back, standArg[7] + lowerleg_fork},
    {standArg[6] + 20 - backfoot_feedback,  standArg[6] - 20 - backfoot_feedback, standArg[7] - lowerleg_back, standArg[7] + lowerleg_fork}
  };

  while (1) {
    PT_WAIT_UNTIL(pt, PtEnable[3]);

    actionPosition[0][0] = standArg[6] + rechts_slant - backfoot_feedback;
    actionPosition[0][1] = standArg[6] - rechts_slant - backfoot_feedback;
    //actionPosition[1][0] = standArg[6] + rechts_slant - backfoot_feedback;
    //actionPosition[1][1] = standArg[6] - rechts_slant - backfoot_feedback;
    //actionPosition[2][0] = standArg[6] + 20 - backfoot_feedback;
    //actionPosition[2][1] = standArg[6] - 20 - backfoot_feedback;
    PT_YIELD(pt);

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

static void watchDog(Pt *pt) {
  PT_BEGIN(pt);
  for (;;) {
    if ((millis() - watchingTime) > 600000) {
      action = hinlegen;
      watchingTime = millis();
    }

    //Serial.println(millis() - watchingTime);
    PT_YIELD(pt);
  }
  PT_END(pt);
}





