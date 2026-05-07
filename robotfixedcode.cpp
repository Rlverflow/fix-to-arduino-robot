/************************************************************************
just so you know, i would rather code c whilst drunk than mess with c++, ama keep this updayed version on github
*************************************************************************/
/*************************************************************
   ROBOT CAR + ROBOTIC ARM
   Improved stable version
   - Cleaner servo handling
   - Deadzone filtering
   - Non-blocking pick/drop sequence
   - Better Bluetooth responsiveness
   - Safer initialization
*************************************************************/

#define REMOTEXY_MODE__SOFTSERIAL

#include <SoftwareSerial.h>
#include <RemoteXY.h>
#include <Servo.h>

/*************************************************************
   REMOTEXY CONFIG
*************************************************************/
#define REMOTEXY_SERIAL_RX 10
#define REMOTEXY_SERIAL_TX 11
#define REMOTEXY_SERIAL_SPEED 9600

#pragma pack(push, 1)

uint8_t const PROGMEM RemoteXY_CONF_PROGMEM[] = {
255,12,0,0,0,191,0,19,0,0,0,0,0,1,200,84,1,1,17,0,
5,9,21,44,44,0,25,26,31,5,122,5,36,36,0,25,26,31,5,122,
45,36,36,0,25,26,31,1,176,8,13,13,0,6,31,0,1,176,49,13,
13,0,1,31,0,1,176,26,13,13,0,133,31,0,1,176,67,13,13,0,
231,31,0,4,83,18,10,46,0,25,26,129,22,6,19,6,0,16,77,111,
116,105,111,110,0,129,108,38,12,6,0,16,65,114,109,0,1,56,63,13,
13,0,246,31,0,129,178,13,9,4,0,16,71,114,97,98,0,129,178,72,
9,4,0,16,68,114,111,112,0,129,179,54,8,4,0,16,80,105,99,107,
0,129,178,31,10,4,0,16,79,112,101,110,0,129,58,68,9,4,0,16,
83,116,111,112,0,129,80,13,17,6,0,16,83,112,101,101,100,0
};

struct {

  int8_t Motion_Front_Back;
  int8_t Motion_Left_Right;

  int8_t joystick_03_x;
  int8_t joystick_03_y;

  int8_t joystick_02_x;
  int8_t joystick_02_y;

  uint8_t Grab;
  uint8_t Pick;
  uint8_t Open;
  uint8_t Drop;

  int8_t slider_01;

  uint8_t Stop;

  uint8_t connect_flag;

} RemoteXY;

#pragma pack(pop)

/*************************************************************
   MOTOR PINS
*************************************************************/
#define IN1 2
#define IN2 3
#define IN3 4
#define IN4 7

#define ENA 5
#define ENB 6

/*************************************************************
   SERVO PINS
*************************************************************/
#define PIN_BASE      12
#define PIN_SHOULDER  13
#define PIN_ELBOW     A0
#define PIN_WRIST     A1
#define PIN_GRIPPER   A2
#define PIN_ROLL      A3

/*************************************************************
   SERVOS
*************************************************************/
Servo sBase;
Servo sShoulder;
Servo sElbow;
Servo sWrist;
Servo sGrip;
Servo sRoll;

/*************************************************************
   ARM VARIABLES
*************************************************************/
int base = 90;
int sh   = 90;
int el   = 90;
int wr   = 90;
int grip = 30;
int roll = 90;

#define GRIP_OPEN   30
#define GRIP_CLOSE  120

/*************************************************************
   SETTINGS
*************************************************************/
int speedPWM = 150;

#define DEADZONE 10

/*************************************************************
   PICK/DROP STATE MACHINE
*************************************************************/
bool pickActive = false;
bool dropActive = false;

unsigned long sequenceTimer = 0;
int sequenceStep = 0;

/*************************************************************
   APPLY ARM POSITIONS
*************************************************************/
void applyArm()
{
  sBase.write(base);
  sShoulder.write(sh);
  sElbow.write(el);
  sWrist.write(wr);
  sGrip.write(grip);
  sRoll.write(roll);
}

/*************************************************************
   MOTOR CONTROL
*************************************************************/
void stopMotors()
{
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

void moveRobot(int x, int y)
{
  if (RemoteXY.Stop)
  {
    stopMotors();
    return;
  }

  if (y > 20)
  {
    // FORWARD
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);

    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  }
  else if (y < -20)
  {
    // BACKWARD
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);

    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
  }
  else if (x > 20)
  {
    // RIGHT
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);

    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
  }
  else if (x < -20)
  {
    // LEFT
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);

    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  }
  else
  {
    stopMotors();
  }

  analogWrite(ENA, speedPWM);
  analogWrite(ENB, speedPWM);
}

/*************************************************************
   NON-BLOCKING PICK SEQUENCE
*************************************************************/
void handlePickSequence()
{
  if (!pickActive) return;

  if (millis() - sequenceTimer < 400) return;

  sequenceTimer = millis();

  switch (sequenceStep)
  {
    case 0:
      sh = 120;
      el = 60;
      wr = 80;
      break;

    case 1:
      sh = 140;
      el = 40;
      break;

    case 2:
      grip = GRIP_CLOSE;
      break;

    case 3:
      sh = 90;
      el = 90;
      pickActive = false;
      sequenceStep = -1;
      break;
  }

  sequenceStep++;
}

/*************************************************************
   NON-BLOCKING DROP SEQUENCE
*************************************************************/
void handleDropSequence()
{
  if (!dropActive) return;

  if (millis() - sequenceTimer < 400) return;

  sequenceTimer = millis();

  switch (sequenceStep)
  {
    case 0:
      sh = 110;
      el = 60;
      wr = 70;
      break;

    case 1:
      grip = GRIP_OPEN;
      break;

    case 2:
      sh = 90;
      el = 90;
      dropActive = false;
      sequenceStep = -1;
      break;
  }

  sequenceStep++;
}

/*************************************************************
   SETUP
*************************************************************/
void setup()
{
  RemoteXY_Init();

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  sBase.attach(PIN_BASE);
  sShoulder.attach(PIN_SHOULDER);
  sElbow.attach(PIN_ELBOW);
  sWrist.attach(PIN_WRIST);
  sGrip.attach(PIN_GRIPPER);
  sRoll.attach(PIN_ROLL);

  applyArm();

  stopMotors();
}

/*************************************************************
   LOOP
*************************************************************/
void loop()
{
  RemoteXY_Handler();

  /**************** SPEED ****************/
  speedPWM = map(RemoteXY.slider_01, 0, 100, 80, 255);

  /**************** MOVEMENT ****************/
  moveRobot(
    RemoteXY.Motion_Left_Right,
    RemoteXY.Motion_Front_Back
  );

  /**************** ARM MANUAL CONTROL ****************/

  if (abs(RemoteXY.joystick_02_x) > DEADZONE)
    base += RemoteXY.joystick_02_x / 20;

  if (abs(RemoteXY.joystick_02_y) > DEADZONE)
    sh += RemoteXY.joystick_02_y / 20;

  if (abs(RemoteXY.joystick_03_x) > DEADZONE)
    el += RemoteXY.joystick_03_x / 20;

  if (abs(RemoteXY.joystick_03_y) > DEADZONE)
    wr += RemoteXY.joystick_03_y / 20;

  /**************** GRIPPER ****************/

  if (RemoteXY.Grab)
    grip += 2;

  if (RemoteXY.Open)
    grip -= 2;

  /**************** START PICK ****************/

  if (RemoteXY.Pick && !pickActive)
  {
    pickActive = true;
    dropActive = false;

    sequenceStep = 0;
    sequenceTimer = millis();
  }

  /**************** START DROP ****************/

  if (RemoteXY.Drop && !dropActive)
  {
    dropActive = true;
    pickActive = false;

    sequenceStep = 0;
    sequenceTimer = millis();
  }

  /**************** HANDLE SEQUENCES ****************/

  handlePickSequence();
  handleDropSequence();

  /**************** LIMITS ****************/

  base = constrain(base, 0, 180);
  sh   = constrain(sh, 30, 150);
  el   = constrain(el, 0, 170);
  wr   = constrain(wr, 0, 180);
  grip = constrain(grip, 30, 120);
  roll = constrain(roll, 0, 180);

  /**************** APPLY ****************/

  applyArm();
}