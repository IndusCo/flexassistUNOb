/*
  UNO B Right Actuator Emulator and Safety Node
  Receives commands from UNO A
  Reads KY 040 encoder and HC SR04 sonar
  Applies safety cutoff
  Drives L293D using PWM and direction
  Sends status back to UNO A
*/

#include <SoftwareSerial.h>

static const int LINK_RX_PIN = 11; // UNO B RX from UNO A TX
static const int LINK_TX_PIN = 12; // UNO B TX to UNO A RX
SoftwareSerial linkSerial(LINK_RX_PIN, LINK_TX_PIN);

// KY 040
static const int ENC_CLK_PIN = 2;
static const int ENC_DT_PIN  = 3;
static const int ENC_SW_PIN  = 4;

// HC SR04
static const int SONAR_TRIG_PIN = 7;
static const int SONAR_ECHO_PIN = 8;

// L293D channel A
static const int MOTOR_EN_PWM_PIN = 5;
static const int MOTOR_IN1_PIN = 9;
static const int MOTOR_IN2_PIN = 10;

// Safety LED
static const int SAFETY_LED_PIN = 6;

// Control state
volatile int encoderDelta = 0;
volatile int lastEncCLK = 0;

int cmdSpeed = 0;
int cmdDir = 1;

int knobSpeed = 0;
int appliedSpeed = 0;

bool safetyStop = false;
unsigned long lastStatusMs = 0;

static long readDistanceCm()
{
  digitalWrite(SONAR_TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(SONAR_TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(SONAR_TRIG_PIN, LOW);

  unsigned long duration = pulseIn(SONAR_ECHO_PIN, HIGH, 30000UL);
  if (duration == 0) return -1;

  long cm = (long)(duration / 58UL);
  return cm;
}

void isrEncCLK()
{
  int clk = digitalRead(ENC_CLK_PIN);
  int dt  = digitalRead(ENC_DT_PIN);

  if (clk != lastEncCLK)
  {
    lastEncCLK = clk;
    if (clk == HIGH)
    {
      if (dt == LOW) encoderDelta++;
      else encoderDelta--;
    }
  }
}

static void applyMotor(int speedValue, int dirValue)
{
  if (speedValue < 0) speedValue = 0;
  if (speedValue > 255) speedValue = 255;

  if (dirValue == 1)
  {
    digitalWrite(MOTOR_IN1_PIN, HIGH);
    digitalWrite(MOTOR_IN2_PIN, LOW);
  }
  else
  {
    digitalWrite(MOTOR_IN1_PIN, LOW);
    digitalWrite(MOTOR_IN2_PIN, HIGH);
  }

  analogWrite(MOTOR_EN_PWM_PIN, speedValue);
}

static void parseCommandLine(const String& line)
{
  // Expected: S,<speed>,<dir>
  if (line.length() < 3) return;
  if (line.charAt(0) != 'S') return;

  int firstComma = line.indexOf(',');
  int secondComma = line.indexOf(',', firstComma + 1);
  if (firstComma < 0 || secondComma < 0) return;

  int sp = line.substring(firstComma + 1, secondComma).toInt();
  int dr = line.substring(secondComma + 1).toInt();

  if (sp < 0) sp = 0;
  if (sp > 255) sp = 255;
  dr = (dr != 0) ? 1 : 0;

  cmdSpeed = sp;
  cmdDir = dr;
}

static void readCommands()
{
  while (linkSerial.available() > 0)
  {
    String line = linkSerial.readStringUntil('\n');
    line.trim();
    if (line.length() > 0)
    {
      parseCommandLine(line);
    }
  }
}

static void updateKnobSpeed()
{
  int delta;
  noInterrupts();
  delta = encoderDelta;
  encoderDelta = 0;
  interrupts();

  if (delta != 0)
  {
    knobSpeed += delta * 5;
    if (knobSpeed < 0) knobSpeed = 0;
    if (knobSpeed > 255) knobSpeed = 255;
  }

  // Push button quick zero
  static bool lastBtn = HIGH;
  bool btn = digitalRead(ENC_SW_PIN);
  if (lastBtn == HIGH && btn == LOW)
  {
    knobSpeed = 0;
  }
  lastBtn = btn;
}

static void computeAndApplyControl(long distanceCm)
{
  int requested = cmdSpeed;

  // Combine command and knob as a limiter
  if (knobSpeed < requested) requested = knobSpeed;

  const long stopThresholdCm = 25;

  if (distanceCm > 0 && distanceCm <= stopThresholdCm)
  {
    safetyStop = true;
    appliedSpeed = 0;
  }
  else
  {
    safetyStop = false;
    appliedSpeed = requested;
  }

  digitalWrite(SAFETY_LED_PIN, safetyStop ? HIGH : LOW);
  applyMotor(appliedSpeed, cmdDir);
}

static void sendStatus(long distanceCm)
{
  // Status: OK,<distCm>,<cmdSpeed>,<knobSpeed>,<appliedSpeed>,<safe>
  linkSerial.print("OK,");
  linkSerial.print(distanceCm);
  linkSerial.print(",");
  linkSerial.print(cmdSpeed);
  linkSerial.print(",");
  linkSerial.print(knobSpeed);
  linkSerial.print(",");
  linkSerial.print(appliedSpeed);
  linkSerial.print(",");
  linkSerial.print(safetyStop ? 1 : 0);
  linkSerial.print("\n");
}

void setup()
{
  pinMode(ENC_CLK_PIN, INPUT_PULLUP);
  pinMode(ENC_DT_PIN, INPUT_PULLUP);
  pinMode(ENC_SW_PIN, INPUT_PULLUP);

  pinMode(SONAR_TRIG_PIN, OUTPUT);
  pinMode(SONAR_ECHO_PIN, INPUT);

  pinMode(MOTOR_EN_PWM_PIN, OUTPUT);
  pinMode(MOTOR_IN1_PIN, OUTPUT);
  pinMode(MOTOR_IN2_PIN, OUTPUT);

  pinMode(SAFETY_LED_PIN, OUTPUT);
  digitalWrite(SAFETY_LED_PIN, LOW);

  lastEncCLK = digitalRead(ENC_CLK_PIN);
  attachInterrupt(digitalPinToInterrupt(ENC_CLK_PIN), isrEncCLK, CHANGE);

  linkSerial.begin(19200);

  applyMotor(0, 1);
}

void loop()
{
  readCommands();
  updateKnobSpeed();

  long dist = readDistanceCm();
  computeAndApplyControl(dist);

  unsigned long now = millis();
  if (now - lastStatusMs >= 200)
  {
    lastStatusMs = now;
    sendStatus(dist);
  }
}
