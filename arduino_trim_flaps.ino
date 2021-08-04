// This connects a trim and flaps control to SPAD.neXt via Simconnect.
//
// SPAD.neXt >= 0.9.7.5 required
//

#include <CmdMessenger.h>
#include <Encoder.h>
#include <Servo.h>

// Pinout
const int elevTrimEncoderPins[] = {2, 3};
const int elevTrimServoPin = 4;
const int flapsSwitchPins[] = {8, 9};
const int flapsServoPin = 10;

// Timers
unsigned long previousUpdate = 0;

// Listen on serial connection for messages from the pc
CmdMessenger messenger(Serial);

String vJoy="1234:BEAD:0";

// This is the list of recognized commands. These can be commands that can either be sent
// or received.
// In order to receive, attach a callback function to these events
enum
{
  kRequest = 0,         // Request from SPAD.neXt              ... Documentation lists as "Command:0"
  kCommand = 1,         // Command to SPAD.neXt                ... Documentation lists as "Command:1"
  kEvent = 2,           // Events from SPAD.neXt               ... Documentation lists as "Command:2"
  kDebug = 3,           // Debug strings to SPAD.neXt Logfile  ... Documentation lists as "Debug Channel"
  kSimCommand = 4,      // Send Event to Simulation            ... Documentation lists as "Command:4"
                        // Command IDs 5-9 are Reserved.
                        // Command IDs 10-49 are for Data Updates.  Where we "expose/subscibe" to data to process in our sketch..   
  sElevTrim = 10,    // CMDID for exposed data to SPAD.neXt.  We will see the data later as a Local Variable in Spad.Next
  rElevTrimInd = 11,
  sFlapsPos = 12,
  rFlapsInd = 13,
};

bool isReady = false;

// Elevator trim wheel encoder variables
// Encoder ElevTrimEncoder(elevTrimEncoderPins[0], elevTrimEncoderPins[1]);

// Elevator trim indicator servo variables
Servo ElevTrimServo;
const int maxElevTrimIndicatorDeflection = 60;
const int maxElevTrimIndicatorPosition = 90 + maxElevTrimIndicatorDeflection / 2;
const int minElevTrimIndicatorPosition = 90 - maxElevTrimIndicatorDeflection / 2;

// Flaps switch variables
uint8_t oldFlapsSwitchPosition  = -99;

// Flaps indicator servo variables
Servo FlapsServo;
const int maxFlapsIndicatorDeflection = 60;
const int maxFlapsIndicatorPosition = 90 + maxFlapsIndicatorDeflection / 2;
const int minFlapsIndicatorPosition = 90 - maxFlapsIndicatorDeflection / 2;

void attachCommandCallbacks()
{
  // Attach callback methods
  messenger.attach(onUnknownCommand);
  messenger.attach(kRequest, onIdentifyRequest);
  messenger.attach(rElevTrimInd, onElevTrimInd);
  messenger.attach(rFlapsInd, onFlapsInd);
}

// ------------------  C A L L B A C K S -----------------------

// Called when a received command has no attached function
void onUnknownCommand()
{
  messenger.sendCmd(kDebug, F("UNKNOWN COMMAND")); 
}

// Callback function to respond to indentify request. This is part of the
// Auto connection handshake.
void onIdentifyRequest()
{
  char *szRequest = messenger.readStringArg();

  if (strcmp(szRequest, "INIT") == 0) {
    messenger.sendCmdStart(kRequest);
    messenger.sendCmdArg(F("SPAD"));
    // Unique Device ID: Change this!
    messenger.sendCmdArg(F("{f392d590-a755-413b-8daf-333d4afa58bd}"));
    // Device Name for UI
    messenger.sendCmdArg(F("Trim/flaps"));
    messenger.sendCmdEnd();
  }

  else if (strcmp(szRequest, "PING") == 0) {
    messenger.sendCmdStart(kRequest);
    messenger.sendCmdArg(F("PONG"));
    messenger.sendCmdArg(messenger.readInt32Arg());
    messenger.sendCmdEnd();
  }
  
  else if (strcmp(szRequest, "CONFIG") == 0) {

    // Expose Elevator Trim Wheel
    messenger.sendCmdStart(kCommand);
    messenger.sendCmdArg(F("ADD"));
    messenger.sendCmdArg(sElevTrim);
    messenger.sendCmdArg(F("controls/elevator_trim"));
    messenger.sendCmdArg(F("S8"));
    messenger.sendCmdArg(F("RW"));
    messenger.sendCmdArg(F("Elevator trim wheel"));
    messenger.sendCmdEnd();
    
    // Request Elevator Trim Updates
    messenger.sendCmdStart(kCommand);
    messenger.sendCmdArg(F("ADD"));
    messenger.sendCmdArg(rElevTrimInd);
    messenger.sendCmdArg(F("indicators/elevator_trim_pct"));
    messenger.sendCmdArg(F("S8"));
    messenger.sendCmdArg(F("RO"));
    messenger.sendCmdArg(F("Elevator trim indicator (pct)"));
    messenger.sendCmdEnd();
    
    // Expose Flaps Switch
    messenger.sendCmdStart(kCommand);
    messenger.sendCmdArg(F("ADD"));
    messenger.sendCmdArg(sFlapsPos);
    messenger.sendCmdArg(F("controls/flaps_handle"));
    messenger.sendCmdArg(F("U8"));
    messenger.sendCmdArg(F("RW"));
    messenger.sendCmdArg(F("Flaps handle position"));
    messenger.sendCmdEnd();
    
    // Request Flaps Updates
    messenger.sendCmdStart(kCommand);
    messenger.sendCmdArg(F("ADD"));
    messenger.sendCmdArg(rFlapsInd);
    messenger.sendCmdArg(F("indicators/flaps_position_pct"));
    messenger.sendCmdArg(F("S8"));
    messenger.sendCmdArg(F("RO"));
    messenger.sendCmdArg(F("Flaps position (pct)"));
    messenger.sendCmdEnd();

    // tell SPAD.neXT we are done with config
    messenger.sendCmd(kRequest, F("CONFIG"));
    isReady = true;
  }
}

// --------------- INDICATORS --------------- 

// Update elevator trim indicator
void onElevTrimInd()
{
  float newPosition = messenger.readFloatArg();

  int elevTrimServoPos = int(90 + newPosition * maxElevTrimIndicatorDeflection);
  elevTrimServoPos = clamp(elevTrimServoPos, minElevTrimIndicatorPosition, maxElevTrimIndicatorPosition);

  // messenger.sendCmd(kDebug, "Trim indicator: ", elevTrimServoPos);

  ElevTrimServo.write(elevTrimServoPos);
}

void onFlapsInd()
{
  float newPosition = messenger.readFloatArg();
  
  int flapsServoPos = 120 - (newPosition * maxFlapsIndicatorDeflection);
  flapsServoPos = clamp(flapsServoPos, minFlapsIndicatorPosition, maxFlapsIndicatorPosition);

  // messenger.sendCmd(kDebug, "Flaps indicator: ", flapsServoPos);
  
  FlapsServo.write(flapsServoPos);
}

// --------------- INPUTS  -------------------

// -------------
// -------------  START Encoder Setup -------------

//Uncomment this for HALFSTEP encoder operation -- The PropWash works with Halfstep..use the slashes next to it to comment it out if your Encoder does not need this.
// #define HALF_STEP
#define ENABLE_PULLUPS  // If we connect the common direct to GND we need the Pull Ups inside.
#define NUMROTARIES 1  // Number of Rotaries that we will connect

struct rotariesdef {
  byte pin1;
  byte pin2;
  int ccwchar;
  int cwchar;
  volatile unsigned char state;
};

// Increase the Structure if you want to add more Rotaries ... Not needed for the 480 which only has the 1 Dual Concentric === which equals 2...
rotariesdef rotaries[NUMROTARIES] {
  {3,2,0,1,0},
};

#define DIR_CCW 0x10
#define DIR_CW 0x20
#define R_START 0x0

// Use the half-step state table (emits a code at 00 and 11)
#ifdef HALF_STEP
  #define R_CCW_BEGIN 0x1
  #define R_CW_BEGIN 0x2
  #define R_START_M 0x3
  #define R_CW_BEGIN_M 0x4
  #define R_CCW_BEGIN_M 0x5
  const unsigned char ttable[6][4] = {
  // R_START (00)
  {R_START_M,            R_CW_BEGIN,     R_CCW_BEGIN,  R_START},
  // R_CCW_BEGIN
  {R_START_M | DIR_CCW, R_START,        R_CCW_BEGIN,  R_START},
  // R_CW_BEGIN
  {R_START_M | DIR_CW,  R_CW_BEGIN,     R_START,      R_START},
  // R_START_M (11)
  {R_START_M,            R_CCW_BEGIN_M,  R_CW_BEGIN_M, R_START},
  // R_CW_BEGIN_M
  {R_START_M,            R_START_M,      R_CW_BEGIN_M, R_START | DIR_CW},
  // R_CCW_BEGIN_M
  {R_START_M,            R_CCW_BEGIN_M,  R_START_M,    R_START | DIR_CCW},
};
// Use the full-step state table (emits a code at 00 only)
#else
  #define R_CW_FINAL 0x1
  #define R_CW_BEGIN 0x2
  #define R_CW_NEXT 0x3
  #define R_CCW_BEGIN 0x4
  #define R_CCW_FINAL 0x5
  #define R_CCW_NEXT 0x6

  const unsigned char ttable[7][4] = {
  // R_START
  {R_START,    R_CW_BEGIN,  R_CCW_BEGIN, R_START},
  // R_CW_FINAL
  {R_CW_NEXT,  R_START,     R_CW_FINAL,  R_START | DIR_CW},
  // R_CW_BEGIN
  {R_CW_NEXT,  R_CW_BEGIN,  R_START,     R_START},
  // R_CW_NEXT
  {R_CW_NEXT,  R_CW_BEGIN,  R_CW_FINAL,  R_START},
  // R_CCW_BEGIN
  {R_CCW_NEXT, R_START,     R_CCW_BEGIN, R_START},
  // R_CCW_FINAL
  {R_CCW_NEXT, R_CCW_FINAL, R_START,     R_START | DIR_CCW},
  // R_CCW_NEXT
  {R_CCW_NEXT, R_CCW_FINAL, R_CCW_BEGIN, R_START},
};
#endif

/* Call this once in setup(). This is the Rotary Encoder initialization Function*/
void rotary_init() {
  for (int i=0;i<NUMROTARIES;i++) {
    pinMode(rotaries[i].pin1, INPUT);
    pinMode(rotaries[i].pin2, INPUT);
    #ifdef ENABLE_PULLUPS
      digitalWrite(rotaries[i].pin1, HIGH);
      digitalWrite(rotaries[i].pin2, HIGH);
    #endif
  }
}

/* Read input pins and process for events. Call this either from a
 * loop or an interrupt (eg pin change or timer).
 *
 * Returns 0 on no event, otherwise 0x80 or 0x40 depending on the direction.
 */
unsigned char rotary_process(int _i) {
   unsigned char pinstate = (digitalRead(rotaries[_i].pin2) << 1) | digitalRead(rotaries[_i].pin1);
  rotaries[_i].state = ttable[rotaries[_i].state & 0xf][pinstate];
  return (rotaries[_i].state & 0x30);
}

void CheckAllEncoders(void) {
  for (int i=0;i<NUMROTARIES;i++) {
    unsigned char result = rotary_process(i);
    if (result == DIR_CCW) {
       messenger.sendCmdStart(kCommand);
       messenger.sendCmdArg(F("EMULATE"));
       messenger.sendCmdArg(vJoy);
       messenger.sendCmdArg(F("BUTTON_7"));
       messenger.sendCmdArg(F("PRESS"));
       messenger.sendCmdEnd();
       messenger.sendCmd(kDebug, F("Button 7 encCCW turn"));
     };
    if (result == DIR_CW) {
       messenger.sendCmdStart(kCommand);
       messenger.sendCmdArg(F("EMULATE"));
       messenger.sendCmdArg(vJoy);
       messenger.sendCmdArg(F("BUTTON_8"));
       messenger.sendCmdArg(F("PRESS"));
       messenger.sendCmdEnd();
       messenger.sendCmd(kDebug, F("Button 8 encCW turn"));   
    };
  }
}

bool BTN_eSWPS = HIGH;
bool BTN_eSWS = HIGH;

// -------------  END Encoder Setup -------------
// -------------

// Set flaps position
void setFlapsPos()
{
  int newPosition = !digitalRead(flapsSwitchPins[0]) * 2;
  newPosition += !digitalRead(flapsSwitchPins[1]);
  newPosition += 1;

  if (newPosition != oldFlapsSwitchPosition) // Has the switch moved?
  {
    oldFlapsSwitchPosition = newPosition;
    
    // messenger.sendCmdStart(sFlapsPos);
    // messenger.sendCmdArg(newPosition);
    // messenger.sendCmdEnd();

    String buttonStr = "BUTTON_";
    buttonStr += newPosition;
    String debugStr = "Set flaps: ";
    debugStr += newPosition;

    messenger.sendCmdStart(kCommand);
    messenger.sendCmdArg(F("EMULATE"));
    messenger.sendCmdArg(vJoy);
    messenger.sendCmdArg(buttonStr);
    messenger.sendCmdArg(F("PRESS"));
    messenger.sendCmdEnd();
    messenger.sendCmd(kDebug, debugStr);
  }
}

// --------------- S U P P O R T -------------------

// Returns if it has been more than interval (in ms) ago. Used for periodic actions
bool hasExpired(unsigned long &prevTime, unsigned long interval)
{
  if (millis() - prevTime > interval) {
    prevTime = millis();
    return true;
  } else     
    return false;
}

// ------------- M A I N -------------

// Setup function
void setup()
{

  // 115200 is typically the maximum speed for serial over USB
  Serial.begin(115200);

  // Attach my application's user-defined callback methods
  attachCommandCallbacks();

  // Initialize digital pins
  pinMode(flapsSwitchPins[0], INPUT_PULLUP);
  pinMode(flapsSwitchPins[1], INPUT_PULLUP);

  // Initialize encoder
  rotary_init();

  // Attach and cycle servos
  ElevTrimServo.attach(elevTrimServoPin);
  FlapsServo.attach(flapsServoPin);

  ElevTrimServo.write(minElevTrimIndicatorPosition);
  delay(350);
  ElevTrimServo.write(maxElevTrimIndicatorPosition);
  delay(350);
  ElevTrimServo.write(90);
  delay(350);
  FlapsServo.write(minFlapsIndicatorPosition);
  delay(350);
  FlapsServo.write(maxFlapsIndicatorPosition);
  delay(350);
  FlapsServo.write(90);
  delay(350);
}

// Loop function
void loop()
{
  // Process incoming serial data, and perform callbacks
  messenger.feedinSerialData();

  CheckAllEncoders();
  setFlapsPos();
}

int clamp(int value, int min_value, int max_value) {
  return max(min(value, max_value), min_value);
}