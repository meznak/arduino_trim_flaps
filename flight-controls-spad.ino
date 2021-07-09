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

String vJoy="1234:BEAD:1";

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
  sElevTrimPct = 10,    // CMDID for exposed data to SPAD.neXt.  We will see the data later as a Local Variable in Spad.Next
  rElevTrimInd = 11,
  sFlapsPos = 12,
  rFlapsInd = 13,
};

bool isReady = false;

// Elevator trim wheel encoder variables
Encoder ElevTrimEncoder(elevTrimEncoderPins[0], elevTrimEncoderPins[1]);
int oldElevTrimEncoderPosition = -9999;
int maxElevTrimEncoderPosition = 200;
int minElevTrimEncoderPosition = -maxElevTrimEncoderPosition;

// Elevator trim indicator servo variables
Servo ElevTrimServo;
const int maxElevTrimIndicatorDeflection = 30;
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
  messenger.sendCmd(kDebug, "UNKNOWN COMMAND"); 
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
    messenger.sendCmdArg(sElevTrimPct);
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

  else if (strcmp(szRequest, "START") == 0) {
    oldElevTrimEncoderPosition  = -9999;
    ElevTrimEncoder.write(0);
  }
}

// Update elevator trim indicator
void onElevTrimInd()
{
  float newPosition = messenger.readFloatArg();

  int elevTrimServoPos = int(90 + newPosition * maxElevTrimIndicatorDeflection);
  elevTrimServoPos = clamp(elevTrimServoPos, minElevTrimIndicatorPosition, maxElevTrimIndicatorPosition);

  messenger.sendCmd(kDebug, "Trim indicator: " + elevTrimServoPos);

  ElevTrimServo.write(elevTrimServoPos);
}

void onFlapsInd()
{
  int newPosition = messenger.readFloatArg() * 100;
  
  int flapsServoPos = 120 - (newPosition * maxFlapsIndicatorDeflection) / 100;
  flapsServoPos = clamp(flapsServoPos, minFlapsIndicatorPosition, maxFlapsIndicatorPosition);

  messenger.sendCmd(kDebug, "Flaps indicator: " + flapsServoPos);
  
  FlapsServo.write(flapsServoPos);
}

// --------------- S E T T E R S  -------------------

// Set elevator trim position
void setElevTrimPos()
{
  // TODO: detect movement, not position
  int newRawPosition = ElevTrimEncoder.read();

  if (newRawPosition != oldElevTrimEncoderPosition) // Has the wheel moved?
  {
    int newPosition = max(minElevTrimEncoderPosition, min(newRawPosition, maxElevTrimEncoderPosition));

    oldElevTrimEncoderPosition = newPosition;
    
    if (newPosition != newRawPosition) // Has the wheel moved past its limits?
    {
      ElevTrimEncoder.write(newPosition);
    }

    int elevTrimPct = int(float(newPosition) / float(maxElevTrimEncoderPosition) * 100.0);

    messenger.sendCmdStart(sElevTrimPct);
    messenger.sendCmdArg(elevTrimPct);
    messenger.sendCmdEnd();
  }
}

// Set flaps trim position
void setFlapsPos()
{
  int newPosition = !digitalRead(flapsSwitchPins[0]) * 2;
  newPosition += !digitalRead(flapsSwitchPins[1]);

  if (newPosition != oldFlapsSwitchPosition) // Has the switch moved?
  {
    oldFlapsSwitchPosition = newPosition;
    
    messenger.sendCmdStart(sFlapsPos);
    messenger.sendCmdArg(newPosition);
    messenger.sendCmdEnd();
  }
}

// --------------- S U P P O R T  -------------------

// Returns if it has been more than interval (in ms) ago. Used for periodic actions
bool hasExpired(unsigned long &prevTime, unsigned long interval)
{
  if (millis() - prevTime > interval) {
    prevTime = millis();
    return true;
  } else     
    return false;
}

// ------------------ M A I N  ----------------------

// Setup function
void setup()
{

  // 115200 is typically the maximum speed for serial over USB
  Serial.begin(115200);

  // Attach my application's user-defined callback methods
  attachCommandCallbacks();

  // initialize digital pins
  pinMode(flapsSwitchPins[0], INPUT_PULLUP);
  pinMode(flapsSwitchPins[1], INPUT_PULLUP);

  // Attach and cycle servos
  ElevTrimServo.attach(elevTrimServoPin);
  FlapsServo.attach(flapsServoPin);

  ElevTrimServo.write(60);
  // delay(250);
  FlapsServo.write(120);
  // delay(250);
  ElevTrimServo.write(120);
  // delay(250);
  FlapsServo.write(60);
  // delay(250);
  ElevTrimServo.write(90);
  // delay(250);
  FlapsServo.write(90);
  // delay(250);
}

// Loop function
void loop()
{
  // Process incoming serial data, and perform callbacks
  messenger.feedinSerialData();

  setElevTrimPos();
  setFlapsPos();
}

int clamp(int value, int min_value, int max_value) {
  return max(min(value, max_value), min_value);
}