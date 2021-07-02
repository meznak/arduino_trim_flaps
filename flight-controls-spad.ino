// *** SNDEMO ***

// This example shows how to autoconnect between the SPAD.neXt and Arduino.
//
// SPAD.neXt >= 0.9.7.5 required
//
// It demonstrates how to
// - Respond to a connection request from SPAD.neXt
// - Use a identifier to handshake
// - Expose a data value to SPAD.neXt
// - Request Data Updates from SPAD.neXt

#include <CmdMessenger.h>  // CmdMessenger
#include <Encoder.h>
#include <Servo.h>

// Pinout
const int ledPin = LED_BUILTIN;
const int elevTrimEncoderPins[] = {2, 3};
const int elevTrimServoPin = 4;
const int flapsSwitchPins[] = {8, 9};
const int flapsServoPin = 10;

// Timers
unsigned long previousUpdate = 0;

// Listen on serial connection for messages from the pc
CmdMessenger messenger(Serial);

// This is the list of recognized commands. These can be commands that can either be sent
// or received.
// In order to receive, attach a callback function to these events
enum
{
  kRequest = 0, // Request from SPAD.neXt
  kCommand = 1, // Command to SPAD.neXt
  kEvent = 2, // Events from SPAD.neXt
  kDebug = 3, // Debug strings to SPAD.neXt Logfile
  kSimCommand = 4, // Send Event to Simulation
  kLed = 10, // CMDID for exposed data to SPAD.neXt
  kHeading = 11, // CMDID for data updates from SPAD.neXt
  kElevTrimPct = 12,
  kElevTrimInd = 13,
  kFlapsPos = 14,
  kFlapsInd = 15,
};

int lastLedState = 999;
bool isReady = false;

// Elevator trim wheel encoder variables
Encoder ElevTrimEncoder(elevTrimEncoderPins[0], elevTrimEncoderPins[1]);
int oldElevTrimEncoderPosition = -9999;
int maxElevTrimEncoderPosition = 50;
int minElevTrimEncoderPosition = -maxElevTrimEncoderPosition;

// Elevator trim indicator servo variables
Servo ElevTrimServo;
const int maxElevTrimIndicatorDeflection = 30;

// Flaps switch variables
uint8_t oldFlapsSwitchPosition  = -99;

// Flaps indicator servo variables
Servo FlapsServo;
const int maxFlapsIndicatorDeflection = 60;

void attachCommandCallbacks()
{
  // Attach callback methods
  messenger.attach(onUnknownCommand);
  messenger.attach(kRequest , onIdentifyRequest);
//  messenger.attach(kLed, onTurnLedOn);
//  messenger.attach(kElevTrimPos , onElevTrimPos);
  messenger.attach(kElevTrimInd , onElevTrimInd);
  messenger.attach(kFlapsInd , onFlapsInd);
}

// ------------------  C A L L B A C K S -----------------------

// Called when a received command has no attached function
void onUnknownCommand()
{
  messenger.sendCmd(kDebug,"UNKNOWN COMMAND"); 
}

// Callback function to respond to indentify request. This is part of the
// Auto connection handshake.
void onIdentifyRequest()
{
  char *szRequest = messenger.readStringArg();

  if (strcmp(szRequest, "INIT") == 0) {
    messenger.sendCmdStart(kRequest);
    messenger.sendCmdArg("SPAD");
    // Unique Device ID: Change this!
    messenger.sendCmdArg(F("{f392d590-a755-413b-8daf-333d4afa58bd}"));
    // Device Name for UI
    messenger.sendCmdArg("Arduino Demo");
    messenger.sendCmdEnd();
  }

  else if (strcmp(szRequest, "PING") == 0) {
    messenger.sendCmdStart(kRequest);
    messenger.sendCmdArg("PONG");
    messenger.sendCmdArg(messenger.readInt32Arg());
    messenger.sendCmdEnd();
  }
  
  else if (strcmp(szRequest, "CONFIG") == 0) {

    // Expose LED
    messenger.sendCmdStart(kCommand);
    messenger.sendCmdArg("ADD");
    messenger.sendCmdArg(kLed);
    messenger.sendCmdArg("leds/systemled"); // will become "SERIAL:<guid>/leds/systemled"
    messenger.sendCmdArg("U8");
    messenger.sendCmdArg("RW");
    messenger.sendCmdArg("Led");
    messenger.sendCmdArg("Toggle LED on/off");
    messenger.sendCmdEnd();

    // Expose Elevator Trim Wheel
    messenger.sendCmdStart(kCommand);
    messenger.sendCmdArg("SUBSCRIBE");
    messenger.sendCmdArg(kElevTrimPct);
    messenger.sendCmdArg("SIMCONNECT:ELEVATOR TRIM PCT");
    messenger.sendCmdEnd();
    
    // Request Elevator Trim Updates
    messenger.sendCmdStart(kCommand);
    messenger.sendCmdArg("SUBSCRIBE");
    messenger.sendCmdArg(kElevTrimInd);
    messenger.sendCmdArg("SIMCONNECT:ELEVATOR TRIM INDICATOR");
    messenger.sendCmdEnd();
    
    // Expose Flaps Switch
    messenger.sendCmdStart(kCommand);
    messenger.sendCmdArg("SUBSCRIBE");
    messenger.sendCmdArg(kFlapsPos);
    messenger.sendCmdArg("SIMCONNECT:FLAPS HANDLE INDEX");
    messenger.sendCmdEnd();
    
    // Request Flaps Updates
    messenger.sendCmdStart(kCommand);
    messenger.sendCmdArg("SUBSCRIBE");
    messenger.sendCmdArg(kFlapsInd);
    messenger.sendCmdArg("SIMCONNECT:TRAILING EDGE FLAPS LEFT PERCENT");
    messenger.sendCmdEnd();

    // tell SPAD.neXT we are done with config
    messenger.sendCmd(kRequest, "CONFIG");
    isReady = true;

    // Make sure led is turned off and SPAD.neXt gets the value
    setLED(LOW);
  }

  else if (strcmp(szRequest, "START") == 0) {
    oldElevTrimEncoderPosition  = -9999;
    ElevTrimEncoder.write(0);
  }
  
  return;
}

// Update elevator trim indicator
void onElevTrimInd()
{
  // TODO: Change this to int
  float newPosition = messenger.readFloatArg();

  int elevTrimServoPos = int(90 + newPosition * maxElevTrimIndicatorDeflection);
  
  ElevTrimServo.write(elevTrimServoPos);
}

void onFlapsInd()
{
  int newPosition = messenger.readFloatArg() * 100;
  
  int flapsServoPos = 120 - (newPosition * maxFlapsIndicatorDeflection) / 100;

  messenger.sendCmd(kDebug, newPosition);
  messenger.sendCmd(kDebug, flapsServoPos);
  
  FlapsServo.write(flapsServoPos);
}

// --------------- S E T T E R S  -------------------

// Set elevator trim position
void setElevTrimPos()
{
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

    messenger.sendCmdStart(kElevTrimPct);
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
    
    messenger.sendCmdStart(kFlapsPos);
    messenger.sendCmdArg(newPosition);
    messenger.sendCmdEnd();
  }
}

void setNav1Freq()
{
//  int newRawPosition = Nav1FreqEncoder.read();
//
//  if (newRawPosition != oldNav1FreqEncoderPosition) // Has the wheel moved?
//  {
//    int newPosition = max(minNav1FreqEncoderPosition, min(newRawPosition, maxNav1FreqEncoderPosition));
//
//    oldNav1FreqEncoderPosition = newPosition;
//    
//    if (newPosition != newRawPosition) // Has the wheel moved past its limits?
//    {
//      Nav1FreqTrimEncoder.write(newPosition);
//    }
//
//    int val = map(newPosition, minNav1FreqEncoderPosition, maxNav1FreqEncoderPosition, minNav1FreqPosition, maxNav1FreqPosition);
//    float scaled_val = float(val) / float(float_int_scalar);
//
//    messenger.sendCmd(kDebug, newRawPosition);
//    messenger.sendCmd(kDebug, newPosition);
//    messenger.sendCmd(kDebug, val);
//    messenger.sendCmd(kDebug, scaled_val);
//
//    messenger.sendCmdStart(kNav1FreqPos);
//    messenger.sendCmdArg(scaled_val);
//    messenger.sendCmdEnd();
//  }
}

void setNav2Freq()
{
  return;
//  int newRawPosition = Nav2FreqEncoder.read();
//
//  if (newRawPosition != oldNav2FreqEncoderPosition) // Has the wheel moved?
//  {
//    int newPosition = max(minNav2FreqEncoderPosition, min(newRawPosition, maxNav2FreqEncoderPosition));
//
//    oldNav2FreqEncoderPosition = newPosition;
//    
//    if (newPosition != newRawPosition) // Has the wheel moved past its limits?
//    {
//      Nav2FreqTrimEncoder.write(newPosition);
//    }
//
//    int val = map(newPosition, minNav2FreqEncoderPosition, maxNav2FreqEncoderPosition, minNav2FreqPosition, maxNav2FreqPosition);
//    float scaled_val = float(val) / float(float_int_scalar);
//
//    messenger.sendCmd(kDebug, newRawPosition);
//    messenger.sendCmd(kDebug, newPosition);
//    messenger.sendCmd(kDebug, val);
//    messenger.sendCmd(kDebug, scaled_val);
//
//    messenger.sendCmdStart(kNav2FreqPos);
//    messenger.sendCmdArg(scaled_val);
//    messenger.sendCmdEnd();
//  }
}
  
// Update system LED and post state back to SPAD.neXt
void setLED(int ledVal)
{
  digitalWrite(ledPin,ledVal);
  // Update Led-Data on SPAD.neXt
  if ((ledVal != lastLedState) && isReady)
  {
    lastLedState = ledVal;
    messenger.sendCmd(kLed,ledVal);
  }
}

// --------------- S U P P O R T  -------------------

// Returns if it has been more than interval (in ms) ago. Used for periodic actions
bool hasExpired(unsigned long &prevTime, unsigned long interval)
{
  if (  millis() - prevTime > interval ) {
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
  pinMode(ledPin, OUTPUT);
  pinMode(flapsSwitchPins[0], INPUT_PULLUP);
  pinMode(flapsSwitchPins[1], INPUT_PULLUP);

  // Turn LED on, will be turned off when connection to SPAD.neXt it up and running
  setLED(HIGH);

  // Attach and cycle servos
  ElevTrimServo.attach(elevTrimServoPin);
  FlapsServo.attach(flapsServoPin);

  ElevTrimServo.write(60);
  delay(250);
  FlapsServo.write(120);
  delay(250);
  ElevTrimServo.write(120);
  delay(250);
  FlapsServo.write(60);
  delay(250);
  ElevTrimServo.write(90);
  delay(250);
  FlapsServo.write(90);
  delay(250);
}

// Loop function
void loop()
{
  // Process incoming serial data, and perform callbacks
  messenger.feedinSerialData();

  setElevTrimPos();
  setFlapsPos();
//  setNav1Freq();
//  setNav2Freq();
}
