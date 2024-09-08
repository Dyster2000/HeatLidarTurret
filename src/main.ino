/*

************************************************************************************
* MIT License
*
* Copyright (c) 2023 Crunchlabs LLC (IRTurret Control Code)

* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the "Software"), to deal
* in the Software without restriction, including without limitation the rights
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the Software is furnished
* to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in all
* copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
* INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
* PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
* HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF
* CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE
* OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*
************************************************************************************
*/

//////////////////////////////////////////////////
              //  LIBRARIES  //
//////////////////////////////////////////////////
#include <Arduino.h>
#include <EEPROM.h>
#include <Servo.h>
#include "PinDefinitionsAndMore.h" // Define macros for input and output pin etc.
#include <Melopero_AMG8833.h>
#include <TFLI2C.h>      // TFLuna-I2C Library v.0.1.1
#include <IRremote.hpp>
#include <time.h>


/*
** if you want to add other remotes (as long as they're on the same protocol above):
** press the desired button and look for a hex code similar to those below (ex: 0x11)
** then add a new line to #define newCmdName 0x11,
** and add a case to the switch statement like case newCmdName: 
** this will let you add new functions to buttons on other remotes!
** the best remotes to try are cheap LED remotes, some TV remotes, and some garage door openers
*/

//defines the specific command code for each button on the remote
#define left 0x8
#define right 0x5A
#define up 0x52
#define down 0x18
#define ok 0x1C
#define cmd1 0x45
#define cmd2 0x46
#define cmd3 0x47
#define cmd4 0x44
#define cmd5 0x40
#define cmd6 0x43
#define cmd7 0x7
#define cmd8 0x15
#define cmd9 0x9
#define cmd0 0x19
#define star 0x16
#define pound 0xD

#define SENSOR_ARRAY_X  8
#define SENSOR_ARRAY_Y  8
#define TEMPERATURE_OFFSET  1.0
#define TEMPERATURE_BOTTOM  20
#define TEMPERATURE_CEILING 35
#define PITCH_P 4
#define YAW_P 4
#define HOT_TEMP  28.0
#define WAVE_TIME 500
#define WAVE_SPEED 90

enum Mode
{
  Mode_Scanning,
  Mode_Tracking,
  Mode_Admin
};

struct Point
{
  int x;
  int y;
};

//////////////////////////////////////////////////
          //  PINS AND PARAMETERS  //
//////////////////////////////////////////////////
//this is where we store global variables!
Melopero_AMG8833 sensor;
TFLI2C tflI2C;

Servo yawServo; //names the servo responsible for YAW rotation, 360 spin around the base
Servo pitchServo; //names the servo responsible for PITCH rotation, up and down tilt
Servo rollServo; //names the servo responsible for ROLL rotation, spins the barrel to fire darts

bool gotIRInput = false;

//const char *adminPassword = "1234";
const char *adminPassword = "8";
int currentPasswordIndex = 0;

const int pitchHome = 100;
const int pitchMax = 120; // this sets the maximum angle of the pitch servo to prevent it from crashing, it should remain below 180, and be greater than the pitchMin
const int pitchMin = 50; // this sets the minimum angle of the pitch servo to prevent it from crashing, it should remain above 0, and be less than the pitchMax
const int yawHome = 90;
const int yawMax = 155; // this sets the maximum angle of the pitch servo to prevent it from crashing, it should remain below 180, and be greater than the pitchMin
const int yawMin = 25; // this sets the minimum angle of the pitch servo to prevent it from crashing, it should remain above 0, and be less than the pitchMax


int yawServoVal = yawHome; //initialize variables to store the current value of each servo
int pitchServoVal = pitchHome;
int rollServoVal;

const int pitchMoveSpeed = 8; //this variable is the angle added to the pitch servo to control how quickly the PITCH servo moves - try values between 3 and 10
const int yawMoveSpeed = 10; //this variable is the speed controller for the continuous movement of the YAW servo motor. It is added or subtracted from the yawStopSpeed, so 0 would mean full speed rotation in one direction, and 180 means full rotation in the other. Try values between 10 and 90;
const int rollMoveSpeed = 90; //this variable is the speed controller for the continuous movement of the ROLL servo motor. It is added or subtracted from the rollStopSpeed, so 0 would mean full speed rotation in one direction, and 180 means full rotation in the other. Keep this at 90 for best performance / highest torque from the roll motor when firing.
const int rollStopSpeed = 90; //value to stop the roll motor - keep this at 90

const int yawPrecision = 150; // this variable represents the time in milliseconds that the YAW motor will remain at it's set movement speed. Try values between 50 and 500 to start (500 milliseconds = 1/2 second)
const int rollPrecision = 165; // this variable represents the time in milliseconds that the ROLL motor with remain at it's set movement speed. If this ROLL motor is spinning more or less than 1/6th of a rotation when firing a single dart (one call of the fire(); command) you can try adjusting this value down or up slightly, but it should remain around the stock value (160ish) for best results.

bool isHot = false;
float sensorDataY[SENSOR_ARRAY_X] = {0.0};
float sensorDataX[SENSOR_ARRAY_Y] = {0.0};
float overall_max_intensity = 0;
float overall_min_intensity = 1000;
float current_max_intensity = 0;
float current_min_intensity = 1000;

static const int shootDistanceAddress = 0;
int16_t shootDistance = 500;
static const int16_t shootDistanceDefault = 500;

Mode CurrentMode  = Mode_Scanning;
bool sweepLeft = true;


//////////////////////////////////////////////////
              //  S E T U P  //
//////////////////////////////////////////////////
void setup()
{
  Serial.begin(9600); // initializes the Serial communication between the computer and the microcontroller

  // initializing I2C to use default address AMG8833_I2C_ADDRESS_B and Wire (I2C-0):
  Wire.begin();
  sensor.initI2C(AMG8833_I2C_ADDRESS_A);

  Serial.print(F("Resetting sensor ... "));
  int statusCode = sensor.resetFlagsAndSettings();
  Serial.println(sensor.getErrorDescription(statusCode));

  Serial.print(F("Setting FPS ... "));
  statusCode = sensor.setFPSMode(FPS_MODE::FPS_10);
  Serial.println(sensor.getErrorDescription(statusCode));

  yawServo.attach(10); //attach YAW servo to pin 10
  pitchServo.attach(11); //attach PITCH servo to pin 11
  rollServo.attach(12); //attach ROLL servo to pin 12

  // Start the receiver and if not 3. parameter specified, take LED_BUILTIN pin from the internal boards definition as default feedback LED
  IrReceiver.begin(9, ENABLE_LED_FEEDBACK);

  // Just to know which program is running on my microcontroller
  //Serial.println(F("START " __FILE__ " from " __DATE__ "\r\nUsing library version " VERSION_IRREMOTE));

  ReadShootDistance();

  homeServos(); //set servo motors to home position
}

////////////////////////////////////////////////
              //  L O O P  //
////////////////////////////////////////////////

void loop()
{
  int delayTime = 100;

  CheckForIRInput();
  switch (CurrentMode)
  {
    case Mode_Tracking:
      delayTime = DoTracking();
      break;

    case Mode_Admin:
      delayTime = DoAdmin();
      break;

    case Mode_Scanning:
    default:
      delayTime = DoScanning();
      break;
  }

  delay(delayTime);
}

void CheckForIRInput()
{
  gotIRInput = IrReceiver.decode();
  if (gotIRInput)
  {
    if (true) // Print errors?
    {
      // Print a short summary of received data
      IrReceiver.printIRResultShort(&Serial);
      IrReceiver.printIRSendUsage(&Serial);
      if (IrReceiver.decodedIRData.protocol == UNKNOWN) //command garbled or not recognized
      {
        Serial.println(F("Received noise or an unknown (or not yet enabled) protocol - if you wish to add this command, define it at the top of the file with the hex code printed below (ex: 0x8)"));
        // We have an unknown protocol here, print more info
        IrReceiver.printIRResultRawFormatted(&Serial, true);
      }
      Serial.println();
    }

    /*
    * !!!Important!!! Enable receiving of the next value,
    * since receiving has stopped after the end of the current received data packet.
    */
    IrReceiver.resume(); // Enable receiving of the next value
  }
}

int DoScanning()
{
  Point movement;

  if (CheckPassword())
  {
    return 10;
  }

  if (calcHeatMovement(movement))
  {
    Serial.println(F("[DoScanning] Change state to Tracking"));
    CurrentMode = Mode_Tracking;
    return 10;
  }
  else
  {
    if (sweepLeft)
    {
      if (!moveLeft())
        sweepLeft = false;
    }
    else
    {
      if (!moveRight())
        sweepLeft = true;
    }
    return 500;
  }
}

int DoTracking()
{
  Point movement;
  int delay = 100;
  
  if (CheckPassword())
  {
    return 10;
  }

  if (!calcHeatMovement(movement))
  {
    Serial.println(F("[DoTracking] Change state to Scanning"));
    CurrentMode = Mode_Scanning;
    delay = 100;
  }
  else
  {
    Serial.print(F("adjust yaw="));
    Serial.print(movement.y);
    Serial.print(F(", adjust pitch="));
    Serial.println(movement.x);

    if ((movement.x != 0) || (movement.y != 0))
    {
      // Handle yaw movement
      yawServoVal = constrain(yawServoVal + movement.y, yawMin, yawMax);
      Serial.print(F("Move yaw to angle: "));
      Serial.println(yawServoVal);
      yawServo.write(yawServoVal);

      // Handle pitch movement
      pitchServoVal = constrain(pitchServoVal + movement.x, pitchMin, pitchMax);
      Serial.print(F("Move pitch to angle: "));
      Serial.println(pitchServoVal);
      pitchServo.write(pitchServoVal);
      delay = 30;
    }
    else
    {
      int16_t dist = getDist();

      Serial.print(F("Dist: "));
      Serial.println(dist);          // print the data...
      if (dist < shootDistance)
      {
        fire(1);
        delay = 2500;
      }
      else
        delay = 10;
    }
  }
  return delay;
}

int DoAdmin()
{
  if (gotIRInput)
  {
    switch(IrReceiver.decodedIRData.command)
    {
      case cmd1: // Set shoot distance
      {
        int16_t distance = getDist();
        SetShootDistance(distance);
        break;
      }

      case cmd9: // Exit admin mode
      Serial.println(F("[DoAdmin] Change state to Scanning"));
        CurrentMode = Mode_Scanning;
        break;

      case up://pitch up
        upMove(1);
        break;
      
      case down://pitch down
        downMove(1);
        break;

      case left://fast counterclockwise rotation
        moveLeft();
        break;
      
      case right://fast clockwise rotation
        moveRight();
        break;
      
      case ok: //firing routine 
        if ((IrReceiver.decodedIRData.flags & IRDATA_FLAGS_IS_REPEAT) == 0)
          fire(1);
        //Serial.println(F("FIRE"));
        break;
    }
  }
  return 100;
}

bool CheckPassword()
{
  if (gotIRInput && (IrReceiver.decodedIRData.flags & IRDATA_FLAGS_IS_REPEAT) == 0)
  {
    uint8_t pressed = 0xFF;
    uint8_t val = adminPassword[currentPasswordIndex] - '0';

    switch (IrReceiver.decodedIRData.command)
    {
      case cmd1:
        pressed = 1;
        break;
      case cmd2:
        pressed = 2;
        break;
      case cmd3:
        pressed = 3;
        break;
      case cmd4:
        pressed = 4;
        break;
      case cmd5:
        pressed = 5;
        break;
      case cmd6:
        pressed = 6;
        break;
      case cmd7:
        pressed = 7;
        break;
      case cmd8:
        pressed = 8;
        break;
      case cmd9:
        pressed = 9;
        break;
      case cmd0:
        pressed = 0;
        break;
    }

    Serial.print(F("[CheckPassword] pressed="));
    Serial.print(pressed);
    Serial.print(F(", currentPasswordIndex="));
    Serial.print(currentPasswordIndex);
    Serial.print(F(", PW[ix]="));
    Serial.println(val);

    if (pressed == val)
    {
      Serial.println(F("[CheckPassword] PW Step Right"));
      currentPasswordIndex++;
      if (currentPasswordIndex == strlen(adminPassword))
      {
        Serial.println(F("[CheckPassword] Change state to Admin"));
        CurrentMode = Mode_Admin;
        shakeHeadYes(3);
        return true; 
      }
    }
    else
    {
      Serial.println(F("[CheckPassword] PW Wrong"));
      currentPasswordIndex = 0;
    }
  }
  return false;
}

void shakeHeadYes(int moves)
{
  Serial.println(F("YES"));
  int startAngle = pitchServoVal; // Current position of the pitch servo
  int lastAngle = pitchServoVal;
  int nodAngle = startAngle + 20; // Angle for nodding motion

  for (int i = 0; i < moves; i++) // Repeat nodding motion three times
  {
    // Nod up
    for (int angle = startAngle; angle <= nodAngle; angle++)
    {
      pitchServo.write(angle);
      delay(7); // Adjust delay for smoother motion
    }
    delay(50); // Pause at nodding position
    // Nod down
    for (int angle = nodAngle; angle >= startAngle; angle--)
    {
      pitchServo.write(angle);
      delay(7); // Adjust delay for smoother motion
    }
    delay(50); // Pause at starting position
  }
}

void shakeHeadNo(int moves)
{
  Serial.println(F("NO"));
  int startAngle = pitchServoVal; // Current position of the pitch servo
  int lastAngle = pitchServoVal;
  int nodAngle = startAngle + 20; // Angle for nodding motion

  for (int i = 0; i < moves; i++)  // Repeat nodding motion three times
  {
    // Nod up
    for (int angle = startAngle; angle <= nodAngle; angle++)
    {
      pitchServo.write(angle);
      delay(7); // Adjust delay for smoother motion
    }
    delay(50); // Pause at nodding position
    // Nod down
    for (int angle = nodAngle; angle >= startAngle; angle--)
    {
      pitchServo.write(angle);
      delay(7); // Adjust delay for smoother motion
    }
    delay(50); // Pause at starting position
  }
}

bool moveLeft()
{
  if(yawServoVal < yawMax)
  {
    yawServoVal += yawMoveSpeed; //decrement the current angle and update
    yawServo.write(yawServoVal);
    //***Serial.print(F("LEFT TO "));
    //***Serial.println(yawServoVal);
    return true;
  }
  else
    return false;
}

bool moveRight()
{
  if(yawServoVal > yawMin)
  {
    yawServoVal -= yawMoveSpeed; //decrement the current angle and update
    yawServo.write(yawServoVal);
    //***Serial.print(F("RIGHT TO "));
    //***Serial.println(yawServoVal);
    return true;
  }
  else
    return false;
}

void upMove(int moves)
{
  for (int i = 0; i < moves; i++)
  {
    if(pitchServoVal > pitchMin)//make sure the servo is within rotation limits (greater than 10 degrees by default)
    {
      pitchServoVal = pitchServoVal - pitchMoveSpeed; //decrement the current angle and update
      pitchServo.write(pitchServoVal);
      delay(50);
      Serial.println(F("UP"));
    }
  }
}

void downMove (int moves)
{
  for (int i = 0; i < moves; i++)
  {
    if(pitchServoVal < pitchMax) //make sure the servo is within rotation limits (less than 175 degrees by default)
    {
      pitchServoVal = pitchServoVal + pitchMoveSpeed;//increment the current angle and update
      pitchServo.write(pitchServoVal);
      delay(50);
      Serial.println(F("DOWN"));
    }
  }
}

/**
 * a single dart
 */
void fire(int num)
{
  rollServo.write(rollStopSpeed + rollMoveSpeed);//start rotating the servo
  delay(rollPrecision * num);//time for approximately 60 degrees of rotation
  rollServo.write(rollStopSpeed);//stop rotating the servo
  delay(5); //delay for smoothness
  Serial.println(F("FIRING"));
}

void fireAll() //function to fire all 6 darts at once
{
  rollServo.write(rollStopSpeed + rollMoveSpeed);//start rotating the servo
  delay(rollPrecision * 6); //time for 360 degrees of rotation
  rollServo.write(rollStopSpeed);//stop rotating the servo
  delay(5); // delay for smoothness
  Serial.println(F("FIRING ALL"));
}

void homeServos()
{
  yawServoVal = yawHome;
  yawServo.write(yawServoVal);
  pitchServoVal = pitchHome; // store the pitch servo value
  pitchServo.write(pitchServoVal); //set PITCH servo to 100 degree position
  delay(100);
  Serial.println(F("HOMING"));
}
   
bool calcHeatMovement(Point &movementToTarget)
{
  bool isHot;

  movementToTarget.x = 0;
  movementToTarget.y = 0;

  int statusCode = sensor.updateThermistorTemperature();
  if (statusCode < 0)
  {
    Serial.print(F("updateThermistorTemperature="));
    Serial.println(sensor.getErrorDescription(statusCode));
  }
  statusCode = sensor.updatePixelMatrix();
  if (statusCode < 0)
  {
    Serial.print(F("updatePixelMatrix="));
    Serial.println(sensor.getErrorDescription(statusCode));
  }
  // NOTE: Temp intensity appears to range between about 20-40

  float max_intensity = 0;
  float min_intensity = 1000;
  float movementAmount[SENSOR_ARRAY_X] = { -4, -3, -2, -1, 1, 2, 3, 4 };

  for (uint8_t x = 0; x < SENSOR_ARRAY_X; x++)
    sensorDataY[x] = 0.0;
  for (uint8_t y = 0; y < SENSOR_ARRAY_Y; y++)
    sensorDataX[y] = 0.0;

  // loop through pixels to find min/max value
  for (uint8_t x = 0; x < SENSOR_ARRAY_X; x++)
  {
    for (uint8_t y = 0; y < SENSOR_ARRAY_Y; y++)
    {
      float pixel_intensity = sensor.pixelMatrix[y][x];
      if (pixel_intensity < min_intensity)
        min_intensity = pixel_intensity;
      else if (pixel_intensity > max_intensity)
        max_intensity = pixel_intensity;
    }
  }

  //--set the HOT flag if any of the pixels are warm enough to be a body--
  if(max_intensity < HOT_TEMP)
    isHot = false;
  else
  {
    isHot = true;

    // loop through pixels to find min/max value
    for (uint8_t x = 0; x < SENSOR_ARRAY_X; x++)
    {
      for (uint8_t y = 0; y < SENSOR_ARRAY_Y; y++)
      {
        float pixel_intensity = sensor.pixelMatrix[y][x];
        if (pixel_intensity < TEMPERATURE_BOTTOM)
          pixel_intensity = TEMPERATURE_BOTTOM;
        else if (pixel_intensity >= TEMPERATURE_CEILING)
          pixel_intensity = TEMPERATURE_CEILING;
        pixel_intensity -= min_intensity;
        sensorDataY[x] += pixel_intensity;
        sensorDataX[y] += pixel_intensity;
      }
    }

    for (uint8_t x = 0; x < SENSOR_ARRAY_X; x++)
      sensorDataY[x] /= SENSOR_ARRAY_Y;
    for (uint8_t y = 0; y < SENSOR_ARRAY_Y; y++)
      sensorDataX[y] /= SENSOR_ARRAY_X;

    // Calc yaw
    float movement = 0.0;
    for (uint8_t x = 0; x < SENSOR_ARRAY_X; x++)
      movement += movementAmount[x] * sensorDataY[x];

    float movementDeg = movement / 15;
    if (abs(movementDeg) < 1.5)
      movementDeg = 0;
    movementToTarget.y = movementDeg;

    // Calc pitch
    movement = 0;
    movementDeg = 0;
    for (uint8_t y = 0; y < SENSOR_ARRAY_Y; y++)
      movement += movementAmount[y] * sensorDataX[y];

    movementDeg = movement / 5;
    if (abs(movementDeg) < 1.5)
      movementDeg = 0;
    movementToTarget.x = movementDeg;
  }

  return isHot;
}

void testHeatOutput()
{
  int statusCode = sensor.updateThermistorTemperature();
  if (statusCode < 0)
  {
    Serial.print(F("updateThermistorTemperature="));
    Serial.println(sensor.getErrorDescription(statusCode));
  }
  statusCode = sensor.updatePixelMatrix();
  if (statusCode < 0)
  {
    Serial.print(F("updatePixelMatrix="));
    Serial.println(sensor.getErrorDescription(statusCode));
  }
  // Temp intensity appears to range between about 20-40

  //pin_times: X0Y0, X0Y1, X1Y0, X1Y1

  float max_intensity = 0;
  float min_intensity = 1000;

  for (uint8_t x = 0; x < SENSOR_ARRAY_X; x++)
    sensorDataY[x] = 0.0;
  for (uint8_t y = 0; y < SENSOR_ARRAY_Y; y++)
    sensorDataX[y] = 0.0;

  // loop through pixels to find min/max value
  for (uint8_t x = 0; x < SENSOR_ARRAY_X; x++)
  {
    for (uint8_t y = 0; y < SENSOR_ARRAY_Y; y++)
    {
      float pixel_intensity = sensor.pixelMatrix[y][x];
      if (pixel_intensity < min_intensity)
        min_intensity = pixel_intensity;
    }
  }

  // loop through pixels to find min/max value
  for (uint8_t x = 0; x < SENSOR_ARRAY_X; x++)
  {
    for (uint8_t y = 0; y < SENSOR_ARRAY_Y; y++)
    {
      float pixel_intensity = sensor.pixelMatrix[y][x];
      if (pixel_intensity < TEMPERATURE_BOTTOM)
        pixel_intensity = TEMPERATURE_BOTTOM;
      else if (pixel_intensity >= TEMPERATURE_CEILING)
        pixel_intensity = TEMPERATURE_CEILING;
      pixel_intensity -= min_intensity;
      sensorDataY[x] += pixel_intensity;
      sensorDataX[y] += pixel_intensity;
    }
  }

  for (uint8_t x = 0; x < SENSOR_ARRAY_X; x++)
    sensorDataY[x] /= SENSOR_ARRAY_Y;
  for (uint8_t y = 0; y < SENSOR_ARRAY_Y; y++)
    sensorDataX[y] /= SENSOR_ARRAY_X;
}

int16_t getDist()
{
  int16_t  tfDist;    // distance in centimeters

  if (tflI2C.getData(tfDist, TFL_DEF_ADR)) // If read okay...
  {
    //Serial.print(F("Dist: "));
    //Serial.println(tfDist);          // print the data...
  }
  else
  {
    tfDist = 0;
    tflI2C.printStatus();           // else, print error.
    Serial.println(F(""));
  }

  return tfDist;
}

void ReadShootDistance()
{
  uint8_t low = EEPROM.read(shootDistanceAddress);
  uint8_t high = EEPROM.read(shootDistanceAddress + 1);

  shootDistance = (high << 8) | low;
  if (shootDistance == 0)
    shootDistance = shootDistanceDefault;
  Serial.print(F("LoadDist="));
  Serial.println(shootDistance);
}

void SetShootDistance(int16_t distance)
{
  shootDistance = distance;

  Serial.print(F("SaveDist="));
  Serial.println(shootDistance);

  uint8_t low = shootDistance & 0xFF;
  uint8_t high = (shootDistance >> 8) & 0xFF;

  EEPROM.write(shootDistanceAddress, low);
  EEPROM.write(shootDistanceAddress + 1, high);
}
