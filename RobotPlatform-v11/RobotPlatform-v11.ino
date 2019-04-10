// --- Robot Specification -----------------------------------------
//
// Left = B, Right = A
// 
// Pin Mapping
//                          13 Direction Motor B - Out
//                          12 Direction Motor A - Out
//                          11 PWM Motor B - Out
//                          10 RC-Channel 6 - In / Bumper B
//                           9 Speed B - In
//                           8 Speed A - In
//                           7 RC-Channel 4 - In
//                           6 RC-Channel 3 - In
//   A0   Distance IR - In   5 RC-Channel 2 - In     
//   A1   Trigger - Out      4 RC-Channel 1 - In
//   A2   Distance Echo - In 3 PWM Motor A - Out
//   A3                      2 Bumper A - In
//   A4                      1 Bumper B - In
//   A5                      0
//

// --- Libraries ---------------------------------------------------
//
#include <DistanceSensor_Lib.h>
#include <ArdumotoDriver_Lib.h>
#include <NewPing.h>

// --- Constants ---------------------------------------------------
//

// --- Debugging ---
#define DEBUGGING 0
#define DEBUGGING2 0 

#define FREEDRIVE 1
 
// --- NewPing ---
#define TRIGGER_PIN  A1  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN     A2  // Arduino pin tied to echo pin on the ultrasonic sensor.
#define MAX_DISTANCE 400 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.

 
// --- Motor Board ---
//     Fixed by the Ardumoto Board
// PWM control for motor output A is on digital pin 3
// PWM control for motor output B is on digital pin 11
// Direction control for motor outputs A is on digital pin 12
// Direction control for motor outputs B is on digital pin 13
#define RIGHT_MOTOR_INIT 0    // Motor A = 0
#define LEFT_MOTOR_INIT 1     // Motor B = 1

// --- Pin Constants ---
//     Distance sensor
#define SENSORPIN A0
#define CLOSEENOUGH 30
#define CLOSEENOUGH2 50
//     Bumper sensors
#define SENSORPINLEFT 10 // or 1
#define SENSORPINRIGHT 2
//     Channels
#define Ch1Pin 4
#define Ch2Pin 5
#define Ch3Pin 6
#define Ch4Pin 7
#define Ch5Pin A5 // is realy 10 // to-do: add in comment above

// --- Speed Constants ---
#define NORMALSPEED 200
#define TURNRIGHT -250
#define TURNLEFT 250
#define LEFTSPEEDFACTOR 1.02

// --- Timer Constants ---
#define NORMALDELAY 100
#define TURNDELAY 200

// --- Variables ---------------------------------------------------
//
int ch1; // Here's where we'll keep our channel values
int ch2;
int ch3;
int ch4;
int ch5;
int Speed; // Forward/Back speed
int Turn; // Turning Factor
boolean LeftHit;
boolean RightHit;
boolean ToClose;
boolean ToClose2;
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); // NewPing setup of pins and maximum distance.


// --- Create Instances --------------------------------------------
//
// Motors
ArdumotoDriverClass MotorLeft(LEFT_MOTOR_INIT);
ArdumotoDriverClass MotorRight(RIGHT_MOTOR_INIT);

// sensors
DistanceSensorClass Sensor1(SENSORPIN);

// --- Functions ---------------------------------------------------
//
void SetDriveSpeeds(int DriveSpeed, int DriveTurn)
{
  int TurnFactor = 10; // out of 10
  MotorLeft.setSpeed(DriveSpeed+(DriveTurn*TurnFactor/10));
  MotorRight.setSpeed(DriveSpeed-(DriveTurn*TurnFactor/10));
}

// --- Initiallization ---------------------------------------------
//
void setup()
{
  Serial.begin(9600);
  pinMode(SENSORPINLEFT, INPUT_PULLUP);    
  pinMode(SENSORPINRIGHT, INPUT_PULLUP);   
  pinMode(Ch1Pin, INPUT); // Set our RC  input pins as such
  pinMode(Ch2Pin, INPUT);
  pinMode(Ch3Pin, INPUT);
  pinMode(Ch4Pin, INPUT);
  pinMode(Ch5Pin, INPUT);
}

// --- Main Loop ---------------------------------------------------
//
void loop()
{
  if (DEBUGGING == 1)
  {
    // Do Not Move
    Serial.print("Tests 1: IR: ");
    Serial.print(Sensor1.getDistanceRaw());
    Serial.print("-");
    Serial.print(Sensor1.getDistanceVolt());
    Serial.print("-");
    Serial.print(Sensor1.getDistanceCentimeter());
    Serial.print(", L-R: ");
    Serial.print(digitalRead(SENSORPINLEFT));
    Serial.print("-");
    Serial.print(digitalRead(SENSORPINRIGHT));
    Serial.print(", Sonar: ");
    Serial.print(sonar.ping()/ US_ROUNDTRIP_CM);
      
    Serial.print("\n");
    delay(300);
  }
  else
  {
    // Move
    ch5 = pulseIn(Ch5Pin, HIGH, 25000);
    if (FREEDRIVE==1) { ch5 = 1600;}
    if (ch5 < 1000)
    {
      SetDriveSpeeds(0, 0);
    }
    if ((ch5 > 1000) && (ch5 < 1500))
    {  
      // Move on Remote Control
      ch1 = pulseIn(Ch1Pin, HIGH, 25000); // Read the pulse width of 
      ch2 = pulseIn(Ch2Pin, HIGH, 25000); // each channel
      ch3 = pulseIn(Ch3Pin, HIGH, 25000);
      ch4 = pulseIn(Ch4Pin, HIGH, 25000);
      Speed = map(ch3, 800,1800, -500, 500);  // Center over zero
      Speed = constrain(Speed, -255, 255);    // Rescale, perhaps replace with map
      Turn = map(ch2,800,1800,-500,500);
      Turn = constrain(Turn, -255, 255);
      if (Speed > -40 && Speed < 40)
      {
        Speed = 0;
      }
      SetDriveSpeeds(Speed, -Turn);
    } 
    else if (ch5 > 1500)
    {  
      // Run Free
      // While no hit, run
      unsigned int uS = sonar.ping(); // Send ping, get ping time in microseconds (uS).
      ToClose2 = (uS / US_ROUNDTRIP_CM) <= CLOSEENOUGH2; // Convert ping time to distance in cm and print result (0 = outside set distance range)
      LeftHit = ! digitalRead(SENSORPINLEFT);
      RightHit = ! digitalRead(SENSORPINRIGHT);
      ToClose = Sensor1.getDistanceCentimeter() <= CLOSEENOUGH;

      if (DEBUGGING2 == 1)
      {
      ToClose = false;

        Serial.print("tests 2: ");
        Serial.print(LeftHit);
        Serial.print("-");
        Serial.print(LeftHit);
        Serial.print("-");
        Serial.print(ToClose);
        Serial.print("-");
        Serial.print(ToClose2);
        Serial.print("\n");
        delay(300);

      }

      if (!(LeftHit || RightHit || ToClose||ToClose2))
      {
        
        SetDriveSpeeds(NORMALSPEED, 0);
        delay(NORMALDELAY);
      }
      else
      {
        if (LeftHit)
        {
          SetDriveSpeeds(-NORMALSPEED, 0);
          delay(TURNDELAY);
          SetDriveSpeeds(-NORMALSPEED, TURNRIGHT);
        }
        else if (RightHit)
        {
          SetDriveSpeeds(-NORMALSPEED, 0);
          delay(TURNDELAY);
          SetDriveSpeeds(-NORMALSPEED, TURNLEFT);
        }
        else
        {
          SetDriveSpeeds(0, TURNLEFT);
        }
        delay(NORMALDELAY);
        SetDriveSpeeds(0, 0);
      }
    }
  }
}
