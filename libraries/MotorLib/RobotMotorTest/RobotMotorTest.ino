
/**
 * @file TestRobot.ino
 * @brief Arduino robot vehicle firmware.
 * @author Dorus van der Linden
 */

// Libraries
#include <ArdumotoDriver_Lib.h>

// constants
#define LEFT_MOTOR_INIT 0
#define RIGHT_MOTOR_INIT 1

// motors
ArdumotoDriverClass MotorLeft(LEFT_MOTOR_INIT);
ArdumotoDriverClass MotorRight(RIGHT_MOTOR_INIT);

void setup()
{
    Serial.begin(9600);
    MotorLeft.setSpeed(255);
    MotorRight.setSpeed(255);
    delay(1000);
    MotorLeft.setSpeed(-255);
    MotorRight.setSpeed(-255);
    delay(1000);
    MotorLeft.setSpeed(0);
    MotorRight.setSpeed(0);
}

void loop()
{

}
