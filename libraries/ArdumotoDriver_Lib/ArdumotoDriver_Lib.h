/**
 * @file: ArdumotoDriver_Lib.h
 * @brief: Motor device driver for the Sparkfun Ardumoto motor shield.
 * @description of use: Power motors with pwm between -225, 0 and 255 for backward, stop and forward
 * @author: Dorus van der Linden
 * @original author Miguel Grinberg
 */
 
#ifndef ArdumotoDriver_h
    #define ArdumotoDriver_h

    #include "Arduino.h"
    #include "GenericMotorDriver_Lib.h"

class ArdumotoDriverClass : public GenericMotorDriverClass
{
public:
	ArdumotoDriverClass(const int number);
	void setSpeed(const int MotorSpeed);
	int getSpeed() const;

private:
	int _currentSpeed;
	int _number;
	int _MotorPinsPwm[2];
	int _MotorPinsDir[2];
    void MotorForward(const int MotorPwm); // pwm defined
    void MotorBackward(const int MotorPwm); // pwm defined
    void MotorStop(); //stop
    };

#endif

