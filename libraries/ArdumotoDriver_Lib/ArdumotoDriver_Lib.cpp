/**
 * @file ArdumotoDriver_Lib.h
 * @brief Motor device driver for the Sparkfun Ardumoto motor shield.
 * @author Dorus van der Linden
 * @original author Miguel Grinberg
 */

#include "Arduino.h"
#include "ArdumotoDriver_Lib.h"



    ArdumotoDriverClass::ArdumotoDriverClass(const int number): GenericMotorDriverClass(), _currentSpeed(0)
        {
            _number = number;
			//PWM control for motor output A is on digital pin 3
			_MotorPinsPwm[0] = 3;
			//PWM control for motor output B is on digital pin 11
			_MotorPinsPwm[1] = 11;
			//direction control for motor outputs A is on digital pin 12
			_MotorPinsDir[0] = 12;
			//direction control for motor outputs B is on digital pin 13
			_MotorPinsDir[1] = 13;
	}

    void ArdumotoDriverClass::setSpeed(const int MotorSpeed)
        {
        _currentSpeed = MotorSpeed;  
        if (MotorSpeed < 0) 
            {
	    //Move backward
            MotorBackward(abs(MotorSpeed));  // MotorPwm = absolute speed
            }
        else if (MotorSpeed == 0) 
            {
	    //Stop motor
            MotorStop();
            }
        else
            {
	    //Move forward
            MotorForward(MotorSpeed);
            }
        }


        int ArdumotoDriverClass::getSpeed() const
        {
            return _currentSpeed;
        }
        
        void ArdumotoDriverClass::MotorForward(const int MotorPwm) // pwm defined
        { 
            analogWrite(_MotorPinsPwm[_number], MotorPwm);    //Set motor to run at MotorPwm
            digitalWrite(_MotorPinsDir[_number], HIGH);       //Set motor direction
        }

        void ArdumotoDriverClass::MotorBackward(const int MotorPwm) // pwm defined
        {
            analogWrite(_MotorPinsPwm[_number], MotorPwm);    //Set motor to run at MotorPwm
            digitalWrite(_MotorPinsDir[_number], LOW);       //Set motor direction
        }

        void ArdumotoDriverClass::MotorStop() //stop
        { 
          analogWrite(_MotorPinsPwm[_number], 0);          //Set motor to run at 0% = stop
          digitalWrite(_MotorPinsDir[_number], LOW);       //Set motor direction to forward
        }
