/*
comment header
 */

#include <Arduino.h>
#include <DistanceSensor_Lib.h>

#define SensorRefVoltage 5 
// or 3 for 3.3V: put a wire between the AREF pin and the 3.3V VCC pin
//This increases accuracy (and uses a different function)


// Constructor
DistanceSensorClass::DistanceSensorClass(int distancePin)
{
	pinMode(distancePin, INPUT);
	_distancePin=distancePin;
	setAveraging(100);		      
	setARefVoltage(SensorRefVoltage);	
}


// setAveraging(int avg): 
void DistanceSensorClass::setAveraging(int avg)
{
	_average=avg;
}


// getDistanceRaw(): Returns the distance as a raw value: ADC output: 0 -> 1023
int DistanceSensorClass::getDistanceRaw()
{
	return (analogRead(_distancePin));
}


// getDistanceVolt(): Returns the distance as a Voltage: ADC Input: 0V -> 5V (or 0V -> 3.3V)
float DistanceSensorClass::getDistanceVolt()
{
	if (_SensorRefVoltage ==3)
	{
          return ((float)getDistanceRaw()*3.3/1023.0) ;
	}
	else
	{
          return ((float)getDistanceRaw()*5.0/1023.0) ;
	}
}


// getDistanceCentimeter(): Returns the distance in centimeters
int DistanceSensorClass::getDistanceCentimeter()
{
	if (_SensorRefVoltage ==3)
	{
          return 65*pow(getDistanceVolt(), -1.10); // not correct yet
	}
	else
	{
          return 25*pow(getDistanceVolt(), -1.10);
	}

}



// setARefVoltage:set the ADC reference voltage: (default value: 5V, set to 3 for 3.3V)
void DistanceSensorClass::setARefVoltage(int refV)
{
	_SensorRefVoltage =refV;
	if (_SensorRefVoltage ==3)
	{
		analogReference(EXTERNAL);
	}
}

