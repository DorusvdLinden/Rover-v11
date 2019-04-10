/*
comment header
 */

#ifndef DistanceSensor_Lib_h
#define DistanceSensor_Lib_h
#include <Arduino.h>

class DistanceSensorClass
{
	public:
		DistanceSensorClass(int distancePin);

		int getDistanceRaw();
		float getDistanceVolt();
		int getDistanceCentimeter();

		void setAveraging(int avg);    
		void setARefVoltage(int _refV);

	private:
		int _distancePin;
		int _average; // number of samples to be averaged in getDistanceCentimeter, default value is 100.
		int _SensorRefVoltage;
};
#endif