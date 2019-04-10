// Libraries
#include <DistanceSensor_Lib.h>

// motors
DistanceSensorClass Sensor1(13);

void setup()
{
    Serial.begin(9600);
    Sensor1.getDistanceCentimeter();
}

void loop()
{

}
