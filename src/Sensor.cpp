#include "Sensor.hpp"
#include "GraphMapper.hpp"

using namespace slam;

void Sensor::addReading(Measurement* m)
{
	mReadings.push_back(m);
	mMapper->addReading(m);
}

MeasurementList Sensor::getAllReadings()
{
	return mReadings;
}

Measurement* Sensor::getLastReading()
{
	return mReadings[mReadings.size()-1];
}