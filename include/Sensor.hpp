#ifndef SLAM_SENSOR_HPP
#define SLAM_SENSOR_HPP

#include "GraphMapper.hpp"

namespace slam
{
	typedef std::vector<Measurement*> MeasurementList;
	
	class Sensor
	{
	public:
		Sensor(GraphMapper* m):mMapper(m){}
		~Sensor(){}
	
	protected:
		void addReading(Measurement* m)
		{
			mReadings.push_back(m);
			mMapper->addReading(m);
		}
		
		MeasurementList getAllReadings() { return mReadings; }
		Measurement getLastReading() { return mReadings.at(mReadings.size()-1); }
		
	protected:
		GraphMapper* mMapper;
		MeasurementList mReadings;
	};
}

#endif