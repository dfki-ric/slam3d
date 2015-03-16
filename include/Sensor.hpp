#ifndef SLAM_SENSOR_HPP
#define SLAM_SENSOR_HPP

#include "PoseGraph.hpp"

namespace slam
{
	class Sensor
	{
	public:
		Sensor(PoseGraph* g):mPoseGraph(g){}
		~Sensor(){}
	
	protected:
		void addReading(Measurement* m)
		{
			VertexObject object;
			object.measurement = m;
			mPoseGraph->addVertex()
		}
		
	protected:
		std::vector<Measurement*> mReadings;
		PoseGraph* mPoseGraph;
	};
}

#endif