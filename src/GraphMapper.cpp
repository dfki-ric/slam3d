#include "GraphMapper.hpp"

using namespace slam;

GraphMapper::GraphMapper(Logger* log)
{
	mSolver = NULL;
	mLogger = log;
}

GraphMapper::~GraphMapper()
{
	
}

void GraphMapper::setSolver(Solver* solver)
{
	mSolver = solver;
}

bool GraphMapper::optimize()
{
	if(!mSolver)
	{
		mLogger.message(ERROR, "A solver must be set before optimize() is called!")
		return false;
	}
	
	// Give the graph structure to the solver
	
	return true;
}

void GraphMapper::registerSensor(std::string& name, Sensor* s)
{
	std::pair<SensorList::iterator, bool> result;
	result = mSensors.insert(SensorList::value_type(name, s));
	if(!result.second)
	{
		mLogger.message(ERROR, (boost::format("Sensor with name %1% already exists!") % name).str());
		return;
	}
}

void GraphMapper::addReading(Measurement* m)
{
	// Get the odometric pose for this measurement
	Transform pose;
	mOdometry->getOdometricPose(m->getTimestamp(), pose);

	// Add the vertex to the pose graph
	VertexObject v;
	v.odometric_pose = pose;
	v.corrected_pose = pose;
	v.measurement = m;
	Vertex newVertex = mPoseGraph.addVertex(v);
	
	// Get the last added vertex from this sensor
	SensorList::iterator sensor = mSensors.find(m->getSensorName());
	if(sensor == mSensors.end())
	{
		mLogger.message(ERROR, (boost::format("Sensor with name %1% does not exist!") % m->getSensorName()).str());
		return;
	}
	timeval previous = sensor->second.getLastReading().getTimestamp();
	
	// Add an edge representing the odometry information
	EdgeObject e;
	TransformWithCovariance twc = mOdometry->getRelativePose(previous, m->getTimestamp());
	e.transform = twc.transform;
	e.covariance = twc.covariance;
	mPoseGraph.addEdge(prevVertex, newVertex, e);
}