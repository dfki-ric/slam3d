#include "GraphMapper.hpp"

using namespace slam;

GraphMapper::GraphMapper(Logger* log)
{
	mOdometry = NULL;
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

void GraphMapper::setOdometry(Odometry* odom)
{
	mOdometry = odom;
}

bool GraphMapper::optimize()
{
	if(!mSolver)
	{
		mLogger->message(ERROR, "A solver must be set before optimize() is called!");
		return false;
	}
	
	// Give the graph structure to the solver
	mPoseGraph.optimize(mSolver);
	return true;
}
/*
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
*/
void GraphMapper::addReading(Measurement* m)
{
	// Get the sensor responsible for this measurement
	// Can throw std::out_of_range if sensor is not registered
	Sensor* sensor = mSensors.at(m->getSensorName());
	
	// Get the odometric pose for this measurement
	Transform pose = Transform::Identity();
	if(mOdometry)
	{
		pose = mOdometry->getOdometricPose(m->getTimestamp());
	}

	// Add the vertex to the pose graph
	VertexObject v;
	v.odometric_pose = pose;
	v.corrected_pose = pose;
	v.measurement = m;
	Vertex newVertex = mPoseGraph.addVertex(v);
	
	// Get the last added vertex 
	timeval previous = mPoseGraph.getLastVertexObject().measurement->getTimestamp();
	
	// Add an edge representing the odometry information
	if(mOdometry)
	{
		EdgeObject odomEdge;
		TransformWithCovariance twc = mOdometry->getRelativePose(previous, m->getTimestamp());
		odomEdge.transform = twc.transform;
		odomEdge.covariance = twc.covariance;
//		mPoseGraph.addEdge(prevVertex, newVertex, odomEdge);
	}
	
	// Add an edge to the previous reading of this sensor
/*	EdgeObject icpEdge;
	TransformWithCovariance twc = sensor->calculateTransform(m, <LAST_FROM_THIS_SENSOR>);
	icpEdge.transform = twc.transform;
	icpEdge.covariance = twc.covariance;
	mPoseGraph.addEdge(, newVertex, icpEdge);
*/
}