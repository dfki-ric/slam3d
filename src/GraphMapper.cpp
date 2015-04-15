#include "GraphMapper.hpp"

#include "boost/format.hpp"

using namespace slam;

GraphMapper::GraphMapper(Logger* log)
{
	mOdometry = NULL;
	mSolver = NULL;
	mLogger = log;
	mCurrentPose = Transform::Identity();
}

GraphMapper::~GraphMapper()
{
	std::ofstream file;
	file.open("pose_graph.dot");
	mPoseGraph.dumpGraphViz(file);
	file.close();
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

void GraphMapper::registerSensor(Sensor* s)
{
	std::pair<SensorList::iterator, bool> result;
	result = mSensors.insert(SensorList::value_type(s->getName(), s));
	if(!result.second)
	{
		mLogger->message(ERROR, (boost::format("Sensor with name %1% already exists!") % s->getName()).str());
		return;
	}
	
	mLastVertices.insert(LastVertexList::value_type(s->getName(), mPoseGraph.getNullVertex()));
}

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
	Vertex prevVertex = mPoseGraph.getLastVertex();
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
		mPoseGraph.addEdge(prevVertex, newVertex, odomEdge);
	}
	
	// Add an edge to the previous reading of this sensor
	Vertex prevSensorVertex = mLastVertices.at(m->getSensorName());
	if(prevSensorVertex != mPoseGraph.getNullVertex())
	{
		EdgeObject icpEdge;
		TransformWithCovariance twc = sensor->calculateTransform(m, mPoseGraph.getMeasurement(prevSensorVertex));
		icpEdge.transform = twc.transform;
		icpEdge.covariance = twc.covariance;
		mPoseGraph.addEdge(prevSensorVertex, newVertex, icpEdge);
		
		// Update current pose estimate
		mCurrentPose = twc.transform * mCurrentPose;
	}else
	{
		mLogger->message(INFO, (boost::format("Added first Reading of sensor '%1%'") % m->getSensorName()).str());
	}

	// Set last vertex for this sensor
	mLastVertices[m->getSensorName()] = newVertex;
}