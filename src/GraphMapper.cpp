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
	
	mLastVertices.insert(LastVertexList::value_type(s->getName(), -1));
}

void GraphMapper::addReading(Measurement* m)
{
	// Get the sensor responsible for this measurement
	// Can throw std::out_of_range if sensor is not registered
	Sensor* sensor = NULL;
	try
	{
		sensor = mSensors.at(m->getSensorName());
	}catch(std::out_of_range e)
	{
		mLogger->message(ERROR, (boost::format("Sensor '%1%' has not been registered!") % m->getSensorName()).str());
		return;
	}
	
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
	PoseGraph::IdType prevVertex = mPoseGraph.getLastVertex();
	PoseGraph::IdType newVertex = mPoseGraph.addVertex(v);
	
	// Add an edge representing the odometry information
	if(mOdometry && prevVertex < 0)
	{
		EdgeObject odomEdge;
		timeval previous = mPoseGraph.getVertex(prevVertex).measurement->getTimestamp();
		TransformWithCovariance twc = mOdometry->getRelativePose(previous, m->getTimestamp());
		odomEdge.transform = twc.transform;
		odomEdge.covariance = twc.covariance;
		mPoseGraph.addEdge(prevVertex, newVertex, odomEdge);
	}
	
	// Add an edge to the previous reading of this sensor
	LastVertexList::iterator it = mLastVertices.find(m->getSensorName());
	PoseGraph::IdType prevSensorVertex = mLastVertices.at(m->getSensorName());
	if(prevSensorVertex >= 0)
	{
		EdgeObject icpEdge;
		TransformWithCovariance twc = sensor->calculateTransform(m, mPoseGraph.getVertex(prevSensorVertex).measurement);
		icpEdge.transform = twc.transform;
		icpEdge.covariance = twc.covariance;
		mPoseGraph.addEdge(prevSensorVertex, newVertex, icpEdge);
		
		// Update current pose estimate
		mCurrentPose = twc.transform * mCurrentPose;
		mPoseGraph.setCorrectedPose(newVertex, mCurrentPose);
	}else
	{
		mLogger->message(INFO, (boost::format("Added first Reading of sensor '%1%'") % m->getSensorName()).str());
	}

	// Set last vertex for this sensor
	mLastVertices[m->getSensorName()] = newVertex;

}

VertexList GraphMapper::getVerticesFromSensor(std::string sensor)
{
	return mPoseGraph.getVerticesFromSensor(sensor);
}