#include "GraphMapper.hpp"

#include "boost/format.hpp"
#include <graph_analysis/lemon/DirectedGraph.hpp>
#include <graph_analysis/GraphIO.hpp>

using namespace slam;

GraphMapper::GraphMapper(Logger* log)
    : mPoseGraph( new graph_analysis::lemon::DirectedGraph())
{
	mOdometry = NULL;
	mSolver = NULL;
	mLogger = log;
	mCurrentPose = Transform::Identity();
}

GraphMapper::~GraphMapper()
{
	std::string file = "pose_graph.dot";
	graph_analysis::io::GraphIO::write(file, *mPoseGraph, graph_analysis::representation::GEXF);
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
	//mPoseGraph.optimize(mSolver);
        mSolver->optimize(mPoseGraph);
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
	
	mLastVertices.insert(LastVertexMap::value_type(s->getName(), VertexObject::Ptr()));
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
        VertexObject::Ptr newVertex(new VertexObject());
	newVertex->odometric_pose = pose;
	newVertex->corrected_pose = pose;
	newVertex->measurement = m;
	mPoseGraph->addVertex(newVertex);
	
	// Add an edge representing the odometry information
	if(mOdometry && mLastVertex)
	{
                EdgeObject::Ptr odomEdge(new EdgeObject());
                odomEdge->setSourceVertex(mLastVertex);
                odomEdge->setTargetVertex(newVertex);

		timeval previous = mLastVertex->measurement->getTimestamp();
		TransformWithCovariance twc = mOdometry->getRelativePose(previous, m->getTimestamp());
		odomEdge->transform = twc.transform;
		odomEdge->covariance = twc.covariance;
		mPoseGraph->addEdge(odomEdge);
	}
	
	// Add an edge to the previous reading of this sensor
	LastVertexMap::iterator it = mLastVertices.find(m->getSensorName());
        VertexObject::Ptr prevSensorVertex = mLastVertices.at(m->getSensorName());
	if(prevSensorVertex)
	{
                EdgeObject::Ptr icpEdge(new EdgeObject());
                icpEdge->setSourceVertex(prevSensorVertex);
                icpEdge->setTargetVertex(newVertex);

		TransformWithCovariance twc = sensor->calculateTransform(m, prevSensorVertex->measurement);
		icpEdge->transform = twc.transform;
		icpEdge->covariance = twc.covariance;

		mPoseGraph->addEdge(icpEdge);
		
		// Update current pose estimate
		mCurrentPose = mCurrentPose * twc.transform;
                newVertex->corrected_pose = mCurrentPose;
	}else
	{
		mLogger->message(INFO, (boost::format("Added first Reading of sensor '%1%'") % m->getSensorName()).str());
	}

        // Overall last vertex
        mLastVertex = newVertex;
	// Set last vertex for this sensor
	mLastVertices[m->getSensorName()] = newVertex;
/*
	// Add edges to other measurements nearby
	mPoseGraph.rebuildIndex();
	VertexList neighbors = mPoseGraph.getNearbyVertices(newVertex, 10.0);
	mLogger->message(DEBUG, (boost::format("radiusSearch() found %1% vertices nearby.") % neighbors.size()).str());
	
	for(VertexList::iterator it = neighbors.begin(); it < neighbors.end(); it++)
	{
		if(it->id == mLastVertex || it->id == newVertex)
			continue;
		EdgeObject icpEdge;
		TransformWithCovariance twc = sensor->calculateTransform(m, (*it).measurement);
		icpEdge.transform = twc.transform;
		icpEdge.covariance = twc.covariance;
		mPoseGraph.addEdge((*it).id, newVertex, icpEdge);
	}
*/ 
}

VertexList GraphMapper::getVerticesFromSensor(std::string sensor)
{
    VertexList vertexList;

    graph_analysis::VertexIterator::Ptr vertexIterator = mPoseGraph->getVertexIterator();
    while(vertexIterator->next())
    {
        VertexObject::Ptr vertex = boost::dynamic_pointer_cast<VertexObject>(vertexIterator->current());
        if(vertex->measurement->getSensorName() == sensor)
        {
            vertexList.push_back(vertex);
        }
    }

    return vertexList;
}
