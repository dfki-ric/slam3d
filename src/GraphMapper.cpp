#include "GraphMapper.hpp"
#include "Solver.hpp"

#include <boost/format.hpp>
#include <graph_analysis/lemon/DirectedGraph.hpp>
#include <graph_analysis/GraphIO.hpp>

using namespace slam;

VertexObject::Ptr GraphMapper::fromBaseGraph(graph_analysis::Vertex::Ptr base)
{
	VertexObject::Ptr v = boost::dynamic_pointer_cast<VertexObject>(base);
	if(!v)
	{
		throw BadElementType();
	}
	return v;
}

EdgeObject::Ptr GraphMapper::fromBaseGraph(graph_analysis::Edge::Ptr base)
{
	EdgeObject::Ptr e = boost::dynamic_pointer_cast<EdgeObject>(base);
	if(!e)
	{
		throw BadElementType();
	}
	return e;
}

GraphMapper::GraphMapper(Logger* log)
 : mPoseGraph( new graph_analysis::lemon::DirectedGraph()),
   mIndex(flann::KDTreeSingleIndexParams())
{
	mOdometry = NULL;
	mSolver = NULL;
	mLogger = log;
	mCurrentPose = Transform::Identity();
}

GraphMapper::~GraphMapper()
{
	std::string file = "pose_graph.dot";
	graph_analysis::io::GraphIO::write(file, *mPoseGraph, graph_analysis::representation::GRAPHVIZ);
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
	
	// Add vertices to the solver
	std::vector<graph_analysis::Vertex::Ptr> vertices = mPoseGraph->getAllVertices();
	if(vertices.size() == 0)
	{
		mLogger->message(ERROR, "Graph does not contain any nodes!");
		return false;
	}
	for(std::vector<graph_analysis::Vertex::Ptr>::iterator it = vertices.begin(); it < vertices.end(); it++)
	{
		graph_analysis::GraphElementId id = mPoseGraph->getVertexId(*it);
		mSolver->addNode(id, fromBaseGraph(*it)->corrected_pose);
	}
	
	// Fix first node in the graph
	mSolver->setFixed(mPoseGraph->getVertexId(mFixedVertex));
	
	// Add edges to the solver
	std::vector<graph_analysis::Edge::Ptr> edges = mPoseGraph->getAllEdges();
	for(std::vector<graph_analysis::Edge::Ptr>::iterator it = edges.begin(); it < edges.end(); it++)
	{
		unsigned source = mPoseGraph->getVertexId((*it)->getSourceVertex());
		unsigned target = mPoseGraph->getVertexId((*it)->getTargetVertex());
		EdgeObject::Ptr edge = fromBaseGraph(*it);
		mSolver->addConstraint(source, target, edge->transform, edge->covariance);
	}
	
	// Optimize
	if(!mSolver->compute())
	{
		return false;
	}

	// Retrieve results
	IdPoseVector res = mSolver->getCorrections();
	for(IdPoseVector::iterator it = res.begin(); it < res.end(); it++)
	{
		unsigned int id = it->first;
		Transform tf = it->second;
		VertexObject::Ptr v = fromBaseGraph(mPoseGraph->getVertex(id));
		v->corrected_pose = tf;
	}
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
	newVertex->corrected_pose = mCurrentPose;
	newVertex->measurement = m;
	mPoseGraph->addVertex(newVertex);
	if(!mFixedVertex)
	{
		mFixedVertex = newVertex;
	}
	
	// Add an edge representing the odometry information
	Transform guess = Transform::Identity();
	if(mOdometry && mLastVertex)
	{
		timeval previous = mLastVertex->measurement->getTimestamp();
		TransformWithCovariance twc = mOdometry->getRelativePose(previous, m->getTimestamp());

		EdgeObject::Ptr odomEdge(new EdgeObject());
		odomEdge->setSourceVertex(mLastVertex);
		odomEdge->setTargetVertex(newVertex);
		odomEdge->transform = twc.transform;
		odomEdge->covariance = twc.covariance;
		mPoseGraph->addEdge(odomEdge);
		guess = twc.transform;
	}

	// Overall last vertex
	mLastVertex = newVertex;

	// Add edges to other measurements nearby
	buildNeighborIndex();
	VertexList neighbors = getNearbyVertices(newVertex, 5.0);
	mLogger->message(DEBUG, (boost::format("radiusSearch() found %1% vertices nearby.") % neighbors.size()).str());
	
	bool matched = false;
	int count = 0;
	for(VertexList::iterator it = neighbors.begin(); it < neighbors.end(); it++)
	{
		if(*it == newVertex)// || *it == mLastVertex)
			continue;

		if(count > 5)
			break;
		count++;

		EdgeObject::Ptr icpEdge(new EdgeObject);
		try
		{
			TransformWithCovariance twc = sensor->calculateTransform(m, (*it)->measurement, guess);
			icpEdge->transform = twc.transform;
			icpEdge->covariance = twc.covariance;
			icpEdge->setSourceVertex(*it);
			icpEdge->setTargetVertex(newVertex);
			mPoseGraph->addEdge(icpEdge);
			
			if(!matched)
			{
				mCurrentPose = (*it)->corrected_pose * twc.transform;
				newVertex->corrected_pose = mCurrentPose;
				matched = true;
			}
		}catch(NoMatch &e)
		{
			continue;
		}
	}
	
	if(!matched)
	{
		mLogger->message(WARNING, "Scan matching not possible!");
	}

}

VertexList GraphMapper::getVerticesFromSensor(const std::string& sensor)
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

EdgeList GraphMapper::getEdgesFromSensor(const std::string& sensor)
{
	EdgeList edgeList;
	
	graph_analysis::EdgeIterator::Ptr edgeIterator = mPoseGraph->getEdgeIterator();
	while(edgeIterator->next())
	{
		EdgeObject::Ptr edge = boost::dynamic_pointer_cast<EdgeObject>(edgeIterator->current());
		edgeList.push_back(edge);
	}
	
	return edgeList;
}

void GraphMapper::buildNeighborIndex()
{
	std::vector<graph_analysis::Vertex::Ptr> vertices = mPoseGraph->getAllVertices();
	int numOfVertices = vertices.size();
	flann::Matrix<float> points(new float[numOfVertices * 3], numOfVertices, 3);

	int row = 0;
	mIndexMap.clear();
	for(std::vector<graph_analysis::Vertex::Ptr>::iterator it = vertices.begin(); it < vertices.end(); it++)
	{
		VertexObject::Ptr v = boost::dynamic_pointer_cast<VertexObject>(*it);
		Transform::TranslationPart t = v->corrected_pose.translation();
		points[row][0] = t[0];
		points[row][1] = t[1];
		points[row][2] = t[2];
		mIndexMap.insert(std::pair<int, VertexObject::Ptr>(row, v));
		row++;
	}
	
	mIndex.buildIndex(points);
}

VertexList GraphMapper::getNearbyVertices(VertexObject::Ptr vertex, float radius)
{
	// Fill in the query point
	flann::Matrix<float> query(new float[3], 1, 3);
	Transform::TranslationPart t = vertex->corrected_pose.translation();
	query[0][0] = t[0];
	query[0][1] = t[1];
	query[0][2] = t[2];
	
	// Find points nearby
	std::vector< std::vector<int> > neighbors;
	std::vector< std::vector<NeighborIndex::DistanceType> > distances;
	mIndex.radiusSearch(query, neighbors, distances, radius, mSearchParams);
	
	// Write the result
	VertexList result;
	std::vector<int>::iterator it;
	for(it = neighbors[0].begin(); it < neighbors[0].end(); it++)
	{
		result.push_back(mIndexMap[*it]);
	}
	
	return result;
}
