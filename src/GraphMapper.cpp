#include "GraphMapper.hpp"
#include "Solver.hpp"

#include <boost/format.hpp>
#include <graph_analysis/lemon/DirectedGraph.hpp>
#include <graph_analysis/GraphIO.hpp>

using namespace slam;

// Re-orthogonalize the rotation-matrix
// http://stackoverflow.com/questions/23080791/eigen-re-orthogonalization-of-rotation-matrix
Transform orthogonalize(const Transform& t)
{
	Vector3 x(t(0,0), t(0,1), t(0,2));
	Vector3 y(t(1,0), t(1,1), t(1,2));
	Vector3 z(t(2,0), t(2,1), t(2,2));
	ScalarType error = x.dot(y);
	
	Vector3 x_ort = x - (error/2.0) * y;
	Vector3 y_ort = y - (error/2.0) * x;
	Vector3 z_ort = x_ort.cross(y_ort);

	Transform res = t;
	ScalarType xdot = 0.5 * (3.0 - x_ort.dot(x_ort));
	res(0,0) = xdot * x_ort(0);
	res(0,1) = xdot * x_ort(1);
	res(0,2) = xdot * x_ort(2);
	
	ScalarType ydot = 0.5 * (3.0 - y_ort.dot(y_ort));
	res(1,0) = ydot * y_ort(0);
	res(1,1) = ydot * y_ort(1);
	res(1,2) = ydot * y_ort(2);
	
	ScalarType zdot = 0.5 * (3.0 - z_ort.dot(z_ort));
	res(2,0) = zdot * z_ort(0);
	res(2,1) = zdot * z_ort(1);
	res(2,2) = zdot * z_ort(2);
	
	return res;
}

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
	
	mNeighborRadius = 1.0;
	mMinTranslation = 0.5;
	mMinRotation = 0.1;
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
		mLogger->message(DEBUG, (boost::format("Mapper: Add reading from Sensor '%1%'.") % m->getSensorName()).str());
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
	VertexObject::Ptr newVertex(new VertexObject(m->getSensorName()));
	newVertex->odometric_pose = pose;
	newVertex->corrected_pose = getCurrentPose();
	newVertex->measurement = m;
	
	if(!mFixedVertex)
	{
		mLogger->message(DEBUG, "Add first vertex to the graph.");
		mPoseGraph->addVertex(newVertex);
		mFixedVertex = newVertex;
		if(mSolver)
		{
			graph_analysis::GraphElementId id = mPoseGraph->getVertexId(newVertex);
			mSolver->addNode(id, newVertex->corrected_pose);
			mSolver->setFixed(id);
		}
		return;
	}
	
	// Add an edge representing the odometry information
/*	if(mOdometry && mLastVertex)
	{
		timeval previous = mLastVertex->measurement->getTimestamp();
		TransformWithCovariance twc = mOdometry->getRelativePose(previous, m->getTimestamp());

		EdgeObject::Ptr odomEdge(new EdgeObject());
		odomEdge->setSourceVertex(mLastVertex);
		odomEdge->setTargetVertex(newVertex);
		odomEdge->transform = twc.transform;
		odomEdge->covariance = twc.covariance;
		mPoseGraph->addEdge(odomEdge);
	}
*/

	// Add edges to other measurements nearby
	buildNeighborIndex();
	VertexList neighbors = getNearbyVertices(newVertex, mNeighborRadius);
	mLogger->message(DEBUG, (boost::format("radiusSearch() found %1% vertices nearby.") % neighbors.size()).str());
	
	bool matched = false;
	int added = 0;
	for(VertexList::iterator it = neighbors.begin(); it < neighbors.end() && added < 5; it++)
	{
		if(*it == newVertex)
			continue;

		EdgeObject::Ptr icpEdge;
		try
		{			
			icpEdge = EdgeObject::Ptr(new EdgeObject("icp"));
			Transform guess = (*it)->corrected_pose.inverse() * newVertex->corrected_pose;
			if(std::abs(guess.matrix().determinant() - 1.0) > 0.001)
			{
				mLogger->message(ERROR, (boost::format("Guess transform has determinant %1%!") % guess.matrix().determinant()).str());
			}
			
			TransformWithCovariance twc = sensor->calculateTransform((*it)->measurement, m, guess);
			twc.transform = orthogonalize(twc.transform);
			if(std::abs(twc.transform.matrix().determinant() - 1.0) > 0.001)
			{
				mLogger->message(ERROR, (boost::format("ICP transform has determinant %1%!") % twc.transform.matrix().determinant()).str());
			}
			icpEdge->transform = twc.transform;
			icpEdge->covariance = Covariance::Identity();// twc.covariance;
			icpEdge->setSourceVertex(*it);
			icpEdge->setTargetVertex(newVertex);
			
			if(!matched)
			{
				ScalarType rot = Eigen::AngleAxis<ScalarType>(twc.transform.rotation()).angle();
				ScalarType dx = twc.transform.translation()(0);
				ScalarType dy = twc.transform.translation()(1);
				ScalarType dz = twc.transform.translation()(2);
				ScalarType trans = sqrt(dx*dx + dy*dy + dz*dz);
				mLogger->message(DEBUG, (boost::format("Translation: %1% / Rotation: %2%") % trans % rot).str());
				if(trans < mMinTranslation && rot < mMinRotation)
					return;
				
				mPoseGraph->addVertex(newVertex);
				if(mSolver)
				{
					graph_analysis::GraphElementId id = mPoseGraph->getVertexId(newVertex);
					mSolver->addNode(id, newVertex->corrected_pose);
					mLogger->message(INFO, (boost::format("Added Vertex(id='%1%') to the Graph.") % id).str());
				}
				newVertex->corrected_pose = (*it)->corrected_pose * twc.transform;
				matched = true;
			}
			
			mPoseGraph->addEdge(icpEdge);
			if(mSolver)
			{
				unsigned source = mPoseGraph->getVertexId(*it);
				unsigned target = mPoseGraph->getVertexId(newVertex);
				mLogger->message(INFO, (boost::format("Created edge from node %1% to node %2%.") % source % target).str());
				mSolver->addConstraint(source, target, icpEdge->transform, icpEdge->covariance);
			}
			added++;
		}catch(NoMatch &e)
		{
			continue;
		}
	}
	
	if(!matched)
	{
		mLogger->message(WARNING, "Scan matching not possible!");
	}

	// Overall last vertex
	mLastVertex = newVertex;
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

Transform GraphMapper::getCurrentPose()
{
	if(mLastVertex)
		return mLastVertex->corrected_pose;
	else
		return Transform::Identity();
}
