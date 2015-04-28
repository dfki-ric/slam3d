#include "PoseGraph.hpp"
#include "Solver.hpp"

using namespace slam;

PoseGraph::PoseGraph() : mIndex(flann::KDTreeSingleIndexParams())
{
	mNextVertexId = 0;
	mNextEdgeId = 0;
}

PoseGraph::~PoseGraph()
{
	
}

PoseGraph::IdType PoseGraph::addVertex(VertexObject::Ptr vertexObject)
{
	// Create a new ID for this vertex
	PoseGraph::IdType newVertexId = mNextVertexId;
	mNextVertexId++;
	
	// Add a new vertex to the Graph
	Vertex n = boost::add_vertex(mGraph);
	mGraph[n] = object;
	mGraph[n].id = newVertexId;
	
	// Set the internal index property (This probably shouldn't be done.)
	boost::put(boost::vertex_index_t(), mGraph, n, newVertexId);

	// Insert to ID-Vertex map and memorize last added vertex
	mVertexMap.insert(VertexMap::value_type(newVertexId, n));
	
	// Add to nearest neighbor index
	flann::Matrix<float> point(new float[3], 1, 3);
	Transform::ConstTranslationPart t = object.corrected_pose.translation();
	point[0][0] = t[0];
	point[0][1] = t[1];
	point[0][2] = t[2];
//	mIndex.addPoints(point);
	
	return newVertexId;
}

PoseGraph::IdType PoseGraph::addEdge(IdType source, IdType target, const EdgeObject& object)
{
	PoseGraph::IdType newEdgeId = mNextEdgeId;
	
	std::pair<Edge, bool> result = boost::add_edge(mVertexMap[source], mVertexMap[target], mGraph);
	Edge n = result.first;
	mGraph[n] = object;
	boost::put(boost::edge_index_t(), mGraph, n, newEdgeId);
	mEdgeMap.insert(EdgeMap::value_type(newEdgeId, n));
	mNextEdgeId++;
	
	return newEdgeId;
}

void PoseGraph::removeVertex(PoseGraph::IdType id)
{
	Vertex v = mVertexMap[id];
	mVertexMap.erase(id);
	boost::clear_vertex(v, mGraph);
	boost::remove_vertex(v, mGraph);
}

void PoseGraph::removeEdge(PoseGraph::IdType id)
{
	Edge e = mEdgeMap[id];
	mEdgeMap.erase(id);
	boost::remove_edge(e, mGraph);
}

VertexObject PoseGraph::getVertex(PoseGraph::IdType id)
{
	return mGraph[mVertexMap.at(id)];
}

void PoseGraph::dumpGraphViz(std::ostream& out)
{
	boost::write_graphviz(out, mGraph);
}

void PoseGraph::optimize(Solver* solver)
{
	// Add vertices to the solver
	VertexIterator v_begin, v_end;
	boost::tie(v_begin, v_end) = boost::vertices(mGraph);
	for(VertexIterator it = v_begin; it != v_end; it++)
	{
		solver->addNode(mGraph[*it]);
	}
	
	// Fix first node in the graph
	solver->setFixed(mGraph[*v_begin].id);
	
	// Add edges to the solver
	EdgeIterator e_begin, e_end;
	boost::tie(e_begin,e_end) = boost::edges(mGraph);
	for(EdgeIterator it = e_begin; it != e_end; it++)
	{
		Vertex source = boost::source(*it, mGraph);
		Vertex target = boost::target(*it, mGraph);
		solver->addConstraint(mGraph[*it], mGraph[source].id, mGraph[target].id);
	}
	
	// Optimize
	solver->compute();

	// Retrieve results
	IdPoseVector res = solver->getCorrections();
	for(IdPoseVector::iterator it = res.begin(); it < res.end(); it++)
	{
		unsigned int id = it->first;
		Transform tf = it->second;
		mGraph[mVertexMap.at(id)].corrected_pose = tf;
	}
}

VertexList PoseGraph::getVerticesFromSensor(std::string sensor)
{
	VertexList result;
	VertexRange range = boost::vertices(mGraph);
	for(VertexIterator it = range.first; it != range.second; it++)
	{
		if(mGraph[*it].measurement->getSensorName() == sensor)
		{
			mapInsert(it, result);
		}
	}
	return result;
}

EdgeList PoseGraph::getEdges(unsigned type)
{
	EdgeList result;
	EdgeRange range = boost::edges(mGraph);
	for(EdgeIterator it = range.first; it != range.second; it++)
	{
		if(type == 0 || mGraph[*it].edge_type == type)
		{
			mapInsert(it, result);
		}
	}
	return result;
}

VertexLinkList PoseGraph::getVertexLinks(unsigned type)
{
	VertexLinkList result;
	EdgeRange range = boost::edges(mGraph);
	for(EdgeIterator it = range.first; it != range.second; it++)
	{
		if(type == 0 || mGraph[*it].edge_type == type)
		{
			unsigned source = mGraph[boost::source(*it, mGraph)].id;
			unsigned target = mGraph[boost::target(*it, mGraph)].id;
			result.push_back(VertexLinkList::value_type(source, target));
		}
	}
	return result;
}

void PoseGraph::setCorrectedPose(IdType id, Transform pose)
{
	mGraph[mVertexMap[id]].corrected_pose = pose;
}

void PoseGraph::rebuildIndex()
{
	int numOfVertices = boost::num_vertices(mGraph);
	flann::Matrix<float> points(new float[numOfVertices * 3], numOfVertices, 3);
	VertexRange range = boost::vertices(mGraph);
	int row = 0;
	mIndexMap.clear();
	for(VertexIterator it = range.first; it != range.second; it++)
	{
		VertexObject v = mGraph[*it];
		Transform::TranslationPart t = v.corrected_pose.translation();
		points[row][0] = t[0];
		points[row][1] = t[1];
		points[row][2] = t[2];
		mIndexMap.insert(std::pair<int, Vertex>(row, *it));
		row++;
	}
	
	mIndex.buildIndex(points);
}

VertexList PoseGraph::getNearbyVertices(IdType id, float radius)
{
	// Fill in the query point
	flann::Matrix<float> query(new float[3], 1, 3);
	Transform::TranslationPart t = mGraph[mIndexMap[id]].corrected_pose.translation();
	query[0][0] = t[0];
	query[0][1] = t[1];
	query[0][2] = t[2];
	
	// Find points nearby
	std::vector< std::vector<int> > neighbors;
	std::vector< std::vector<Index::DistanceType> > distances;
	int found = mIndex.radiusSearch(query, neighbors, distances, radius, mSearchParams);
	
	// Write the result
	VertexList result;
	std::vector<int>::iterator it;
	for(it = neighbors[0].begin(); it < neighbors[0].end(); it++)
	{
		result.insert(VertexList::value_type(*it, mGraph[mIndexMap[*it]]));
//		mapInsert(mIndexMap[*it], result);
	}
	
	return result;
}
