#include "PoseGraph.hpp"
#include "Solver.hpp"

using namespace slam;

PoseGraph::PoseGraph()
{
	mNextVertexId = 0;
	mNextEdgeId = 0;
	mLastVertex = -1;
}

PoseGraph::~PoseGraph()
{
	
}

PoseGraph::IdType PoseGraph::addVertex(const VertexObject& object)
{
	PoseGraph::IdType newVertexId = mNextVertexId;
	mNextVertexId++;
	
	Vertex n = boost::add_vertex(mGraph);
	mGraph[n] = object;
	mGraph[n].id = newVertexId;
	boost::put(boost::vertex_index_t(), mGraph, n, newVertexId);
	mVertexMap.insert(VertexMap::value_type(newVertexId, n));
	mLastVertex = newVertexId;
	
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
