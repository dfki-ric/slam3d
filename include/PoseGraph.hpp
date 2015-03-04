#ifndef GRAPH_SLAM_POSEGRAPH_H
#define GRAPH_SLAM_POSEGRAPH_H

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graphviz.hpp>

#include <Measurement.hpp>

namespace slam
{
	struct VertexObject
	{
		Measurement* measurement;
	};
	
	struct EdgeObject
	{
	};
	
	// Definitions of boost-graph related types
	typedef boost::listS VRep;
	typedef boost::listS ERep;
	typedef boost::undirectedS GType;
	typedef boost::property<boost::vertex_index_t, int, VertexObject> VProp;
	typedef boost::property<boost::edge_index_t, int, EdgeObject> EProp;
	typedef boost::adjacency_list<VRep, ERep, GType, VProp, EProp> AdjacencyGraph;
	
	typedef typename boost::graph_traits<AdjacencyGraph>::vertex_descriptor Vertex;
	typedef typename boost::graph_traits<AdjacencyGraph>::edge_descriptor Edge;
	
	typedef typename boost::graph_traits<AdjacencyGraph>::adjacency_iterator AdjacencyIterator;
	typedef	typename std::pair<AdjacencyIterator, AdjacencyIterator> AdjacencyRange;
	
	class PoseGraph
	{
	public:
	
		PoseGraph();
		
		Vertex addVertex(Measurement* m);
		Edge addEdge(Vertex source, Vertex target);
		void removeVertex(Vertex v);
		void removeEdge(Edge e);
	
		AdjacencyRange getAdjacentVertices(Vertex v);

		Measurement* getMeasurement(Vertex v);

		void dumpGraphViz(std::ostream& out);

//	private:
		AdjacencyGraph mGraph;
		unsigned int mNextVertexId;
		unsigned int mNextEdgeId;
	};
}

#endif
