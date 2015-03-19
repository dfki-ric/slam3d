#ifndef GRAPH_SLAM_POSEGRAPH_HPP
#define GRAPH_SLAM_POSEGRAPH_HPP

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graphviz.hpp>

#include <Measurement.hpp>

namespace slam
{
	/**
	 * @class VertexObject
	 * @author Sebastian Kasperski
	 * @date 03/16/15
	 * @file PoseGraph.hpp
	 * @brief Object attached to a vertex in the pose graph.
	 * Contains a pointer to an abstract measurement, which could
	 * be anything, e.g. a range scan, point cloud or image.
	 */
	struct VertexObject
	{
		Transform odometric_pose;
		Transform corrected_pose;
		Measurement* measurement;
		unsigned int id;
	};
	
	/**
	 * @class EdgeObject
	 * @author Sebastian Kasperski
	 * @date 03/16/15
	 * @file PoseGraph.hpp
	 * @brief Object attached to an edge in the pose graph.
	 * Contains the relative transform from source to target.
	 */
	struct EdgeObject
	{
		Transform transform;
		Covariance covariance;
	};
	
	// Definitions of boost-graph related types
	typedef boost::listS VRep;
	typedef boost::listS ERep;
	typedef boost::undirectedS GType;
	typedef boost::property<boost::vertex_index_t, int, VertexObject> VProp;
	typedef boost::property<boost::edge_index_t, int, EdgeObject> EProp;
	typedef boost::adjacency_list<VRep, ERep, GType, VProp, EProp> AdjacencyGraph;
	
	typedef boost::graph_traits<AdjacencyGraph>::vertex_descriptor Vertex;
	typedef graph_traits<AdjacencyGraph>::vertex_iterator VertexIterator;
	typedef std::pair<VertexIterator, VertexIterator> VertexRange;
	
	typedef boost::graph_traits<AdjacencyGraph>::edge_descriptor Edge;
	typedef graph_traits<AdjacencyGraph>::edge_iterator EdgeIterator;
	typedef std::pair<EdgeIterator, EdgeIterator> EdgeRange;
	
	typedef boost::graph_traits<AdjacencyGraph>::adjacency_iterator AdjacencyIterator;
	typedef std::pair<AdjacencyIterator, AdjacencyIterator> AdjacencyRange;
	
	/**
	 * @class PoseGraph
	 * @author Sebastian Kasperski
	 * @date 03/16/15
	 * @file PoseGraph.hpp
	 * @brief This is the central representation of the environment model, used
	 * for graph-based SLAM solutions. Each node in the graph holds one single
	 * measurement that has been added to the world model. Possible
	 * implememtations of measurements should inherit from Measurement, so they
	 * can be attached to a node with a pointer in VertexObject.
	 * 
	 * Spatial relations between measurements of the same or different kind are
	 * represented by edges between their corresponding nodes. These constraints
	 * can be a result of scan-matching, ICP, landmark detection, loop-closing,
	 * etc...
	 */
	class PoseGraph
	{
	public:
	
		PoseGraph();
		~PoseGraph();
		
		/**
		 * @brief Creates a new vertex with given object in the graph.
		 * @param object
		 */
		Vertex addVertex(const VertexObject& object);

		/**
		 * @brief Creates a new edge with given object from source to target.
		 * @param source
		 * @param target
		 * @param object
		 */
		Edge addEdge(Vertex source, Vertex target, const EdgeObject& object);
		
		/**
		 * @brief Remove v from the graph and all connecting edges.
		 * @param v
		 */
		void removeVertex(Vertex v);
		
		/**
		 * @brief Remove e from the graph.
		 * @param e
		 */
		void removeEdge(Edge e);
	
		AdjacencyRange getAdjacentVertices(Vertex v);

		Measurement* getMeasurement(Vertex v);

		/**
		 * @brief Write the cuurent graph in DOT-format to the given stream.
		 * @param out
		 */
		void dumpGraphViz(std::ostream& out);

	private:
		AdjacencyGraph mGraph;
		unsigned int mNextVertexId;
		unsigned int mNextEdgeId;
	};
}

#endif
