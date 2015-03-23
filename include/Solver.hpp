#ifndef SLAM_SOLVER_HPP
#define SLAM_SOLVER_HPP

#include "PoseGraph.hpp"
#include "Logger.hpp"

#include <vector>

namespace slam
{
	typedef std::pair<int, Transform> IdPose;
	typedef std::vector<IdPose> IdPoseVector;

	/**
	 * @class DuplicateVertex
	 * @author Sebastian Kasperski
	 * @date 03/13/15
	 * @file Solver.hpp
	 * @brief Exception thrown when a given Vertex-ID is already present
	 */
	class DuplicateVertex: public std::exception
	{
	public:
		DuplicateVertex(int id):vertex_id(id){}
		virtual const char* what() const throw()
		{
			std::ostringstream msg;
			msg << "Node with ID: " << vertex_id << " has already been defined!";
			return msg.str().c_str();
		}
		
		int vertex_id;
	};

	/**
	 * @class BadEdge
	 * @author Sebastian Kasperski
	 * @date 03/13/15
	 * @file Solver.hpp
	 * @brief Exception thrown when either source or target node of an edge are not
	 * present within the graph.
	 */
	class BadEdge: public std::exception
	{
	public:
		BadEdge(int s, int t):source(s),target(t){}
		virtual const char* what() const throw()
		{
			std::ostringstream msg;
			msg << "Failed to create edge from node " <<  source << " to " << target << "!";
			return msg.str().c_str();
		}
		
		int source;
		int target;
	};
	
	/**
	 * @class Solver
	 * @author Sebastian Kasperski
	 * @date 03/13/15
	 * @file Solver.hpp
	 * @brief Abstact base class for generic graph optimization solutions.
	 */
	class Solver
	{
	public:
		/**
		 * @brief Constructor setting the used logging device.
		 * @param logger Pointer to the logger used by the solver
		 */
		Solver(Logger* logger):mLogger(logger){}
	
		/**
		 * @brief Add a node to the internal graph representation.
		 * @param v VertexObject from the PoseGraph
		 * @param id Unique identifier of the vertex
		 */
		virtual void addNode(const VertexObject &v) = 0;
		
		/**
		 * @brief Add a constraint between two nodes in the graph.
		 * The nodes have to be added BEFORE the edge.
		 * @param e EdgeObject defining the spatial relation between the nodes
		 * @param source The edge's from-node
		 * @param target The edge's to-node
		 */
		virtual void addConstraint(const EdgeObject &e, int source, int target) = 0;
		
		/**
		 * @brief Start optimization of the defined graph.
		 */
		virtual void compute() = 0;
		
		/**
		 * @brief Save the current graph to a file.
		 * The format depends on the specific implementation.
		 * @param filename The name of the file, where the graph is saved to
		 */
		virtual void saveGraph(std::string filename) = 0;
		
		/**
		 * @brief Get the result of the optimization. This should be used after
		 * compute(). Will returns a list of ID's and Transforms, that have to
		 * be applied to the nodes with the given ID to minimize the error
		 * in the PoseGraph.
		 */
		virtual IdPoseVector getCorrections() = 0;
		
		/**
		 * @brief Set the Logger to be used by the Solver.
		 * @param log Specialized logger implementation.
		 */
		void setLogger(Logger* log) {mLogger = log;}
		
	protected:
		Logger* mLogger;
	};
}

#endif