#ifndef SLAM_SOLVER_HPP
#define SLAM_SOLVER_HPP

#include "Types.hpp"
#include "Logger.hpp"

#include <vector>

namespace slam3d
{
	typedef std::pair<int, Transform> IdPose;
	typedef std::vector<IdPose> IdPoseVector;
	
	/**
	 * @class Solver
	 * @brief Abstact base class for generic graph optimization solutions.
	 */
	class Solver
	{
	public:
		/**
		 * @class DuplicateVertex
		 * @brief Exception thrown when a given Vertex-ID is already present.
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
		 * @class UnknownVertex
		 * @brief Exception thrown when a requested Vertex-ID does not exist.
		 */
		class UnknownVertex: public std::exception
		{
		public:
			UnknownVertex(int id):vertex_id(id){}
			virtual const char* what() const throw()
			{
				std::ostringstream msg;
				msg << "Node with ID: " << vertex_id << " does not exist!";
				return msg.str().c_str();
			}
			
			int vertex_id;
		};

		/**
		 * @class BadEdge
		 * @brief Exception thrown when source or target do not exist in the graph.
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

	public:
		/**
		 * @brief Constructor setting the used logging device.
		 * @param logger pointer to the logger used by the solver
		 */
		Solver(Logger* logger):mLogger(logger){}
		
		/**
		 * @brief Virtual Destructor.
		 */
		virtual ~Solver(){}
	
		/**
		 * @brief Adds a node to the internal graph representation.
		 * @param v VertexObject from the PoseGraph
		 * @param id unique identifier of the vertex
		 */
		virtual void addNode(unsigned id, Transform pose) = 0;
		
		/**
		 * @brief Adds a constraint between two nodes in the graph.
		 * @details The source and target nodes have to be added before the edge.
		 * @param source the edge's from-node
		 * @param target the edge's to-node
		 * @param tf
		 * @param cov
		 */
		virtual void addConstraint(unsigned source, unsigned target, Transform tf, Covariance cov = Covariance::Identity()) = 0;
		
		/**
		 * @brief Fix the node with the given id, so it is not moved during optimization.
		 * @details At least one node must be fixed in order to hold the map in place.
		 * @param id
		 */
		virtual void setFixed(unsigned id) = 0;
		
		/**
		 * @brief Start optimization of the defined graph.
		 */
		virtual bool compute() = 0;
	
		/**
		 * @brief Clear internal graph structure by removing all nodes and constraints.
		 */
		virtual void clear() = 0;
	
		/**
		 * @brief Save the current graph to a file.
		 * @details The format depends on the specific implementation.
		 * @param filename The name of the file, where the graph is saved to
		 */
		virtual void saveGraph(std::string filename) = 0;
		
		/**
		 * @brief Get the result of the optimization.
		 * @details This should be used after compute(). It returns a list of
		 * ID's and Transforms, that have to be applied to the nodes with the
		 * given ID to minimize the error in the PoseGraph.
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
