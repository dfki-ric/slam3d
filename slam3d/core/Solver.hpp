// slam3d - Frontend for graph-based SLAM
// Copyright (C) 2017 S. Kasperski
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
// IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
// TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
// PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
// TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
// LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
// NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#ifndef SLAM_SOLVER_HPP
#define SLAM_SOLVER_HPP

#include "Types.hpp"
#include "Logger.hpp"

#include <vector>

namespace slam3d
{
	typedef std::pair<IdType, Transform> IdPose;
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
			DuplicateVertex(IdType id):vertex_id(id){}
			virtual const char* what() const throw()
			{
				std::ostringstream msg;
				msg << "Vertex with ID: " << vertex_id << " has already been defined!";
				return msg.str().c_str();
			}
			
			IdType vertex_id;
		};

		/**
		 * @class UnknownVertex
		 * @brief Exception thrown when a requested Vertex-ID does not exist.
		 */
		class UnknownVertex: public std::exception
		{
		public:
			UnknownVertex(IdType id):vertex_id(id){}
			virtual const char* what() const throw()
			{
				std::ostringstream msg;
				msg << "Vertex with ID: " << vertex_id << " does not exist!";
				return msg.str().c_str();
			}
			
			IdType vertex_id;
		};

		/**
		 * @class BadEdge
		 * @brief Exception thrown when source or target do not exist in the graph.
		 */
		class BadEdge: public std::exception
		{
		public:
			BadEdge(IdType s, IdType t):source(s),target(t){}
			virtual const char* what() const throw()
			{
				std::ostringstream msg;
				msg << "Failed to create edge from vertex " <<  source << " to " << target << "!";
				return msg.str().c_str();
			}
			
			IdType source;
			IdType target;
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
		 * @brief Adds a vertex to the internal graph representation.
		 * @param id unique identifier of the vertex
		 * @param pose initial pose of the new vertex
		 */
		virtual void addVertex(IdType id, const Transform& pose) = 0;
		
		/**
		 * @brief Add an edge to the internal graph representation.
		 * @param source id of source vertex
		 * @param target id of target vertex
		 * @param c pointer to a generic constraint
		 */
		void addEdge(IdType source, IdType target, Constraint::Ptr c)
		{
			switch(c->getType())
			{
			case SE3:
				addEdgeSE3(source, target, boost::static_pointer_cast<SE3Constraint>(c));
				break;
			case GRAVITY:
				addEdgeGravity(source, boost::static_pointer_cast<GravityConstraint>(c));
				break;
			case POSITION:
				addEdgePosition(source, boost::static_pointer_cast<PositionConstraint>(c));
				break;
			default:
				std::ostringstream msg;
				msg << "Edge with type " << c->getTypeName() << " is not known!";
				throw std::runtime_error(msg.str().c_str());
			}
		}
		
		/**
		 * @brief Adds a SE3 edge between two vertices in the graph.
		 * @details The source and target vertices have to be added before the edge.
		 * @param source id of the edge's from-vertex
		 * @param target id if the edge's to-vertex
		 * @param se3 constraint describing a relative pose between two vertices
		 */
		virtual void addEdgeSE3(IdType source, IdType target, SE3Constraint::Ptr se3) = 0;
		
		/**
		 * @brief Adds a gravity edge to a vertex in the graph.
		 * @details The vertex has to be added to the graph before.
		 * @param vertex id of the vertex this edge connects to
		 * @param grav constraint describing the measured direction of gravity
		 */
		virtual void addEdgeGravity(IdType vertex, GravityConstraint::Ptr grav) = 0;
		
		/**
		 * @brief Adds a position edge to a vertex in the graph.
		 * @param vertex id of the vertex this edge connects to
		 * @param pos constraint describing the measured position
		 */
		virtual void addEdgePosition(IdType vertex, PositionConstraint::Ptr pos) = 0;
		
		/**
		 * @brief Fix the vertex with the given id, so it is not moved during optimization.
		 * @details At least one vertex must be fixed in order to hold the map in place.
		 * @param id
		 */
		virtual void setFixed(IdType id) = 0;
		
		/**
		 * @brief Start optimization of the defined graph.
		 */
		virtual bool compute(unsigned iterations = 100) = 0;
	
		/**
		 * @brief Clear internal graph structure by removing all vertices and constraints.
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
		 * ID's and Transforms, that have to be applied to the vertices with the
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
