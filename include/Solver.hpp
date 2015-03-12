#ifndef SLAM_SOLVER_HPP
#define SLAM_SOLVER_HPP

#include "PoseGraph.hpp"
#include "Logger.hpp"

#include <vector>

namespace slam
{
	typedef std::pair<int, Transform> IdPose;
	typedef std::vector<IdPose> IdPoseVector;

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
	
	class Solver
	{
	public:
		Solver(Logger* logger):mLogger(logger){}
	
		virtual void addNode(const VertexObject &v, int id) = 0;
		virtual void addConstraint(const EdgeObject &e, int source, int target) = 0;
		virtual void compute() = 0;
		
		virtual IdPoseVector getCorrections() = 0;
		
		void setLogger(Logger* log) {mLogger = log;}
		
	protected:
		Logger* mLogger;
	};
}

#endif