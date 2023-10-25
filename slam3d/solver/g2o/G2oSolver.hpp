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

#pragma once

#include <slam3d/core/Solver.hpp>
#include <boost/thread/mutex.hpp>

namespace slam3d
{	
	/**
	 * @class G2oSolver
	 * @brief A solver for graph otimization that uses the g2o-backend.
	 * @details See: https://github.com/RainerKuemmerle/g2o for documentation
	 * on the backend.
	 */
	class G2oSolver : public Solver
	{
	public:
		G2oSolver(Logger* logger);
		~G2oSolver();
		
		void addVertex(IdType id, const Transform& pose);
		void addEdgeSE3(IdType source, IdType target, SE3Constraint::Ptr se3);
		void addEdgeGravity(IdType vertex, GravityConstraint::Ptr grav);
		void addEdgePosition(IdType vertex, PositionConstraint::Ptr pos);
		void addEdgeOrientation(IdType vertex, OrientationConstraint::Ptr orient);
		void setFixed(IdType id);
		bool compute(unsigned iterations);
		void clear();
		void saveGraph(std::string filename);
		
		IdPoseVector getCorrections();
		
	protected:
		IdPoseVector mCorrections;
		bool mInitialized;
		boost::mutex mMutex;

	private:
		struct Internal;
		std::unique_ptr<Internal> mInt;
	};
}
