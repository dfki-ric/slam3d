// slam3d - Frontend for graph-based SLAM
// Copyright (C) 2019 S. Kasperski
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

#include <slam3d/core/Sensor.hpp>
#include <slam3d/core/Solver.hpp>

#include <mutex>

namespace slam3d
{
	class ScanSensor : public Sensor
	{
	public:
		ScanSensor(const std::string& n, Logger* l);
		~ScanSensor();

		/**
		 * @brief Sets a specific solver to optimize local patches.
		 * @details This must not be the same instance used as the backend,
		 * as it will be reset after every optimization. If it is not set,
		 * patches will not be optimized before matching.
		 * @param solver used for patch optimization
		 */
		void setPatchSolver(Solver* solver) { mPatchSolver = solver; }

		/**
		 * @brief Set how far to continue with a breadth-first-search through
		 * the pose graph when building local map patches to match new
		 * measurements against. It will use all vertices that are reachable
		 * by a maximum of r edges.
		 * @param r 
		 */
		void setPatchBuildingRange(unsigned r);

		/**
		 * @brief Sets neighbor radius for scan matching
		 * @details New nodes are matched against nodes of the same sensor
		 * within the given radius, but not more then given maximum.
		 * @param r radius within additional edges are created
		 * @param l maximum number of neighbor links
		 */
		void setNeighborRadius(float r, unsigned l);

		/**
		 * @brief Sets the minimum length of a loop.
		 * @param l
		 */
		void setMinLoopLength(unsigned l);

		/**
		 * @brief Sets whether to link each scan to its predecessor.
		 * @param l
		 */
		void setLinkPrevious(bool l);

		/**
		 * @brief Add a new measurement from this sensor.
		 * @param scan
		 */
		bool addMeasurement(const Measurement::Ptr& scan);

		/**
		 * @brief Add a new measurement from this sensor together with an odometry pose.
		 * @param scan
		 * @param odom
		 */
		bool addMeasurement(const Measurement::Ptr& scan, const Transform& odom);

		/**
		 * @brief Check if a new measurement with given odometry would be added.
		 * @param odom
		 */
		bool checkMeasurementDistance(const Transform& odom);

		/**
		 * @brief Create a virtual measurement by accumulating scans from given vertices.
		 * @param vertices list of vertices that should contain a Measurement of this sensor
		 * @param pose origin of the accumulated scan
		 * @throw BadMeasurementType
		 */		
		virtual Measurement::Ptr createCombinedMeasurement(const VertexObjectList& vertices, Transform pose) const = 0;

		/**
		 * @brief Build a local map patch starting from the given source vertex.
		 * @param source
		 */
		Measurement::Ptr buildPatch(IdType source);

		/**
		 * @brief Create a constraint between two measurements.
		 * @details The odometry transformation and the resulting constraint are
		 * with regards to the robot coordinate system. Make sure that sensor_pose
		 * is properly set within the measurements.
		 * @param source
		 * @param target
		 * @param odometry
		 */
		virtual Constraint::Ptr createConstraint(const Measurement::Ptr& source,
		                                         const Measurement::Ptr& target,
		                                         const Transform& odometry,
		                                         bool loop) = 0;

		/**
		 * @brief Create a linking constraint between source and target.
		 * @param source_id
		 * @param target_id
		 */
		virtual void link(IdType source_id, IdType target_id);

		/**
		 * @brief Create a linking constraint between source and target.
		 * @param source_id
		 * @param target_id
		 * @param guess
		 */
		virtual void link(IdType source_id, IdType target_id, const Transform& guess);


		/**
		 * @brief Create connecting edges to nearby vertices.
		 * @param vertex
		 */
		void linkToNeighbors(IdType vertex);
		
		/**
		 * @brief Create connecting edges for last added vertex.
		 * @param mt whether to run in a separate thread
		 */
		void linkLastToNeighbors(bool mt = false);

		/**
		 * @brief Returns the current pose from sequential scan matching.
		 */
		Transform getCurrentPose() const;

	private:
		Solver* mPatchSolver;
		std::mutex mPatchSolverMutex;

		unsigned mPatchBuildingRange;
		unsigned mMaxNeighorLinks;
		float mNeighborRadius;
		unsigned mMinLoopLength;
		bool mLinkPrevious;

		Transform mLastOdometry;
		Transform mLastTransform;
	};
}
