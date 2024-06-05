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

#include <slam3d/core/Sensor.hpp>
#include <slam3d/core/PoseSensor.hpp>
#include <slam3d/core/Graph.hpp>
#include <slam3d/core/MeasurementStorage.hpp>

namespace slam3d
{
	class Mapper
	{
	public:
		Mapper(Graph* graph, Logger* log, const Transform& start = Transform::Identity(), std::shared_ptr<MeasurementStorage> measurements = std::make_shared<MeasurementStorage>());
		virtual ~Mapper();

		/**
		 * @brief Set the start pose. Must be called before the first node is added.
		 * @param start
		 */
		void setStartPose(const Transform& start);

		/**
		 * @brief Access to the graph structure.
		 * @return graph
		 */
		Graph* getGraph() { return mGraph; }
	
		/**
		 * @brief Access to the measurements.
		 * @return measurement storage
		 */
		std::shared_ptr<MeasurementStorage> getMeasurementStorage() { return mMeasurements; }

		/**
		 * @brief Register a pose sensor to create spatial constraints.
		 * @details For each node that is added by a registered sensor, each
		 * registered pose sensor will be triggered to create additional edges
		 * in the graph, e.g an odometry sensor will add an edge between the last
		 * and the new vertex holding the odometry data. 
		 * @param s pose sensor to be registered for mapping
		 */
		void registerPoseSensor(PoseSensor* s);
		
		/**
		 * @brief Register a sensor, so its data can be added to the graph.
		 * @details Multiple sensors can be used, but in this case at least one pose
		 * sensor is required for the mapping to work correctly. Matching is currently
		 * done only between measurements of the same sensor.
		 * @param s sensor to be registered for mapping
		 */
		void registerSensor(Sensor* s);
		
		/**
		 * @brief Add a new measurement to the graph.
		 * @details Creates a new node in the graph, adds the given measurement to it
		 * and calls each registered PoseSensor to create spatial constraints.
		 * @param m pointer to a new measurement
		 * @return id of the newly added vertex
		 */
		IdType addMeasurement(Measurement::Ptr m);

		/**
		 * @brief Add a new measurement from another robot.
		 * @details The new measurement is added to the graph and directly
		 * linked to the measurement with the given uuid. This enforces that
		 * the graph stays connected even when external measurement cannot be
		 * linked to local ones.
		 * @param measurement pointer to a new measurement
		 * @param source_uuid uuid of another measurement
		 * @param twc transform between measurement and source
		 * @param sensor name of sensor that created the constraint (not the measurement!)
		 */
		virtual void addExternalMeasurement(Measurement::Ptr measurement,
		                                    boost::uuids::uuid source_uuid,
		                                    const Transform& transform,
		                                    const Covariance<6>& information,
		                                    const std::string& sensor);

		/**
		 * @brief Add a constraint from another robot between two measurements.
		 * @param source uuid of a measurement
		 * @param target uuid of a measurement
		 * @param twc transform from source to target
		 * @param sensor name of sensor that created the constraint
		 */
		void addExternalConstraint(boost::uuids::uuid source,
		                           boost::uuids::uuid target,
		                           const Transform& transform,
		                           const Covariance<6>& information,
		                           const std::string& sensor);
		/**
		 * @brief Get the current pose of the robot within the generated map.
		 * @details The pose is updated at least whenever a new node is added.
		 * @return current robot pose in map coordinates
		 */
		virtual Transform getCurrentPose();

		/**
		 * @brief Get the id of the last vertex that was locally added to the graph.
		 * @details This will not return external vertices from other robots.
		 * @return vertex id
		 */
		virtual IdType getLastIndex() const { return mLastIndex; }

		/**
		 * @brief Get a measurement for a given vertex id. 
		 * @param id
		 * @return measurement
		 */
		virtual Measurement::Ptr getMeasurement(IdType id);

	protected:
		SensorList mSensors;
		PoseSensorList mPoseSensors;
		Logger* mLogger;
		Graph* mGraph;
		IdType mLastIndex;
		Transform mStartPose;
		std::shared_ptr<MeasurementStorage> mMeasurements;
	};
}
