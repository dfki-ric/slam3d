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

#include <slam3d/core/Graph.hpp>
#include <slam3d/core/ScanSensor.hpp>
#include <slam3d/core/PoseSensor.hpp>
#include <slam3d/sensor/pcl/RegistrationParameters.hpp>
#include <slam3d/sensor/pcl/PointCloudMeasurement.hpp>

namespace slam3d
{
	/**
	 * @class PointCloudSensor
	 * @brief Plugin for the mapper that manages point cloud measurements.
	 */
	class PointCloudSensor : public ScanSensor
	{
	public:
		/**
		 * @brief Constructor
		 * @param n unique name of this sensor (used to identify measurements)
		 * @param l pointer to a Logger to write messages
		 */
		PointCloudSensor(const std::string& n, Logger* l);
		
		/**
		 * @brief Destructor
		 */
		~PointCloudSensor();
		
		/**
		 * @brief Create a virtual measurement by accumulating pointclouds from given vertices.
		 * @param vertices list of vertices that should contain a PointCloudMeasurement
		 * @param pose origin of the accumulated pointcloud
		 * @throw BadMeasurementType
		 */		
		Measurement::Ptr createCombinedMeasurement(const VertexObjectList& vertices, Transform pose) const override;
		
		/**
		 * @brief Create an ICP constraint between two point clouds.
		 * @param source
		 * @param target
		 * @param odometry
		 * @param loop whether this is a loop closure (true) or sequential match (false)
		 */
		virtual Constraint::Ptr createConstraint(const Measurement::Ptr& source,
		                                         const MetaData& source_data,
		                                         const Measurement::Ptr& target,
		                                         const MetaData& target_data,
		                                         const Transform& odometry,
		                                         bool loop) override;
		
		/**
		 * @brief Sets parameters for the internal pointcloud registration.
		 * The standard set is always used to calculate the final transformation.
		 * A coarse set is used to initialize and verify loop-closures.
		 * @param param new configuration paramerters
		 * @param coarse whether to set coarse parameter set
		 */
		void setRegistrationParameters(const RegistrationParameters& param, bool coarse);
		
		/**
		 * @brief Set density of points in input scans.
		 * @param r
		 */
		void setScanResolution(double r);
		
		/**
		 * @brief Set density of points in accumulated map cloud.
		 * @param r
		 */
		void setMapResolution(double r);
		
		/**
		 * @brief Set parameters for outlier removal.
		 * @details An outlier is a point that has less then n neighbors within radius r.
		 * @param r
		 * @param n
		 */
		void setMapOutlierRemoval(double r, unsigned n);
		
		/**
		 * @brief Set parameters for crop box filter.
		 * @param min
		 * @param max
		 */
		void setMapCropBox(const Eigen::Vector4f& min, const Eigen::Vector4f& max);
		
		/**
		 * @brief Reduces the size of the source cloud by sampling with the given resolution.
		 * @param source
		 * @param resolution 
		 */
		static PointCloud::Ptr downsample(PointCloud::Ptr source, double resolution);
		
		/**
		 * @brief Crop the source cloud to a square box with given corners.
		 * @param source
		 * @param min
		 * @param max 
		 */
		static PointCloud::Ptr crop(PointCloud::Ptr in, const Eigen::Vector4f& min, const Eigen::Vector4f& max);
		
		/**
		 * @brief Reduces the size of the source cloud by sampling with internal scan resolution.
		 * @details This method might seem unneccessary and only exists so that ScanResolution
		 * can be an internal parameter of PointCloudSensor. It should be called before a scan is 
		 * given to addMeasurement().
		 * @param source
		 */
		PointCloud::Ptr downsampleScan(PointCloud::Ptr source);
		
		/**
		 * @brief Transform source cloud by given transformation.
		 * @param source
		 * @param tf
		 */
		PointCloud::Ptr transform(PointCloud::ConstPtr source, const Transform tf) const;
		
		/**
		 * @brief Removes outliers from given pointcloud.
		 * @details A point is considered an outlier if it has less then min_neighbors within radius.
		 * @param source
		 * @param radius
		 * @param min_neighbors
		 */
		PointCloud::Ptr removeOutliers(PointCloud::Ptr source, double radius, unsigned min_neighbors) const;
		
		/**
		 * @brief Creates a single point cloud that contains all measurements in vertices.
		 * @details The individual point clouds are transformed by their current pose in the graph,
		 * no additional alignement or optimization is performed during this.
		 * @param vertices
		 * @return accumulated pointcloud
		 * @throw BadMeasurementType
		 */
		PointCloud::Ptr getAccumulatedCloud(const VertexObjectList& vertices) const;
		
		/**
		 * @struct MapConfig
		 * @brief configuration parameters for map building
		 */
		struct MapConfig
		{
			// Default config performs no filtering
			MapConfig():resolution(0.0),outlierRadius(0.0),outlierNeighbors(0)
			{
				cropBoxMin.setConstant(-std::numeric_limits<float>::infinity());
				cropBoxMax.setConstant( std::numeric_limits<float>::infinity());
			}
			
			double   resolution;
			double   outlierRadius;
			unsigned outlierNeighbors;
			Eigen::Vector4f cropBoxMin;
			Eigen::Vector4f cropBoxMax;
		};
		
		/**
		 * @brief Build an accumulated point cloud map from given vertices.
		 * @param vertices
		 */
		PointCloud::Ptr buildMap(const VertexObjectList& vertices) const;
		
		/**
		 * @brief Build an accumulated point cloud map from given vertices with custom map params.
		 * @param vertices
		 * @param config
		 */
		PointCloud::Ptr buildMap(const VertexObjectList& vertices, const MapConfig& config) const;
		
		/**
		 * @brief Fill ground plane around center.
		 * @details Estimates a ground plane within the given cloud using RANSAC 
		 * and fills this plane with additional points within radius around the
		 * origin. If no ground plane exists in the scan, the result is
		 * undefined, e.g. RANSAC will just return any plane in the scan.
		 * @param cloud
		 * @param radius
		 */
		void fillGroundPlane(PointCloud::Ptr cloud, ScalarType radius);
		
		/**
		 * @brief Load a PLY and add it as measurement to the graph.
		 * @param path load ply from this file
		 * @param robot the name which will be added to the measurement
		 */
		void loadPLY(const std::string& path, const std::string& robot);
		
	protected:
		RegistrationParameters mFineConfiguration;
		RegistrationParameters mCoarseConfiguration;
		
		double   mScanResolution;
		MapConfig mMapConfig;
	};
}
