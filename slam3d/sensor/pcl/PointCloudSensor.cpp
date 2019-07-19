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

#include "PointCloudSensor.hpp"

#include <slam3d/core/Mapper.hpp>

#include <pcl/registration/gicp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/radius_outlier_removal.h>

#include <boost/format.hpp>

using namespace slam3d;

typedef pcl::GeneralizedIterativeClosestPoint<PointType, PointType> GICP;

PointCloudSensor::PointCloudSensor(const std::string& n, Logger* l)
 : ScanSensor(n, l)
{
}

PointCloudSensor::~PointCloudSensor()
{

}

PointCloud::Ptr PointCloudSensor::downsample(PointCloud::ConstPtr in, double leaf_size) const
{
	PointCloud::Ptr out(new PointCloud);
	pcl::VoxelGrid<PointType> grid;
	grid.setLeafSize (leaf_size, leaf_size, leaf_size);
	grid.setInputCloud(in);
	grid.filter(*out);
	return out;
}

PointCloud::Ptr PointCloudSensor::removeOutliers(PointCloud::ConstPtr in, double radius, unsigned min_neighbors) const
{
	PointCloud::Ptr out(new PointCloud);
	pcl::RadiusOutlierRemoval<PointType> out_removal;
	out_removal.setInputCloud(in);
	out_removal.setRadiusSearch(radius);
	out_removal.setMinNeighborsInRadius(min_neighbors);
	out_removal.filter(*out);
	return out;
}

PointCloud::Ptr PointCloudSensor::transform(PointCloud::ConstPtr source, const Transform tf) const
{
	PointCloud::Ptr transformedCloud(new PointCloud);
	pcl::transformPointCloud(*source, *transformedCloud, tf.matrix());
	return transformedCloud;
}

PointCloud::Ptr PointCloudSensor::getAccumulatedCloud(const VertexObjectList& vertices) const
{
	PointCloud::Ptr accu(new PointCloud);
	for(VertexObjectList::const_reverse_iterator it = vertices.rbegin(); it != vertices.rend(); it++)
	{
		PointCloudMeasurement::Ptr pcl = boost::dynamic_pointer_cast<PointCloudMeasurement>(it->measurement);
		if(!pcl)
		{
			mLogger->message(ERROR, "Measurement in getAccumulatedCloud() is not a point cloud!");
			throw BadMeasurementType();
		}
		
		PointCloud::Ptr tempCloud = transform(pcl->getPointCloud(), (it->corrected_pose * pcl->getSensorPose()));
		*accu += *tempCloud;
	}
	return accu;
}

Measurement::Ptr PointCloudSensor::createCombinedMeasurement(const VertexObjectList& vertices, Transform pose) const
{
	PointCloud::Ptr cloud = getAccumulatedCloud(vertices);
	PointCloud::Ptr shifted(new PointCloud);
	pcl::transformPointCloud(*cloud, *shifted, pose.inverse().matrix());
	mLogger->message(DEBUG, (boost::format("Patch pointcloud has %1% points.") % cloud->size()).str());
	Measurement::Ptr m(new PointCloudMeasurement(shifted, "AccumulatedPointcloud", mName, Transform::Identity()));
	return m;
}


Constraint::Ptr PointCloudSensor::createConstraint(const Measurement::Ptr& source,
                                                   const Measurement::Ptr& target,
                                                   const Transform& odometry)
{
	// Transform guess in sensor frame
	Transform guess = source->getInverseSensorPose() * odometry * target->getSensorPose();
	
	// Cast to this sensors measurement type
	PointCloudMeasurement::Ptr sourceCloud = boost::dynamic_pointer_cast<PointCloudMeasurement>(source);
	PointCloudMeasurement::Ptr targetCloud = boost::dynamic_pointer_cast<PointCloudMeasurement>(target);
	if(!sourceCloud || !targetCloud)
	{
		mLogger->message(ERROR, "Measurement given to createConstraint() is not a PointCloud!");
		throw BadMeasurementType();
	}
	
	// Set GICP configuration
	GICPConfiguration config;
//	if(coarse)
//	{
//		config = mCoarseConfiguration;
//	}else
//	{
		config = mFineConfiguration;
//	}
	
	// Downsample the scans
	PointCloud::Ptr filtered_source = downsample(sourceCloud->getPointCloud(), config.point_cloud_density);
	PointCloud::Ptr filtered_target = downsample(targetCloud->getPointCloud(), config.point_cloud_density);
	
	// Make sure that there are enough points left (ICP will crash if not)
	if(filtered_target->size() < config.correspondence_randomness || filtered_source->size() < config.correspondence_randomness)
		throw NoMatch("ICP has too few points");
	
	// Configure Generalized-ICP
	GICP icp;
	icp.setMaxCorrespondenceDistance(config.max_correspondence_distance);
	icp.setMaximumIterations(config.maximum_iterations);
	icp.setTransformationEpsilon(config.transformation_epsilon);
	icp.setEuclideanFitnessEpsilon(config.euclidean_fitness_epsilon);
	icp.setCorrespondenceRandomness(config.correspondence_randomness);
	icp.setMaximumOptimizerIterations(config.maximum_optimizer_iterations);
	icp.setRotationEpsilon(config.rotation_epsilon);
	
	// We cannot use the "guess" parameter from align() due to a bug in PCL.
	// Instead we have to shift the source cloud to the target frame before
	// calling align on it.
	// TODO: Change once the issue in PCL is resolved:
	// > https://github.com/PointCloudLibrary/pcl/pull/989
	PointCloud::Ptr shifted_target(new PointCloud);
	pcl::transformPointCloud(*filtered_target, *shifted_target, guess.matrix());
	
	// Source and target are switched at this point!
	// In the pose graph, our edge (with transform) goes from source to target,
	// but ICP calculates the transformation from target to source.
	icp.setInputSource(shifted_target);
	icp.setInputTarget(filtered_source);
	PointCloud result;
	icp.align(result);

	// Check if ICP was successful (kind of...)
	if(!icp.hasConverged() || icp.getFitnessScore() > config.max_fitness_score)
	{
		throw NoMatch((boost::format("ICP failed with Fitness-Score %1% > %2%") % icp.getFitnessScore() % config.max_fitness_score).str());
	}
	
	// Get estimated transform
	Eigen::Isometry3f tf_matrix(icp.getFinalTransformation());
	Transform icp_result = Transform(tf_matrix) * guess;

	// Transform back to robot frame
	TransformWithCovariance twc;
	twc.transform = source->getSensorPose() * icp_result * target->getInverseSensorPose();
	twc.covariance = Covariance<6>::Identity() * mCovarianceScale;
	
	return Constraint::Ptr(new SE3Constraint(mName, twc));
}

/*
void PointCloudSensor::closeLoop(IdType source_id, IdType target_id)
{
	VertexObject source = mMapper->getGraph()->getVertex(source_id);
	VertexObject target = mMapper->getGraph()->getVertex(target_id);

	if(source.measurement->getSensorName() != mName || target.measurement->getSensorName() != mName)
	{
		throw InvalidEdge(source_id, target_id);
	}
	
	Measurement::Ptr source_m = buildPatch(source_id);
	Measurement::Ptr target_m = buildPatch(target_id);
	
	// Estimate the transform from source to target
	TransformWithCovariance guess = mMapper->getGraph()->getTransform(source_id, target_id);
	TransformWithCovariance twc_coarse = calculateTransform(source_m, target_m, guess, true);
	TransformWithCovariance twc = calculateTransform(source_m, target_m, twc_coarse);

	// Create new edge and return the transform
	if(!twc.isValid())
	{
		throw NoMatch((boost::format("transform determinant is %1%") % twc.transform.matrix().determinant()).str());
	}
	SE3Constraint::Ptr se3(new SE3Constraint(mName, twc));
	mMapper->getGraph()->addConstraint(source_id, target_id, se3);
}
*/