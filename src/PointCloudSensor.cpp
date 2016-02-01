#include "PointCloudSensor.hpp"
#include "GraphMapper.hpp"

#include <pcl/registration/gicp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/radius_outlier_removal.h>

#include <boost/format.hpp>

using namespace slam3d;

typedef pcl::GeneralizedIterativeClosestPoint<PointType, PointType> ICP;

PointCloudSensor::PointCloudSensor(const std::string& n, Logger* l, const Transform& p)
 : Sensor(n, l, p)
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

TransformWithCovariance PointCloudSensor::calculateTransform(Measurement* source, Measurement* target, Transform odometry) const
{
	// Transform guess in sensor frame
	Transform guess = source->getInverseSensorPose() * odometry * target->getSensorPose();
	
	// Cast to this sensors measurement type
	PointCloudMeasurement* sourceCloud = dynamic_cast<PointCloudMeasurement*>(source);
	PointCloudMeasurement* targetCloud = dynamic_cast<PointCloudMeasurement*>(target);
	if(!sourceCloud || !targetCloud)
	{
		mLogger->message(ERROR, "Measurement given to calculateTransform() is not a PointCloud!");
		throw BadMeasurementType();
	}
	
	// Downsample the scans
	PointCloud::Ptr filtered_source = downsample(sourceCloud->getPointCloud(), mConfiguration.point_cloud_density);
	PointCloud::Ptr filtered_target = downsample(targetCloud->getPointCloud(), mConfiguration.point_cloud_density);
	
	// Configure Generalized-ICP
	ICP icp;
	icp.setMaxCorrespondenceDistance(mConfiguration.max_correspondence_distance);
	icp.setMaximumIterations(mConfiguration.maximum_iterations);
	icp.setTransformationEpsilon(mConfiguration.transformation_epsilon);
	icp.setEuclideanFitnessEpsilon(mConfiguration.euclidean_fitness_epsilon);
	icp.setCorrespondenceRandomness(mConfiguration.correspondence_randomness);
	icp.setMaximumOptimizerIterations(mConfiguration.maximum_optimizer_iterations);
	icp.setRotationEpsilon(mConfiguration.rotation_epsilon);
	
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
	if(!icp.hasConverged() || icp.getFitnessScore() > mConfiguration.max_fitness_score)
	{
		mLogger->message(WARNING, (boost::format("ICP failed! (Fitness-Score: %1% > %2%)") % icp.getFitnessScore() % mConfiguration.max_fitness_score).str());
		throw NoMatch();
	}
	
	// Get estimated transform
	Eigen::Isometry3f tf_matrix(icp.getFinalTransformation());
	Transform icp_result = Transform(tf_matrix) * guess;

	// Transform back to robot frame
	TransformWithCovariance twc;
	twc.transform = source->getSensorPose() * icp_result * target->getInverseSensorPose();
	twc.covariance = Covariance::Identity();
	return twc;
}

PointCloud::Ptr PointCloudSensor::getAccumulatedCloud(const VertexObjectList& vertices)
{
	PointCloud::Ptr accu(new PointCloud);
	for(VertexObjectList::const_reverse_iterator it = vertices.rbegin(); it != vertices.rend(); it++)
	{
		PointCloudMeasurement* pcl = dynamic_cast<PointCloudMeasurement*>(it->measurement);
		if(!pcl)
		{
			mLogger->message(ERROR, "Measurement in getAccumulatedCloud() is not a point cloud!");
			throw BadMeasurementType();
		}
		
		PointCloud::Ptr tempCloud(new PointCloud);
		pcl::transformPointCloud(*(pcl->getPointCloud()), *tempCloud, (it->corrected_pose * pcl->getSensorPose()).matrix());
		*accu += *tempCloud;
	}
	return accu;
}