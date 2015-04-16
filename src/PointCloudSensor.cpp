#include "PointCloudSensor.hpp"

#include "pcl/registration/gicp.h"
#include <pcl/filters/voxel_grid.h>

#define FLT_SIZE 2.0

using namespace slam;

typedef pcl::GeneralizedIterativeClosestPoint<PointType, PointType> ICP;

PointCloudSensor::PointCloudSensor(std::string n, GraphMapper* m, Logger* l)
 : Sensor(n, m, l)
{
	
}

PointCloudSensor::~PointCloudSensor()
{

}

void PointCloudSensor::addPointCloud(PointCloud::ConstPtr &cloud)
{
	
}

TransformWithCovariance PointCloudSensor::calculateTransform(Measurement* source, Measurement* target) const
{
	mLogger->message(INFO, "PointCloudSensor::calculateTransform()");
	
	// Cast to this sensors measurement type
	PointCloudMeasurement* sourceCloud = dynamic_cast<PointCloudMeasurement*>(source);
	PointCloudMeasurement* targetCloud = dynamic_cast<PointCloudMeasurement*>(target);
	if(!sourceCloud || !targetCloud)
	{
		mLogger->message(ERROR, "Measurement given to calculateTransform() is not a PointCloud!");
		throw BadMeasurementType();
	}
	
	// Downsample the scan
	pcl::VoxelGrid<PointType> grid;
	grid.setLeafSize (FLT_SIZE, FLT_SIZE, FLT_SIZE);
	
	PointCloud::Ptr filtered_source(new PointCloud);
	grid.setInputCloud(sourceCloud->getPointCloud());
	grid.filter(*filtered_source);

	PointCloud::Ptr filtered_target(new PointCloud);
	grid.setInputCloud(PointCloud::ConstPtr(targetCloud->getPointCloud()));
	grid.filter(*filtered_target);
	
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
	
	icp.setInputSource(filtered_source);
	icp.setInputTarget(filtered_target);
	PointCloud result;
	icp.align(result);

	// Get estimated transform
	TransformWithCovariance twc;
	if(icp.hasConverged())// && icp.getFitnessScore() <= mConfiguration.max_fitness_score)
	{
		ICP::Matrix4 tf_matrix = icp.getFinalTransformation();

		// check for nan values (why does this happen?)
		bool valid = (tf_matrix.array() == tf_matrix.array()).all();
		if(!valid)
		{
			mLogger->message(ERROR, "Messurement from ICP contains not numerical values.");
		}else
		{
			twc.transform = tf_matrix.cast<double>();
		}
	}else
	{
		mLogger->message(WARNING, "ICP was not successful!");
	}
	return twc;
}

PointCloud::Ptr PointCloudSensor::getAccumulatedCloud()
{
	PointCloud::Ptr accu(new PointCloud);
	
	return accu;
}