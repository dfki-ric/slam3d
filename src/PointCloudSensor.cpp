#include "PointCloudSensor.hpp"

#include "pcl/registration/gicp.h"

using namespace slam;

PointCloudSensor::PointCloudSensor(std::string n, GraphMapper* m, Logger* l)
 : Sensor(n, m, l)
{
	
}

PointCloudSensor::~PointCloudSensor()
{

}

void PointCloudSensor::addPointCloud(PointCloud cloud)
{
	
}

TransformWithCovariance PointCloudSensor::calculateTransform(const PointCloud* source, const PointCloud* target) const
{
	// Configure Generalized-ICP
	typedef pcl::GeneralizedIterativeClosestPoint<PointType, PointType> GICP;
	GICP icp;
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
	
	icp.setInputSource(boost::shared_ptr<const PointCloud>(source));
	icp.setInputTarget(boost::shared_ptr<const PointCloud>(target));
	PointCloud result;
	icp.align(result);

	// Get estimated transform
	TransformWithCovariance twc;
	if(icp.hasConverged() && icp.getFitnessScore() <= mConfiguration.max_fitness_score)
	{
		GICP::Matrix4 tf_matrix = icp.getFinalTransformation();

		// check for nan values (why does this happen?)
		bool valid = (tf_matrix.array() == tf_matrix.array()).all();
		if(!valid)
		{
			mLogger->message(ERROR, "Messurement from ICP contains not numerical values.");
		}else
		{
			twc.transform = tf_matrix.cast<double>();
		}
	}
	return twc;
}