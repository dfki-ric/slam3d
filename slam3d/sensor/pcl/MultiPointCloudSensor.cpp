#include "MultiPointCloudSensor.hpp"

#include <slam3d/core/Mapper.hpp>

#include <algorithm>
#include <utility>

#include <pcl/common/transforms.h>


#ifdef USE_PCLOMP
	#include <pclomp/gicp_omp.h>
	#include <pclomp/ndt_omp.h>
#endif

#include <pcl/registration/gicp.h>
#include <pcl/registration/ndt.h>


namespace slam3d {


template <typename ICP_TYPE>
Transform doICP(PointCloud::Ptr source,
          PointCloud::Ptr target,
          const Transform& guess,
          const RegistrationParameters& config)
{
	ICP_TYPE icp;
	icp.setMaxCorrespondenceDistance(config.max_correspondence_distance);
	icp.setMaximumIterations(config.maximum_iterations);
	icp.setTransformationEpsilon(config.transformation_epsilon);
	icp.setEuclideanFitnessEpsilon(config.euclidean_fitness_epsilon);
	icp.setCorrespondenceRandomness(config.correspondence_randomness);
	icp.setMaximumOptimizerIterations(config.maximum_optimizer_iterations);
	icp.setRotationEpsilon(config.rotation_epsilon);
	
	PointCloud result;
	icp.setInputSource(target);
	icp.setInputTarget(source);
	icp.align(result, guess.matrix().cast<float>());

	// Check if ICP was successful (kind of...)
	double score = icp.getFitnessScore(config.max_correspondence_distance);
	if(!icp.hasConverged() || score > config.max_fitness_score)
	{
		throw NoMatch((boost::format("ICP failed with Fitness-Score %1% > %2%") % score % config.max_fitness_score).str());
	}
	
	// Get estimated transform
	Transform icp_result(Eigen::Isometry3f(icp.getFinalTransformation()));
	return icp_result;
}

template <typename NDT_TYPE>
Transform doNDT(PointCloud::Ptr source,
                PointCloud::Ptr target,
                const Transform& guess,
                const RegistrationParameters& config)
{
	NDT_TYPE ndt;
	ndt.setOulierRatio(config.outlier_ratio);
	ndt.setMaxCorrespondenceDistance(config.max_correspondence_distance);
	ndt.setMaximumIterations(config.maximum_iterations);
	ndt.setTransformationEpsilon(config.transformation_epsilon);
	ndt.setEuclideanFitnessEpsilon(config.euclidean_fitness_epsilon);
	ndt.setStepSize(config.step_size);
	ndt.setResolution(config.resolution);
	
	// Source and target are switched at this point!
	// In the pose graph, our edge (with transform) goes from source to target,
	// but ICP calculates the transformation from target to source.
	ndt.setInputSource(target);
	ndt.setInputTarget(source);
	PointCloud result;
	ndt.align(result, guess.matrix().cast<float>());

	// Check if NDT was successful (kind of...)
	double score = ndt.getFitnessScore(config.max_correspondence_distance);
	if(!ndt.hasConverged() || score > config.max_fitness_score)
	{
		throw NoMatch((boost::format("NDT failed with Fitness-Score %1% > %2%") % score % config.max_fitness_score).str());
	}
	
	// Get estimated transform
	Eigen::Isometry3f tf_matrix(ndt.getFinalTransformation());
	return Transform(tf_matrix);
}

Transform align(PointCloudMeasurement::Ptr source,
                PointCloudMeasurement::Ptr target,
                const Transform& guess,
                const RegistrationParameters& config)
{
	// Downsample the scans
	PointCloud::Ptr filtered_source = source->getPointCloud();
	PointCloud::Ptr filtered_target = target->getPointCloud();
	if(config.point_cloud_density > 0)
	{
		PointCloud::Ptr filtered_source = PointCloudSensor::downsample(source->getPointCloud(), config.point_cloud_density);
		PointCloud::Ptr filtered_target = PointCloudSensor::downsample(target->getPointCloud(), config.point_cloud_density);
	}
	
	// Make sure that there are enough points left (ICP will crash if not)
	if(filtered_target->size() < 100 || filtered_source->size() < 100)
		throw NoMatch("Too few points after filtering, you may have to decrease 'point_cloud_density'.");
	
	// Configure Generalized-ICP
	switch(config.registration_algorithm)
	{
	case GICP:
		return doICP< pcl::GeneralizedIterativeClosestPoint<PointType, PointType> >
			(filtered_source, filtered_target, guess, config);
	case NDT:
		return doNDT< pcl::NormalDistributionsTransform<PointType, PointType> >
			(filtered_source, filtered_target, guess, config);
#ifdef USE_PCLOMP
	case GICP_OMP:
		return doICP< pclomp::GeneralizedIterativeClosestPoint<PointType, PointType> >
			(filtered_source, filtered_target, guess, config);
	case NDT_OMP:
		return doNDT< pclomp::NormalDistributionsTransform<PointType, PointType> >
			(filtered_source, filtered_target, guess, config);
#else
	case GICP_OMP:
	case NDT_OMP:
		throw std::runtime_error("OMP is not available, you need to rebuild SLAM3D with OMP or use another matching algorithm.");
#endif
	default:
		throw std::runtime_error("Unknown registration algorithm specified.");
	}
}

PointCloud::Ptr MultiPointCloudSensor::transform(PointCloud::ConstPtr source, const Transform tf) const {
    PointCloud::Ptr transformedCloud(new PointCloud);
    pcl::transformPointCloud(*source, *transformedCloud, tf.matrix());
    return transformedCloud;
}


namespace {

// Returns true if any of the requested tags is contained in 'have'.
bool hasAnyTag(const std::vector<std::string>& have, const std::vector<std::string>& requested)
{
	for (const auto& tag : requested)
	{
		if (std::find(have.begin(), have.end(), tag) != have.end())
		{
			return true;
		}
	}
	return false;
}

}  // namespace

PointCloud::Ptr MultiPointCloudSensor::getAccumulatedCloud(const VertexObjectList& vertices,
                                                     const std::vector<std::string>& tags) const {
    PointCloud::Ptr accu(new PointCloud);

	// Exceptions must not escape an OpenMP structured block, so record a
	// bad-measurement error in a shared flag and throw after the parallel region.
	bool badMeasurementType = false;

	#pragma omp parallel for
	for (size_t i = 0; i < vertices.size(); ++i)
	{
		// Once a bad measurement was seen, cheaply skip the remaining iterations
		// (we cannot break out of an OpenMP for loop).
		#pragma omp flush(badMeasurementType)
		if (badMeasurementType)
		{
			continue;
		}

		const VertexObject& vertex = vertices[i];

		// Decide which measurements of this vertex to accumulate:
		// - No tag filter, or the vertex itself carries a requested tag:
		//   use the vertex' own (combined) measurement, positioned by correctedPose.
		// - Otherwise the vertex is ignored and the tags are searched in its
		//   subMeasurements; every matching subMeasurement is accumulated,
		//   positioned by correctedPose * its own sensor pose.
		// - Main VertexObject has no measurement and no tags given: use combined subs
		std::vector<std::pair<Measurement::Ptr, Transform>> selected;
		Measurement::Ptr m = mMapper->getGraph()->getMeasurement(vertex.measurementUuid);
		if (m && (tags.empty() || hasAnyTag(vertex.tags, tags)))
		{
			selected.emplace_back(m, vertex.correctedPose);
		}
		else
		{
			for (const auto& sub : vertex.subMeasurements)
			{
				if ((!m && tags.size() == 0)|| hasAnyTag(sub.tags, tags))
				{
					Measurement::Ptr sm = mMapper->getGraph()->getMeasurement(sub.measurementUuid);
					selected.emplace_back(sm, vertex.correctedPose * m->getSensorPose());
				}
			}
		}

		for (const auto& sel : selected)
		{
			PointCloudMeasurement::Ptr pcl = boost::dynamic_pointer_cast<PointCloudMeasurement>(sel.first);
			if(!pcl)
			{
				mLogger->message(ERROR, "Measurement in getAccumulatedCloud() is not a point cloud!");
				#pragma omp atomic write
				badMeasurementType = true;
				break;
			}

			PointCloud::Ptr tempCloud = transform(pcl->getPointCloud(), sel.second);

			#pragma omp critical
			*accu += *tempCloud;
		}
	}

	if (badMeasurementType)
	{
		throw BadMeasurementType();
	}
	return accu;
}

Measurement::Ptr MultiPointCloudSensor::createCombinedMeasurement(const VertexObjectList& vertices, Transform pose) const {
	PointCloud::Ptr cloud = getAccumulatedCloud(vertices);
	PointCloud::Ptr shifted(new PointCloud);
	pcl::transformPointCloud(*cloud, *shifted, pose.inverse().matrix());
	mLogger->message(DEBUG, (boost::format("Patch MultiScanMeasurement has %1% points.") % cloud->size()).str());
	PointCloudMeasurement::Ptr pcm (new PointCloudMeasurement(shifted, "AccumulatedPointcloud", mName, Transform::Identity()));


	// Measurement::Ptr m(new PointCloudMeasurement(clouds, "AccumulatedPointcloud", mName, Transform::Identity()));
	return pcm;
}


Constraint::Ptr MultiPointCloudSensor::createConstraint(const Measurement::Ptr& source,
                                             const Measurement::Ptr& target,
                                             const Transform& odometry,
                                             bool loop)
{

    // Transform guess in sensor frame
    Transform guess = source->getInverseSensorPose() * odometry * target->getSensorPose();

    // Cast to this sensors measurement type
    PointCloudMeasurement::Ptr sourceCloud = boost::dynamic_pointer_cast<PointCloudMeasurement>(source);
    PointCloudMeasurement::Ptr targetCloud = boost::dynamic_pointer_cast<PointCloudMeasurement>(target);
    if (!sourceCloud || !targetCloud) {
        mLogger->message(ERROR, "Measurement given to createConstraint() is not a PointCloudMeasurement!");
        throw BadMeasurementType();
    }

    // For large loops, refine guess by a coarse ICP
    if (loop) {
        guess = align(sourceCloud, targetCloud, guess, mCoarseConfiguration);
    }

    // Calculate precise alignement with fine ICP
    Transform icp_result = align(sourceCloud, targetCloud, guess, mFineConfiguration);

    // Transform back to robot frame
    Transform transform = source->getSensorPose() * icp_result * target->getInverseSensorPose();
    Covariance<6> covariance = Covariance<6>::Identity() * mCovarianceScale;

    return Constraint::Ptr(new SE3Constraint(mName, transform, covariance.inverse()));


}

}  // namespace slam3d





