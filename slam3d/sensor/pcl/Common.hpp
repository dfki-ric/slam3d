#pragma once

#ifdef USE_PCLOMP
	#include <pclomp/gicp_omp.h>
	#include <pclomp/ndt_omp.h>
#endif

#include <pcl/registration/gicp.h>
#include <pcl/registration/ndt.h>

#include <slam3d/core/Types.hpp>
#include <slam3d/core/Sensor.hpp>
#include <slam3d/sensor/pcl/PointCloudMeasurement.hpp>
#include <slam3d/sensor/pcl/RegistrationParameters.hpp>

namespace slam3d
{
	/**
	 * @brief Reduces the size of the source cloud by sampling with the given resolution.
	 * @param source
	 * @param resolution 
	 */
	PointCloud::Ptr downsample(PointCloud::Ptr source, double resolution, double maxrange = -1.0);
	
	/**
	 * @brief Crop the source cloud to a square box with given corners.
	 * @param source
	 * @param min
	 * @param max 
	 */
	PointCloud::Ptr crop(PointCloud::Ptr in, const Eigen::Vector4f& min, const Eigen::Vector4f& max);

	pcl::Registration<PointType, PointType>::Ptr createRegistration(const RegistrationParameters& config);

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

	Transform align(
		PointCloudMeasurement::Ptr source,
		PointCloudMeasurement::Ptr target,
		const Transform& guess,
		const RegistrationParameters& config);
}
