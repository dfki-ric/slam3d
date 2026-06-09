#include "MultiScanSensor.hpp"

#include <slam3d/core/Mapper.hpp>

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

Transform align(MultiScanMeasurement::Ptr source,
                MultiScanMeasurement::Ptr target,
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


MultiScanMeasurement::MultiScanMeasurement(const std::vector<slam3d::PointCloudMeasurement::Ptr>& clouds, const std::string& r, const std::string& s, const boost::uuids::uuid id) : PointCloudMeasurement(std::make_shared<PointCloud>(),r, s, slam3d::Transform::Identity(), id), clouds(clouds) {

	
	// todo make this transform if frames differ
	for (const auto& subcloud : clouds) {
		PointCloud::Ptr tempCloud;
		if (!subcloud->getSensorPose().isApprox(slam3d::Transform::Identity())) {
			tempCloud = PointCloud::Ptr(new PointCloud);
			pcl::transformPointCloud(*subcloud->getPointCloud(), *tempCloud, subcloud->getSensorPose().matrix());
		} else {
			tempCloud = subcloud->getPointCloud();
		}
		*mPointCloud.get() += *tempCloud.get();
	}
	
	mPointCloud->header = clouds[0]->getPointCloud()->header;
	mStamp.tv_sec  = clouds[0]->getPointCloud()->header.stamp / 1000000;
	mStamp.tv_usec = clouds[0]->getPointCloud()->header.stamp % 1000000;
}


const PointCloud::Ptr MultiScanMeasurement::getCombinedPointCloud(const std::string& annotation) {
    PointCloud::Ptr combined(new PointCloud);
    for (unsigned int i = 0; i < clouds.size(); ++i) {
        if (annotation == "" || annotation == annotations[i]) {
            *combined += *(clouds[i]->getPointCloud());
        }
    }
    return combined;
}

const std::vector<PointCloud::Ptr> MultiScanMeasurement::getCloudsByAnnotation(const std::string &annotation) {
    std::vector<PointCloud::Ptr> result;
    for (unsigned int i = 0; i < clouds.size(); ++i) {
        if (annotation == annotations[i]) {
            result.push_back(clouds[i]->getPointCloud());
        }
    }
    return result;
}

PointCloud::Ptr MultiScanSensor::transform(PointCloud::ConstPtr source, const Transform tf) const {
    PointCloud::Ptr transformedCloud(new PointCloud);
    pcl::transformPointCloud(*source, *transformedCloud, tf.matrix());
    return transformedCloud;
}


PointCloud::Ptr MultiScanSensor::getAccumulatedCloud(const VertexObjectList& vertices) const {
    PointCloud::Ptr accu(new PointCloud);

    for (VertexObjectList::const_reverse_iterator it = vertices.rbegin(); it != vertices.rend(); it++) {
        Measurement::Ptr m = mMapper->getGraph()->getMeasurement(it->measurementUuid);
        MultiScanMeasurement::Ptr pcl = boost::dynamic_pointer_cast<MultiScanMeasurement>(m);
        if (!pcl) {
            mLogger->message(ERROR, "Measurement in getAccumulatedCloud() is not a MultiScanMeasurement cloud!");
            throw slam3d::BadMeasurementType();
        }
        PointCloud::Ptr tempCloud = this->transform(pcl->getCombinedPointCloud(), (it->correctedPose * pcl->getSensorPose()));
        *accu += *tempCloud;
    }
    return accu;
}

Measurement::Ptr MultiScanSensor::createCombinedMeasurement(const VertexObjectList& vertices, Transform pose) const {
	printf("%s:%i\n", __PRETTY_FUNCTION__, __LINE__);
	PointCloud::Ptr cloud = getAccumulatedCloud(vertices);
	PointCloud::Ptr shifted(new PointCloud);
	pcl::transformPointCloud(*cloud, *shifted, pose.inverse().matrix());
	mLogger->message(DEBUG, (boost::format("Patch MultiScanMeasurement has %1% points.") % cloud->size()).str());
	std::vector<slam3d::PointCloudMeasurement::Ptr> clouds;
	PointCloudMeasurement::Ptr pcm (new PointCloudMeasurement(shifted, "AccumulatedPointcloud", mName, Transform::Identity()));
	clouds.push_back(pcm);
	Measurement::Ptr m(new MultiScanMeasurement(clouds, "AccumulatedPointcloud", mName));
	return m;
}


Constraint::Ptr MultiScanSensor::createConstraint(const Measurement::Ptr& source,
                                             const Measurement::Ptr& target,
                                             const Transform& odometry,
                                             bool loop)
{

    // Transform guess in sensor frame
    Transform guess = source->getInverseSensorPose() * odometry * target->getSensorPose();

    // Cast to this sensors measurement type
    MultiScanMeasurement::Ptr sourceCloud = boost::dynamic_pointer_cast<MultiScanMeasurement>(source);
    MultiScanMeasurement::Ptr targetCloud = boost::dynamic_pointer_cast<MultiScanMeasurement>(target);
    if (!sourceCloud || !targetCloud) {
        mLogger->message(ERROR, "Measurement given to createConstraint() is not a MultiScanMeasurement!");
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



void MultiScanSensor::setRegistrationParameters(const RegistrationParameters& conf, bool coarse)
{
	if(coarse)
	{
		mLogger->message(INFO, " = RegistrationParameters (Coarse) =");
		mCoarseConfiguration = conf;
	}else
	{
		mLogger->message(INFO, " = RegistrationParameters (Fine) =");
		mFineConfiguration = conf;
	}
	mLogger->message(INFO, (boost::format("correspondence_randomness:    %1%") % conf.correspondence_randomness).str());
	mLogger->message(INFO, (boost::format("euclidean_fitness_epsilon:    %1%") % conf.euclidean_fitness_epsilon).str());
	mLogger->message(INFO, (boost::format("max_correspondence_distance:  %1%") % conf.max_correspondence_distance).str());
	mLogger->message(INFO, (boost::format("max_fitness_score:            %1%") % conf.max_fitness_score).str());
	mLogger->message(INFO, (boost::format("maximum_iterations:           %1%") % conf.maximum_iterations).str());
	mLogger->message(INFO, (boost::format("maximum_optimizer_iterations: %1%") % conf.maximum_optimizer_iterations).str());
	mLogger->message(INFO, (boost::format("point_cloud_density:          %1%") % conf.point_cloud_density).str());
	mLogger->message(INFO, (boost::format("rotation_epsilon:             %1%") % conf.rotation_epsilon).str());
	mLogger->message(INFO, (boost::format("transformation_epsilon:       %1%") % conf.transformation_epsilon).str());
}

void MultiScanSensor::setMapResolution(double r)
{
	mLogger->message(INFO, (boost::format("map_resolution:         %1%") % r).str());
	mMapResolution = r;
}

void MultiScanSensor::setMapOutlierRemoval(double r, unsigned n)
{
	mLogger->message(INFO, (boost::format("map_outlier_radius:     %1%") % r).str());
	mLogger->message(INFO, (boost::format("map_outlier_neighbors:  %1%") % n).str());
	mMapOutlierRadius = r;
	mMapOutlierNeighbors = n;
}


}  // namespace slam3d





